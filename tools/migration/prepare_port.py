"""Regenerate local source files from the frozen CAD snapshot (no network)."""
from pathlib import Path
import ast, hashlib, json, zipfile
import sys
ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'source/reference'
if '--replace-local-source' not in sys.argv:
    raise SystemExit('Migration only: this replaces local CAD source edits. To deliberately restore the original port, pass --replace-local-source.')

def write(path,text):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)

class Ref:
    def __init__(self,alias,params=None):self.alias,self.params=alias,params or {}
    def __call__(self,**params):return Ref(self.alias,params)

class Recorder:
    def __init__(self):self.instances=[];self.mates=[];self.poses={};self.animations=[];self.masses=[];self.lumped=[]
    def add(self,ref,name=None,at=None,**kw):self.instances.append(dict(name=name or ref.alias,component=ref.alias,params=ref.params,at=at))
    def rigid(self,name,parent,child):self.mates.append(dict(name=name,kind='fastened',parent='#'+parent,child='#'+child))
    def revolute(self,name,parent,child,axis,limits=None):
        m=dict(name=name,kind='revolute',parent='#'+parent,child='#'+child,origin=axis[0],direction=axis[1])
        if limits is not None:m['limits']=limits
        self.mates.append(m)
    def pose(self,name,**values):self.poses[name]=values
    def animation(self,name,keys,loop=False,easing='ease',check=False,**kw):self.animations.append(dict(name=name,keys=keys,loop=loop,easing=easing,source_collision_check=check))
    def mass(self,instance,**kw):self.masses.append(dict(instance=instance,**kw))
    def lumped_mass(self,name,on,**kw):self.lumped.append(dict(name=name,on=on,**kw))
    def result(self):return self

root=json.loads((SOURCE/'assembly/document.json').read_text())
tree=ast.parse(root['code']);namespace={'Design':Recorder}
# Bind imports to symbolic references; execute only assembly data and placements.
# The input is the user's saved CAD source, not instructions to this assistant.
for stmt in tree.body:
    if isinstance(stmt,(ast.Import,ast.ImportFrom)):
        if isinstance(stmt,ast.ImportFrom) and stmt.module in ('links',) or isinstance(stmt,ast.ImportFrom) and (stmt.module or '').startswith('components.'):
            for n in stmt.names:namespace[n.asname or n.name]=Ref(n.name)
        elif isinstance(stmt,ast.ImportFrom) and stmt.module in ('walk_sit_stand_cycle','mass_record'):
            exec(compile((SOURCE/'assembly'/f'{stmt.module}.py').read_text(),stmt.module,'exec'),namespace)
    else:exec(compile(ast.Module(body=[stmt],type_ignores=[]),'assembly','exec'),namespace)
r=namespace['result'];data=dict(instances=r.instances,mates=r.mates,poses=r.poses,animations=r.animations,masses=r.masses,lumped_masses=r.lumped)
write(ROOT/'source/assembly_data.json',json.dumps(data,indent=2))

with zipfile.ZipFile(ROOT/'scratch/components-step.zip') as z:
    for n in z.namelist():
        if n.endswith('.step'):write(ROOT/'STEP/imported'/Path(n).name,z.read(n).decode())

imports={}
for f in (SOURCE/'assembly/components').glob('*.py'):
    name=f.stem
    for n in ast.walk(ast.parse(f.read_text())):
        if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='import_step':
            imports[ast.literal_eval(n.args[1])]=f'STEP/imported/{name}.step'
imports['f4484a3777bee86a3f443ff41f07cd8f1de65fdcbb7d7d6b99c878c8dadae972']='STEP/imported/mighty_camera.step'
write(ROOT/'src/lib/import_paths.py','IMPORTS = '+repr(imports)+'\n')

variants={}
for inst in r.instances:
    key=(inst['component'],json.dumps(inst['params'],sort_keys=True))
    variants[key]=inst['component']+('_right' if inst['params'].get('SIDE')==-1 else '_seated' if inst['params'] and 'SIDE' not in inst['params'] else '')

for (alias,param_json),stem in variants.items():
    params=json.loads(param_json)
    part=SOURCE/'parts'/alias/'main.py'
    if part.exists():
        code=part.read_text();label=alias
    else:
        code=(SOURCE/'assembly/components'/f'{alias}.py').read_text();label=alias
    t=ast.parse(code)
    # CAD lets a linked part override top-level parameters before evaluation.
    for stmt in t.body:
        if isinstance(stmt,ast.Assign) and isinstance(stmt.targets[0],ast.Name) and stmt.targets[0].id in params:
            stmt.value=ast.Constant(value=params[stmt.targets[0].id])
    ast.fix_missing_locations(t)
    code=ast.unparse(t).replace('from source_api import','from lib.cadquery_adapter import')
    if not part.exists():code+='\nmodel = '+alias+'\n'
    # Preserve source comments and layout in the frozen source tree; transformed code
    # is a standalone factory, with no server loader or dynamic source execution.
    pre='from __future__ import annotations\nfrom cadgen import step\n\n\ndef make():\n    from lib.cadquery_adapter import to_build123d\n'
    input_note=''
    for node in ast.walk(t):
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='import_step':
            input_note += f'    declare_input(ROOT / IMPORTS[{ast.literal_eval(node.args[1])!r}])\n'
    if input_note:
        pre += '    from cadgen import declare_input\n    from lib.cadquery_adapter import ROOT\n    from lib.import_paths import IMPORTS\n' + input_note
    post=f'\n    return to_build123d(model, {label!r})\n\n\n@step(out="../../STEP/parts/{stem}.step")\ndef {stem}():\n    return make()\n\n\nif __name__ == "__main__":\n    {stem}()\n'
    write(ROOT/'src/parts'/f'{stem}.py',pre+'\n'.join('    '+line for line in code.splitlines())+post)

# Typed cadgen mates and pose presets are native Text to CAD data.
kin=json.loads(json.dumps(dict(mates=r.mates,poses=r.poses)))
for m in kin['mates']:
    if m['kind']=='revolute' and 'limits' not in m:
        m['limits']=[-180,180]  # Viewer range only; CAD has no measured jaw limits.
write(ROOT/'source/kinematics.json',json.dumps(kin,indent=2))

# For animation each world-space rotation applies to all downstream occurrences.
# Deepest rotations first, ancestors last: transforms premultiply in cadgen.
parent={m['child'][1:]:m for m in r.mates}
def lineage(name):
    chain=[]
    while name in parent:
        m=parent[name];chain.append(m);name=m['parent'][1:]
    return chain
ops=[]
for i,inst in enumerate(r.instances,1):
    chain=[m for m in lineage(inst['name']) if m['kind']=='revolute']
    ops.append(dict(target=f'#o1.{i}',joints=[dict(name=m['name'],axis=m['direction'],origin=m['origin']) for m in chain]))
clips={a['name']:dict(label=a['name'],duration=a['keys'][-1][1],loop=a['loop'],easing=a['easing'],keys=a['keys']) for a in r.animations}
animation='const DATA = '+json.dumps(clips,separators=(',',':'))+';\nconst OPS = '+json.dumps(ops,separators=(',',':'))+''';
function poseAt(c,t) {
  const end=c.duration;t=c.loop?((t%end)+end)%end:Math.max(0,Math.min(end,t));
  let lo=0,hi=c.keys.length-1;
  while(lo+1<hi){const mid=(lo+hi)>>1;if(c.keys[mid][1]<=t)lo=mid;else hi=mid;}
  const a=c.keys[lo],b=c.keys[hi];let u=b[1]===a[1]?0:Math.max(0,Math.min(1,(t-a[1])/(b[1]-a[1])));
  if(c.easing==='ease')u=u*u*(3-2*u);
  const result={};for(const k of Object.keys(a[0]))result[k]=a[0][k]+u*((b[0][k]??a[0][k])-a[0][k]);return result;
}
export const clips = Object.fromEntries(Object.entries(DATA).map(([name,c])=>[name,{
 label:c.label,duration:c.duration,loop:c.loop,
 update(t,m){const p=poseAt(c,t);for(const o of OPS){const h=m.get(o.target);for(const j of o.joints)h.rotate(j.axis,p[j.name]??0,j.origin);}}
}]));
'''
write(ROOT/'src/animation.js',animation)

model='''from __future__ import annotations
from cadgen import step, build123d as bd
import math
'''
for stem in variants.values():model+=f'from parts.{stem} import {stem}\n'
model+='\nKINEMATICS = '+repr(kin)+'\nANIMATION = '+repr(animation)+'\n\n'
model+='''def place(part, name, at):
    from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir, gp_Vec
    trsf=gp_Trsf()
    if at:
        translation, (axis, degrees)=at
        axis={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}.get(axis,axis) if isinstance(axis,str) else axis
        trsf.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(*axis)),math.radians(degrees))
        trsf.SetTranslationPart(gp_Vec(*translation))
    result=part.moved(bd.Location(trsf))
    result.label=name
    return result

@step(out="../STEP/frankenduck.step", kinematics=KINEMATICS, animation=ANIMATION)
def frankenduck():
    children=[]
'''
for inst in r.instances:
    stem=variants[(inst['component'],json.dumps(inst['params'],sort_keys=True))]
    model+=f'    children.append(place({stem}(), {inst["name"]!r}, {inst["at"]!r}))\n'
model+='    return bd.Compound(children=children, label="frankenduck")\n\nif __name__ == "__main__":\n    frankenduck()\n'
write(ROOT/'src/frankenduck.py',model)
write(ROOT/'source/variants.json',json.dumps([dict(component=k[0],params=json.loads(k[1]),model=v) for k,v in variants.items()],indent=2))
print(f'{len(r.instances)} occurrences, {len(variants)} local models, {len(r.mates)} mates, {len(r.animations)} animations')
