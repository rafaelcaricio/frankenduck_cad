"""Hip roll and yaw bearing integration candidate. mm; no source assembly motion changes."""
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model
from .upper_leg_carrier_v10 import instances,imported
from .right_leg_bearing_revision import right_import
from .hip_internal_reinforcement_long import left_hip,right_hip
from .adapter_bridge_v3 import servo_adapter_review,world_parts,adapter_print,bearing_envelope,COLLAR_FRONT,cyl,HORN_FRONT_Z
from .collision_bridge_v2 import place,volume,styled
SERVOS=('servo_hip_l','servo_hip_l_2','servo_yaw2roll','servo_bearing_roll')

def parts():
 i=instances()
 hips=[place(fn(),i['servo_upper_leg_'+side]['at']) for side,fn in [('left',left_hip),('right',right_hip)]]
 links=[imported('yaw2roll_left',i,'#62b6a7'),right_import('yaw2roll_right')]
 body=[imported(n,i,'#74988c') for n in ('trunk_base','left_shell_native','right_shell_native')]
 return i,hips,links,body

def hardware(i):
 vendor=[p.rotate(bd.Axis.X,180) for p in world_parts(servo_adapter_review()) if '从动' not in p.label]
 result=[]
 for n in SERVOS:
  for p in vendor:
   q=place(p,i[n]['at']);q.label=n+' / '+p.label;result.append(q)
  b=place(bearing_envelope(),i[n]['at']);b.label=n+' / 6704 bearing';b.color='#468dd5';result.append(b)
 return result

@model
def initial_fit():
 i,hips,links,body=parts();hw=hardware(i)
 report=[]
 for n in SERVOS:
  a=place(adapter_print().translate((0,0,COLLAR_FRONT)),i[n]['at'])
  b=place(bearing_envelope(),i[n]['at'])
  for p in hips+links+body:
   av=volume(a.intersect(p));bv=volume(b.intersect(p))
   if av>1e-5 or bv>1e-5:
    local=place(p,i[n]['at'],inverse=True);bb=local.bounding_box()
    report.append({'servo':n,'part':p.label,'adapter_overlap':av,'bearing_overlap':bv,'local_bounds':[list(bb.min),list(bb.max)]})
 print('HIP_SERVO_INITIAL',json.dumps(report),flush=True)
 return bd.Compound(children=[*hips,*links,*hw],label='Hip roll and yaw - initial bearing fit, body hidden')



@model
def detail_probe():
 i=instances();out=[]
 a=adapter_print().translate((0,0,COLLAR_FRONT))
 for side,yn in [('left','servo_yaw2roll'),('right','servo_bearing_roll')]:
  shell=place(imported(side+'_shell_native',i,'#74988c'),i[yn]['at'],inverse=True)
  hit=place(a,i[yn]['at']).intersect(imported(side+'_shell_native',i,'#74988c'));bb=place(bd.Compound(children=list(hit.solids())),i[yn]['at'],inverse=True).bounding_box()
  src=imported('yaw2roll_left',i,'#62b6a7') if side=='left' else right_import('yaw2roll_right')
  src=place(src,i[yn]['at'],inverse=True)
  faces=[f for f in src.faces() if f.geom_type==bd.GeomType.PLANE and abs(f.center().Z+4)<1e-5 and f.normal_at().Z<-.99]
  print('DETAIL',json.dumps({'side':side,'shell_hit_bounds':[list(bb.min),list(bb.max)],'bottom_faces':[(f.area,list(f.center())) for f in faces]}),flush=True)
  out.append(src)
 return bd.Compound(children=out)



def as_solid(s):
 assert len(s.solids())==1
 return s.solids()[0]

def seat_relief(s):
 return (s-(cyl(16,-3.7,3)-cyl(9.7,-4,4))-cyl(9.7,HORN_FRONT_Z,3)).clean()

def revise_hip(side):
 i=instances();rn='servo_hip_l' if side=='left' else 'servo_hip_l_2'
 src=place((left_hip() if side=='left' else right_hip()),i['servo_upper_leg_'+side]['at'])
 src=as_solid(place(src,i[rn]['at'],inverse=True))
 s=seat_relief(src)
 assert s.is_valid and len(s.solids())==1
 samples=[]
 for x,y in ((11,0),(-11,0),(0,11),(0,-11),(8,8),(-8,8),(8,-8),(-8,-8)):
  samples.append([x,y,volume(s.intersect(cyl(.05,-12,-3.699).translate((x,y,0))))/(3.141592653589793*.05**2)])
 print('ROLL_WALL',side,samples,flush=True)
 # Existing curved shoulder yields 2.246 mm at two diagonal samples.
 assert min(t[2] for t in samples)>2.2,samples
 assert volume(s.intersect(bearing_envelope()))<1e-5
 s=place(s,i[rn]['at']);s.label=side.title()+' hip link - pitch and roll bearing relief';s.color='#72b6ad'
 return s

@model
def left_hip_dual():return revise_hip('left')
@model
def right_hip_dual():return revise_hip('right')

def revise_yaw(side):
 i=instances();yn='servo_yaw2roll' if side=='left' else 'servo_bearing_roll'
 src=imported('yaw2roll_left',i,'#62b6a7') if side=='left' else right_import('yaw2roll_right')
 src=as_solid(place(src,i[yn]['at'],inverse=True))
 faces=[f for f in src.faces() if f.geom_type==bd.GeomType.PLANE and abs(f.center().Z+4)<1e-5 and f.normal_at().Z<-.99]
 assert len(faces)==1
 fill=bd.extrude(faces[0],amount=2.7,dir=(0,0,-1))
 s=seat_relief(src.fuse(fill,tol=1e-6)).clean()
 assert s.is_valid and len(s.solids())==1
 samples=[]
 for x,y in ((11,0),(-11,0),(0,11),(0,-11),(8,8),(-8,8)):
  samples.append(volume(s.intersect(cyl(.05,-7,-3.699).translate((x,y,0))))/(3.141592653589793*.05**2))
 assert min(samples)>2.99,samples
 assert volume(s.intersect(bearing_envelope()))<1e-5
 for x,y in ((7,0),(-7,0),(0,7),(0,-7)):
  assert volume(s.intersect(cyl(1.69,-10,3).translate((x,y,0))))<1e-5
 print('YAW_WALL',side,samples,flush=True)
 s=place(s,i[yn]['at']);s.label=side.title()+' yaw-to-roll bracket - inward reinforced bearing plate';s.color='#62b6a7'
 return s

@model
def left_yaw_bracket():return revise_yaw('left')
@model
def right_yaw_bracket():return revise_yaw('right')

def yaw_adapter_shape(side):
 a=as_solid(adapter_print().translate((0,0,COLLAR_FRONT)))
 # Side relief only behind the bearing seat; shell exterior stays untouched.
 x0=-25 if side=='left' else 13.35
 cut=bd.Box(11.65,40,5,align=(bd.Align.MIN,bd.Align.CENTER,bd.Align.MIN)).translate((x0,0,2.5))
 s=(a-cut).clean()
 assert s.is_valid and len(s.solids())==1
 # Front bearing seat and both screw pads are unchanged.
 protected=bd.Box(60,80,22.49,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((0,0,-20))
 assert volume((a-s).intersect(protected))<1e-5
 s.label=side.title()+' hip yaw adapter - shell side relief';s.color='#f39632'
 return s

@model
def left_yaw_adapter():return yaw_adapter_shape('left')
@model
def right_yaw_adapter():return yaw_adapter_shape('right')

@model
def hip_servos_revised():
 i=instances()
 hips=[left_hip_dual(),right_hip_dual()]
 links=[left_yaw_bracket(),right_yaw_bracket()]
 hw=hardware(i)
 for side,n,fn in [('left','servo_yaw2roll',left_yaw_adapter),('right','servo_bearing_roll',right_yaw_adapter)]:
  hw=[p for p in hw if p.label!=n+' / PRINTED front adapter']
  q=place(fn(),i[n]['at']);q.label=n+' / PRINTED front adapter';hw.append(q)
 body=[imported(n,i,'#74988c') for n in ('trunk_base','left_shell_native','right_shell_native')]
 hits=[]
 for p in hips+links:
  for h in hw:
   if 'horn' in h.label:continue
   v=volume(p.intersect(h))
   if v>1e-4:hits.append([p.label,h.label,v])
 for p in hw:
  if 'PRINTED' not in p.label and 'bearing' not in p.label:continue
  for b in body:
   v=volume(p.intersect(b))
   if v>1e-4:hits.append([p.label,b.label,v])
 print('HIP_SERVO_REVISED_HITS',json.dumps(hits),flush=True)
 assert not hits,hits
 return bd.Compound(children=[*hips,*links,*hw],label='Hip roll and yaw - four bearing adapters, revised links and brackets')

