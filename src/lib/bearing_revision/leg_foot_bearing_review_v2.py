"""Complete left-leg review with V10 carrier and ankle horn bearing.
Review only; downstream placements follow provisional horn seating offsets.
"""
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model
from .upper_leg_carrier_v10 import instances, imported, carrier_installed_complete
from .collision_bridge_v2 import place, volume, styled
from .adapter_bridge_v3 import servo_adapter_review, world_parts, bearing_envelope, HORN_FRONT_Z
from .leg_bearing_revision import left_leg
from .ankle_bearing_revision import left_ankle

def delta(at):
    axis,angle=at[1]
    axis={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}.get(axis,axis) if isinstance(axis,str) else axis
    return bd.Vector(0,0,HORN_FRONT_Z).rotate(bd.Axis((0,0,0),axis),angle)

def ankle_parts():
    i=instances();kdelta=delta(i['servo_leg']['at']);adelta=delta(i['servo_ankle_left']['at'])
    lower=place(left_leg(),i['servo_leg']['at']);lower.label='Revised lower leg - knee bearing relief';lower.color='#3ca997'
    hardware=[]
    for p in world_parts(servo_adapter_review()):
        if '从动' in p.label: continue
        q=place(p.rotate(bd.Axis.X,180),i['servo_ankle_left']['at']).translate(kdelta)
        q.label='Foot servo / '+p.label
        hardware.append(q)
    bearing=place(bearing_envelope(),i['servo_ankle_left']['at']).translate(kdelta)
    bearing.label='Foot horn / 6704 bearing 20x27x4';bearing.color='#468dd5'
    adapter=next(p for p in hardware if 'PRINTED front adapter' in p.label)
    links=[]
    for name in ('ankle_left','ankle_eye_left','ankle_bearing_outer_keeper_left','ankle_bearing_inner_cap_left','foot_a_left','foot_b_left'):
        p=(place(left_ankle(),i['servo_ankle_left']['at']).translate(kdelta) if name=='ankle_left' else imported(name,i,'#72b6ad').translate(kdelta+adelta))
        links.append(p)
    targets=[lower,*links]
    report=[];red=[]
    for p in targets:
        row={'part':p.label}
        for h,key,color in ((bearing,'bearing','#fa2238'),(adapter,'adapter','#d640ba')):
            common=p.intersect(h);v=volume(common)
            row[key+'_overlap_mm3']=v
            row[key+'_gap_mm']=p.distance_to(h)
            if v>1e-5:
                red.append(styled(common,key+' interference / '+p.label,color))
        report.append(row)
    print('FOOT_FIT',json.dumps({'knee_chain_shift_mm':list(kdelta),'ankle_link_shift_mm':list(adelta),'seating':'provisional vendor datum','parts':report}))
    # Also report hardware/body collisions, distinct from intentionally touching surfaces.
    print('FOOT_SERVO_FIT',json.dumps([{ 'part':p.label,'adapter_overlap_mm3':volume(adapter.intersect(p)),'bearing_overlap_mm3':volume(bearing.intersect(p))} for p in hardware if p is not adapter]))
    return lower,hardware,bearing,links,red

@model
def foot_detail():
    lower,hardware,bearing,links,red=ankle_parts()
    # Expose collision volumes without changing source geometry or placements.
    a=next(p for p in hardware if 'PRINTED front adapter' in p.label)
    context=[styled(p-a-bearing,p.label,p.color) for p in [lower,*links]]
    displayed=[]
    for p in [*hardware,bearing]:
        s=p
        if p is a or p is bearing:
            for link in [lower,*links]:s=s-link
        displayed.append(styled(s,p.label,p.color))
    return bd.Compound(children=[*context,*displayed,*red],label='Foot bearing fit - revised ankle bracket')

@model
def complete_leg():
    upper=carrier_installed_complete()
    lower,hardware,bearing,links,red=ankle_parts()
    return bd.Compound(children=[upper,lower,*hardware,bearing,*links],label='Complete left leg - V10 carrier, revised lower leg, revised ankle and foot bearing')

@model
def foot_section():
    assembly=foot_detail()
    cutter=bd.Box(200,100,75,align=(bd.Align.MIN,)*3).translate((0,42.8991,-35))
    parts=[]
    for p in world_parts(assembly):
        cut=p.intersect(cutter)
        if volume(cut)>1e-6:parts.append(styled(cut,p.label,p.color))
    return bd.Compound(children=parts,label='Revised ankle bearing axial section')

