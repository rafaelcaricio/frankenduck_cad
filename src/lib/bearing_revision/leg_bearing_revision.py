"""Left and right Frankenduck knee relief candidates for the V3 adapter."""
import json
from pathlib import Path
from cadgen import build123d as bd
from .runtime import cached_shape as model, read_step
from .collision_bridge_v2 import place, flatten, volume, styled
from .adapter_bridge_v3 import (
    bearing_envelope, adapter_print, servo_adapter_review,
    world_parts, HORN_FRONT_Z, COLLAR_FRONT, cyl,
)

ROOT = Path(__file__).resolve().parents[3]
AXIAL_GAP = .20
LAND_RADIUS = 9.70
POCKET_FLOOR = -.20 - 3.50
HEAD_END_Y = 14.0
BLEND_END_Y = 18.0
REINFORCEMENT_Z = -6.10

def local_source(side):
    data = {i['name']:i for i in json.loads((ROOT/'source/bearing_revision_baseline.json').read_text())['instances']}
    right = side == 'right'
    leg = data['leg_right' if right else 'leg_left']
    servo = data['servo_leg_2' if right else 'servo_leg']
    filename = 'leg_left_native_right.step' if right else 'leg_left_native.step'
    raw = flatten(read_step(str(ROOT/'STEP/parts'/filename)))
    return place(place(raw,leg['at']),servo['at'],inverse=True).translate((0,0,HORN_FRONT_Z))

def box(dx,dy,dz,x=0,y=0,z=0):
    return bd.Box(dx,dy,dz,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((x,y,z))

def yz_prism(points):
    profile=bd.Polygon(*points,align=None)
    face=bd.Plane.YZ * profile
    return bd.extrude(face,amount=20,both=True)

def build_leg(side):
    source=local_source(side)
    bearing=bearing_envelope()
    assert abs(POCKET_FLOOR-(bearing.bounding_box().min.Z-AXIAL_GAP))<1e-6
    # In both servo frames positive local Y runs toward the ankle.
    assert source.bounding_box().max.Y>50,source.bounding_box()
    # Open across the width at the knee. A ramp blends into the original leg
    # below the bearing envelope, so no thin circumferential wall remains.
    relief=yz_prism([(-20,POCKET_FLOOR),(HEAD_END_Y,POCKET_FLOOR),
                     (BLEND_END_Y,-.5),(BLEND_END_Y,6),(-20,6)])
    relief=relief-cyl(LAND_RADIUS,-12,10)
    # Add an outer knee doubler. Use the actual original section at z=-3.9
    # for its outline, so the revision follows each existing handed leg.
    slab=source.intersect(box(70,50,.08,y=0,z=-3.96))
    section_faces=[f for f in slab.faces() if f.geom_type==bd.GeomType.PLANE
                   and abs(f.center().Z+3.96)<1e-5 and abs(f.normal_at().Z)>.99]
    assert section_faces
    face=max(section_faces,key=lambda f:f.area)
    # Rebuild this section's outer outline as a solid blank. Screw and
    # tool-access geometry is recut below using the original dimensions.
    face=bd.Face(face.outer_wire())
    doubler=bd.extrude(face,amount=(-3.96-REINFORCEMENT_Z),dir=(0,0,-1))
    limit=yz_prism([(-20,REINFORCEMENT_Z-.01),(HEAD_END_Y,REINFORCEMENT_Z-.01),
                    (BLEND_END_Y,-3.96),(BLEND_END_Y,-3.85),(-20,-3.85)])
    doubler=bd.Compound(children=list(doubler.intersect(limit).solids()))
    # Ensure overlap with the original section, then retain only the knee.
    doubler=doubler+bd.extrude(face,amount=.06,dir=(0,0,1))
    doubler=doubler.intersect(box(80,38,20,y=-1,z=-12))
    doubler=bd.Compound(children=list(doubler.solids()))
    for x,y in ((7,0),(-7,0),(0,7),(0,-7)):
        doubler=doubler-cyl(1.7,-10,1).translate((x,y,0))
        doubler=doubler-cyl(3.4,-10,-5.295).translate((x,y,0))
    doubler=doubler-cyl(3.2,-10,1)
    result=source.fuse(doubler,tol=1e-5)-relief
    return checked_leg(result.clean(),side)

def checked_leg(result,side):
    bearing=bearing_envelope()
    assert result.is_valid and len(result.solids())==1,(side,result.is_valid,len(result.solids()))
    adapter=adapter_print().translate((0,0,COLLAR_FRONT))
    gap=bearing.distance_to(result)
    assert volume(result.intersect(bearing))<1e-5 and gap>=.1999,(side,gap)
    assert volume(result.intersect(adapter))<1e-5,side
    # Check remaining web at five representative positions between fasteners.
    thickness=[]
    for x,y in ((11,0),(-11,0),(0,11),(0,-11),(8,8)):
        sample=result.intersect(cyl(.05,-12,1).translate((x,y,0)))
        z=sum(s.volume for s in sample.solids())/(3.141592653589793*.05**2)
        thickness.append(round(z,3))
    assert min(thickness)>2.0,(side,thickness)
    print('LEG_REVIEW',json.dumps({'side':side,'bearing_gap_mm':gap,
           'adapter_gap_mm':adapter.distance_to(result),'sample_web_mm':thickness,
           'bearing_overlap_mm3':0,'one_solid':True}))
    result.label=f'{side.title()} leg - knee bearing relief candidate'
    result.color='#3ca997'
    return result

@model
def left_leg():
    # The source legs are handed mirrors. Mirror the verified right solid so
    # the left mesh inherits the same clean knee Boolean.
    return checked_leg(right_leg().mirror(bd.Plane.YZ),'left')

@model
def right_leg():
    return build_leg('right')

@model
def knee_review():
    leg=left_leg()
    bearing=bearing_envelope()
    bearing.label='6704 bearing';bearing.color='#5299e3'
    parts=[p.rotate(bd.Axis.X,180) for p in world_parts(servo_adapter_review())]
    parts.extend([leg,bearing])
    cutter=bd.Box(100,200,150,align=(bd.Align.MAX,bd.Align.CENTER,bd.Align.CENTER))
    complete=[p.rotate(bd.Axis.X,180) for p in parts]
    section=[]
    for part in parts:
        cut=part.intersect(cutter)
        if volume(cut)>1e-6:
            section.append(styled(cut,part.label,part.color).rotate(bd.Axis.X,180).translate((75,0,0)))
    return bd.Compound(children=[
        bd.Compound(children=complete,label='Left knee - installed'),
        bd.Compound(children=section,label='Cutaway through the bearing relief')],
        label='Frankenduck revised knee with V3 adapter')

