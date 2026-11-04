"""Measured 6704 inner-ring interface, mm; canonical inner-ring contact.
22 mm inner-ring OD measured by user. Seating diameters and joint poses stay fixed.
"""
from cadgen import build123d as bd
from .bearing_revision.adapter_bridge_v3 import cyl
from .bearing_revision.collision_bridge_v2 import volume
INNER_RING_OD = 22.0
LIP_OD = 21.6
SHIELD_GAP = .4
BEARING_FRONT = -3.5
HORN_LAND_RADIUS = 9.7

def revise_local(source):
    # Material above the bearing face remains only on the existing horn land.
    # The integral step reaches the inner ring; shield/outer ring are recessed.
    recess = cyl(16, BEARING_FRONT-SHIELD_GAP, -3.499) - cyl(HORN_LAND_RADIUS,-4.1,-3.4)
    body = source-recess
    lip = cyl(LIP_OD/2, BEARING_FRONT-SHIELD_GAP-.05, BEARING_FRONT)-cyl(9.5,-4.1,-3.4)
    result=body.fuse(lip,tol=1e-6).clean()
    removed=volume(source.intersect(recess))
    added=volume(lip)-volume(body.intersect(lip))
    assert abs(volume(result)-(volume(source)-removed+added))<max(.05,volume(source)*1e-5), ("volume balance",volume(source),volume(result),removed,added)
    assert result.is_valid and len(result.solids())==1
    # Preserve the horn seating land, all four screw bores and the centre bore.
    land=cyl(9.49,-20,10)
    old_land=bd.Compound(children=list(source.intersect(land).solids())); new_land=bd.Compound(children=list(result.intersect(land).solids()))
    common=old_land.intersect(new_land)
    assert abs(volume(old_land)-volume(new_land))<1e-5, ("land volumes",volume(old_land),volume(new_land))
    assert abs(volume(common)-volume(old_land))<1e-5, ("land common",volume(common),volume(old_land))
    for x,y in ((7,0),(-7,0),(0,7),(0,-7)):
        assert volume(result.intersect(cyl(1.69,-10,2).translate((x,y,0))))<1e-5
    ring=cyl(11,-3.5,.5)-cyl(10,-3.6,.6)
    shield=cyl(13.5,-3.5,.5)-cyl(11,-3.6,.6)
    assert volume(result.intersect(ring))<1e-5
    assert volume(result.intersect(shield))<1e-5
    assert result.distance_to(ring)<1e-5
    assert result.distance_to(shield)>=.1999 # radial distance to lip edge
    support=result.intersect(cyl(10.79,-3.501,-3.499)-cyl(10.01,-3.502,-3.498))
    assert volume(support)>.04
    return result

TARGETS={
 'hip_left_native': ['servo_upper_leg_left','servo_hip_l'],
 'hip_left_native_right': ['servo_upper_leg_right','servo_hip_l_2'],
 'yaw2roll_left_native': ['servo_yaw2roll'],
 'yaw2roll_left_native_right': ['servo_bearing_roll'],
 'leg_left_native': ['servo_leg'],
 'leg_left_native_right': ['servo_leg_2'],
 'ankle_left_native': ['servo_ankle_left'],
 'ankle_left_native_right': ['servo_ankle_right'],
 'neck_pitch_001_native': ['servo_head_yaw'],
 'yaw_roll_link_native': ['servo_head_roll'],
}

def revise_native(shape,model):
 from pathlib import Path
 import json
 from cadgen import declare_input
 from .bearing_revision.collision_bridge_v2 import place
 path=Path(__file__).resolve().parents[2]/'source/assembly_data.json';declare_input(path)
 items={i['name']:i for i in json.loads(path.read_text())['instances']}
 result=shape.rotate(bd.Axis.Z,-90)
 for servo in TARGETS[model]:
  at=items[servo]['at'];result=place(revise_local(place(result,at,inverse=True)),at)
 return result.rotate(bd.Axis.Z,90)
