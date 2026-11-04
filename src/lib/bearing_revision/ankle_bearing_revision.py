"""Left ankle horn bracket relief candidate, in the ankle servo frame. mm."""
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model
from .upper_leg_carrier_v10 import instances, imported
from .collision_bridge_v2 import place, volume
from .adapter_bridge_v3 import adapter_print, bearing_envelope, COLLAR_FRONT, HORN_FRONT_Z, cyl

@model
def left_ankle():
    i=instances()
    source=place(imported('ankle_left',i,'#72b6ad'),i['servo_ankle_left']['at'],inverse=True).translate((0,0,HORN_FRONT_Z))
    # Open relief around the whole collar, keeping the original horn land.
    # 0.20 axial bearing clearance; 0.35 radial clearance to the 31.3 collar.
    relief=cyl(16.0,-3.70,5.0)-cyl(9.70,-4,6)
    result=(source-relief).clean()
    assert result.is_valid and len(result.solids())==1
    bearing=bearing_envelope();adapter=adapter_print().translate((0,0,COLLAR_FRONT))
    assert volume(result.intersect(bearing))<1e-5
    assert volume(result.intersect(adapter))<1e-5
    assert bearing.distance_to(result)>=.1999
    land=cyl(9.7,-10,1)
    assert abs(volume(source.intersect(land))-volume(result.intersect(land)))<1e-5
    # Existing mounting bores and counterbores must remain unfilled.
    for x,y in ((7,0),(-7,0),(0,7),(0,-7)):
        assert volume(result.intersect(cyl(1.69,-10,1).translate((x-.0001,y,0))))<1e-5
    samples=[]
    for x,y in ((11,0),(-11,0),(0,-10.8),(8,8),(-8,8),(0,11)):
        g=result.intersect(cyl(.05,-12,-3.699).translate((x,y,0)))
        thickness=volume(g)/(3.141592653589793*.05**2)
        samples.append([x,y,thickness])
    assert min(row[2] for row in samples)>3.39
    print('ANKLE_REVISION',json.dumps({'one_solid':True,'bearing_gap_mm':bearing.distance_to(result),'adapter_gap_mm':adapter.distance_to(result),'section_samples_mm':samples,'removed_mm3':volume(source)-volume(result)}))
    result.label='Revised left ankle - horn bearing relief';result.color='#72b6ad'
    return result

