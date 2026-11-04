"""Head-bearing candidate using the existing STS3215 adapter, in world mm.

Reference: DuckBuddy hardware/rev-a/mjcf/robot_walk.xml: head-yaw bearing
and paired head-roll support. Frankenduck retains its existing passive F688.
Canonical head bearing geometry; physical fit remains untested.
"""
from importlib import import_module
from cadgen import build123d as bd
from .bearing_revision.runtime import cached_shape
from .bearing_revision.collision_bridge_v2 import place, volume
from .bearing_revision.adapter_bridge_v3 import (
    adapter_print, bearing_envelope, COLLAR_FRONT, HORN_FRONT_Z, cyl,
)
from .bearing_revision.factories import servo
from .viewer_palette import paint_original

YAW = ((0., -35.1, 258.0048), ('Z', -90))
ROLL = ((0., -14.935, 280.5798), ((-.5773502691896257, -.5773502691896257, -.5773502691896258), 119.99999999999999))
AXIAL_GAP = .20
RELIEF_RADIUS = 16.0
LAND_RADIUS = 9.7

@cached_shape
def original(module, occurrence):
    module_obj = import_module('parts.' + module)
    factory = getattr(module_obj, 'baseline_make', module_obj.make)
    s = place(factory(), ((0, 0, 0), ('Z', -90)))
    s.label = occurrence
    return paint_original(s, occurrence)

@cached_shape
def base_adapter():
    s = adapter_print().translate((0, 0, COLLAR_FRONT))
    s.color = '#1d1d1f'
    return s


def relieved_link(source, pose):
    s = place(source, pose, inverse=True)
    relief = cyl(RELIEF_RADIUS, -3.5 - AXIAL_GAP, 5) - cyl(LAND_RADIUS, -4, 6)
    result = (s - relief - cyl(LAND_RADIUS, HORN_FRONT_Z, 5)).clean()
    assert result.is_valid and len(result.solids()) == 1
    assert volume(result.intersect(base_adapter())) < 1e-5
    assert volume(result.intersect(bearing_envelope())) < 1e-5
    assert result.distance_to(bearing_envelope()) >= AXIAL_GAP - 1e-5
    # Retain all material behind the recess, preserving the outer supporting web.
    back = bd.Box(200, 200, 200, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX)).translate((0, 0, -3.7))
    assert abs(volume(s.intersect(back)) - volume(result.intersect(back))) < 1e-4
    return place(result, pose)


from .cadquery_adapter import to_build123d

@cached_shape
def yaw_roll_link():
    import cadquery as cq
    s=relieved_link(original('yaw_roll_link_native','yaw_roll_link_r05'),ROLL)
    window=(cq.Workplane('XY').box(9.5,19,12).edges('|Z').fillet(1)
            .translate((-15.95,-35.1,295)))
    s=(s-to_build123d(window,'cable_window')).clean()
    local=place(s,ROLL,inverse=True)
    # Remove the fragile front lip, retaining the rear mounting plate.
    cutter=bd.Box(20,80,10,align=(bd.Align.MIN,bd.Align.CENTER,bd.Align.MIN)).translate((12.6252,0,-3.7))
    s=place((local-cutter).clean(),ROLL)
    assert s.is_valid and len(s.solids())==1
    s.label='yaw_roll_link_r05'
    return paint_original(s,s.label)

@cached_shape
def head_frame():
    # Start from the unfused frame. The adapter now travels with the servo.
    s=original('head_frame_native','head_frame_r05')
    adapter=place(base_adapter(),ROLL)
    # Clearance around the removable adapter, including its assembly seams.
    for delta in [(0,0,0),(.2,0,0),(-.2,0,0),(0,.2,0),(0,-.2,0),(0,0,.2),(0,0,-.2)]:
        s=s-adapter.translate(delta)
    # Open the narrow adapter guide to the top for straight insertion.
    guide=bd.Box(49.05,6.195,50).translate(((-33.2+15.85)/2,(-16.23-10.035)/2,289.73))
    s=(s-guide).clean()
    # Annotation 11: straighten the sloped side-window edge to world Y=-46.
    # Extend only the marked lower/right band; retain rounded corners.
    import cadquery as cq
    window=(cq.Workplane('XY').box(10,6,18.35).edges('|X').fillet(.6)
            .translate((17,-49,285.875)))
    s=(s-to_build123d(window,'side_window_extension')).clean()
    window=(cq.Workplane('XY').box(12.5,14,19).edges('|Y').fillet(1)
            .translate((-17.45,23,280.58)))
    s=(s-to_build123d(window,'wider_roll_plug')).clean()
    assert s.is_valid and len(s.solids())==1
    s.label='head_frame_r05'
    return paint_original(s,s.label)

