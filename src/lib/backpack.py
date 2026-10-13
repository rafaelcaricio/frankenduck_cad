"""Exposed PAP 204 A1 backpack with a screwed two-part mount; all helper geometry in robot world mm.

Rear is +Y and up is +Z. Native trunk/shell factories use a frame rotated
+90 degrees about Z; adapters below return their original local frame.
The pack is a visual model: rail/latch fit and printed strength are unverified.
"""
from functools import lru_cache
import math
from cadgen import build123d as bd

TILT_DEG = 20.0
INWARD_MM = 15.0
LIFT_MM = 23.761265
PIVOT = (0.0, 101.2, 126.815)
REFERENCE_BATTERY_AT = ((0, 134.4, 126.815), ((1, -1, 1), 120))
# Compose the original native battery frame with the approved forward lean.
_C = math.cos(math.radians(TILT_DEG))
_S = math.sin(math.radians(TILT_DEG))
_ANGLE = math.acos((-_S - 1)/2)
_AXIS = (_C/(2*math.sin(_ANGLE)), -_C/(2*math.sin(_ANGLE)), (1-_S)/(2*math.sin(_ANGLE)))
BATTERY_AT = ((0, PIVOT[1]-INWARD_MM+33.2*_C, PIVOT[2]+LIFT_MM+33.2*_S), (_AXIS, math.degrees(_ANGLE)))


def incline(shape):
    """Transform reference-upright CQ geometry into robot world coordinates."""
    return shape.rotate(PIVOT, (1,PIVOT[1],PIVOT[2]), TILT_DEG).translate((0,-INWARD_MM,LIFT_MM))


def yz(y,z):
    return (PIVOT[1]-INWARD_MM+(y-PIVOT[1])*_C-(z-PIVOT[2])*_S,
            PIVOT[2]+LIFT_MM+(y-PIVOT[1])*_S+(z-PIVOT[2])*_C)


def face_y(z):
    y0,z0=yz(60,145.5)
    return y0-(z-z0)*_S/_C


def plate_profile():
    import cadquery as cq
    # Rounded nose, full rail-bearing width, and a compact rounded screw shoulder.
    # Entire structural backing shares a flat print face; no unsupported rail forks.
    return (cq.Workplane('XZ',origin=(0,67,0)).moveTo(-12,65.315).lineTo(12,65.315)
            .threePointArc((17.657,67.658),(20,73.315))
            .spline([(24,78),(28,81)],includeCurrent=True).lineTo(28,124)
            .spline([(30,129),(32,135)],includeCurrent=True)
            .lineTo(32,152.5).threePointArc((30.243,156.743),(26,158.5))
            .lineTo(-26,158.5).threePointArc((-30.243,156.743),(-32,152.5))
            .lineTo(-32,135).spline([(-30,129),(-28,124)],includeCurrent=True)
            .lineTo(-28,81).spline([(-24,78),(-20,73.315)],includeCurrent=True)
            .threePointArc((-17.657,67.658),(-12,65.315)).close())


def release_window():
    import cadquery as cq
    return (cq.Workplane('XY').box(32,32,70).translate((0,58,189))
            .edges('|Z').fillet(5).edges('|X or |Y').fillet(2).val())
BATTERY_BOTTOM_Z = 89.46415313
RIB_WIDTH = 18.0
PLATE_THICKNESS = 7.0
RAIL_CHEEK = 5.0
RAIL_LIP_NOMINAL = 4.0
SHELL_ALLOWANCE = 0.8
SHELL_SPLIT_HALF_GAP = 0.3
BATTERY_ALLOWANCE = 0.7
SWITCH_ALLOWANCE = 0.8


def offsets(distance, include_zero=True):
    return ([(0, 0, 0)] if include_zero else []) + [
        tuple(sign * distance if i == axis else 0 for i in range(3))
        for axis in range(3) for sign in (-1, 1)]


def world(shape):
    return bd.Compound(shape.wrapped).moved(bd.Location((0, 0, 0), (0, 0, 1), -90))


def native(shape, label):
    assert shape.is_valid and len(shape.solids()) == 1, label
    result = shape.moved(bd.Location((0, 0, 0), (0, 0, 1), 90))
    result.label = label
    result.color = bd.Color('#99deef')
    return result


def thumb_approach():
    """Open approach from above the red release paddle, not a battery enclosure."""
    import cadquery as cq
    # 18 mm fingertip corridor approaching along the battery slide direction.
    return incline(cq.Solid.makeCylinder(9,60,cq.Vector(0,59,162),cq.Vector(0,0,1)))


# Four M3 x 12 socket-head screws, with M3 hex nuts slid in from the sides.
# Nominal hardware: 5.5 mm nut across flats / 2.4 mm thick; verify actual fit.
JOINT_X = (-24.0, 24.0)
JOINT_Z = (138.5, 150.5)
JOINT_FACE_Y = 60.0
SCREW_CLEARANCE_D = 3.4
HEAD_CLEARANCE_D = 6.4
HEAD_SEAT_Y = 63.8
NUT_SLOT_Y = (53.0, 55.8)
NUT_SLOT_HEIGHT = 5.8


def rib_profile():
    """Compact straight gusset with a continuous roof at the trunk top plane."""
    import cadquery as cq
    return (cq.Workplane('YZ').moveTo(27.75,146.45).lineTo(34,146.45)
            .lineTo(42,137.45).lineTo(face_y(137.45),137.45)
            .lineTo(face_y(164.45),164.45).lineTo(27.75,164.45).close())


@lru_cache(maxsize=1)
def frame_supports():
    import cadquery as cq
    arms = []
    for sign in (-1, 1):
        # In-profile rounds soften the load-path transitions without a separate pad.
        arm = (rib_profile().extrude(RIB_WIDTH).edges('|X').fillet(2).val()
               .translate((sign*24-RIB_WIDTH/2, 0, 0)))
        arms.append(arm)
    return bd.Compound(cq.Compound.makeCompound(arms).wrapped)


def joint_cutters(include_head=False, include_nut=False):
    import cadquery as cq
    tools = []
    for x in JOINT_X:
        for z in JOINT_Z:
            tools.append(cq.Solid.makeCylinder(SCREW_CLEARANCE_D/2, 20,
                         cq.Vector(x,50,z), cq.Vector(0,1,0)))
            if include_head:
                tools.append(cq.Solid.makeCylinder(HEAD_CLEARANCE_D/2, 8,
                             cq.Vector(x,HEAD_SEAT_Y,z),cq.Vector(0,1,0)))
            if include_nut:
                # Side loading slot. Its height captures the nut flats; the rear
                # wall takes clamp load. Clearance is outside the clamping faces.
                x0,x1 = ((20.6,34.0) if x>0 else (-34.0,-20.6))
                tools.append(cq.Solid.makeBox(x1-x0, NUT_SLOT_Y[1]-NUT_SLOT_Y[0],
                             NUT_SLOT_HEIGHT,cq.Vector(x0,NUT_SLOT_Y[0],z-NUT_SLOT_HEIGHT/2)))
    return cq.Compound.makeCompound(tools)


@lru_cache(maxsize=1)
def carrier():
    """Separate dock: flat mounting face, full plate and exposed battery rails."""
    import cadquery as cq
    from parts.parkside_pap_204_a1 import make as make_battery
    plate = plate_profile().extrude(PLATE_THICKNESS).val()
    # Turn the lower end back toward the battery instead of leaving a flat
    # tongue below it. The 45-degree root grows from the flat print face.
    plate=plate.cut(cq.Solid.makeBox(100,70,24,cq.Vector(-50,40,60)))
    tail=(cq.Workplane('YZ').polyline([(60,88),(67,88),(67,84),(91,65.315),
            (78.685,65.315),(60,84)]).close().extrude(56).val().translate((-28,0,0)))
    tail=tail.intersect(plate_profile().extrude(50,both=True).val())
    mount=plate.fuse(tail).clean()
    roots=[e for e in mount.Edges() if abs(e.Center().z-84)<.01 and abs(e.Center().y-60)<.01 and e.Length()>35]
    if roots: mount=mount.fillet(3.0,roots)
    for sign in (-1,1):
        cheek = (cq.Workplane('XY').box(RAIL_CHEEK,11.6,50)
                 .translate((sign*25,71.8,106)).edges('|Z').fillet(1).val())
        lip = cq.Workplane('XY').box(7.6,RAIL_LIP_NOMINAL,50).translate((sign*23.7,78.5,106)).val()
        mount = mount.fuse(cheek).fuse(lip)
    result = bd.Compound(mount.cut(release_window()).wrapped)
    translation,(axis,angle) = REFERENCE_BATTERY_AT
    battery = make_battery().moved(bd.Location(translation,axis,angle))
    for solid in battery.solids():
        if result.distance_to(solid)>BATTERY_ALLOWANCE: continue
        for delta in offsets(BATTERY_ALLOWANCE): result -= solid.moved(bd.Location(delta))
        # Extra continuous cutter band at the turned lower tip. Use CQ solids
        # throughout this common/cut so the result retains 3D tool volume.
        if solid.bounding_box().min.Z < 86:
            tail_band=cq.Solid.makeBox(120,100,30,cq.Vector(-60,40,56))
            for delta in offsets(1.1):
                tool=cq.Shape.cast(solid.wrapped).translate(delta).intersect(tail_band)
                if tool.Solids():
                    result=bd.Compound(cq.Shape.cast(result.wrapped).cut(tool).wrapped)
    from parts.parkside_pap_204_a1 import rail_half_width, rail_neck_half, rail_neck_top, rail_top
    rear = REFERENCE_BATTERY_AT[0][1]
    flange_front = rear-rail_top-BATTERY_ALLOWANCE
    flange_back = rear-rail_neck_top+BATTERY_ALLOWANCE
    for half_width,front,back in (
            (rail_half_width+BATTERY_ALLOWANCE,flange_front,flange_back),
            (rail_neck_half+BATTERY_ALLOWANCE,flange_back,83.0)):
        result -= bd.Compound(cq.Solid.makeBox(2*half_width,back-front,58,
                              cq.Vector(-half_width,front,80)).wrapped)
    result -= bd.Compound(joint_cutters(include_head=True).wrapped)
    assert result.is_valid and len(result.solids())==1
    result=bd.Compound(incline(cq.Shape.cast(result.wrapped)).wrapped)
    result.label='backpack_dock_native'
    result.color=bd.Color('#99deef')
    return result


@lru_cache(maxsize=1)
def dock():
    """Combined occupied support geometry, for clearance checks only."""
    return frame_supports()+carrier()


def integrate_trunk(base):
    """Trunk and short gussets only; the battery dock is a separate screwed part."""
    import cadquery as cq
    from parts.power_switch_kcd1_native import make as make_switch
    land = (cq.Workplane('XY').box(66,7.95,18).translate((0,31.725,155.45))
            .edges('|Z').fillet(1.5).val())
    result = world(base)+bd.Compound(land.wrapped)+frame_supports()
    for x in (-25,-18,18,25):
        result += bd.Compound(cq.Solid.makeCylinder(2.61,9,cq.Vector(x,31.7,155.45)).wrapped)
    switch = world(make_switch())
    for delta in offsets(SWITCH_ALLOWANCE): result -= switch.moved(bd.Location(delta))
    result -= bd.Compound(incline(joint_cutters(include_nut=True)).wrapped)
    result -= bd.Compound(thumb_approach().wrapped)
    return native(result,'trunk_base_native')


@lru_cache(maxsize=1)
def shell_clearance():
    """Open shell termination around the fixed dock, with an explicit 0.8 mm gap."""
    import cadquery as cq
    gap=SHELL_ALLOWANCE
    # End the rear skin just in front of the exposed pack, rather than at the
    # front face of the backing. This lets the original rounded skin cover it.
    result=incline(cq.Solid.makeBox(180,150,230,cq.Vector(-90,70.5,20)))
    # Conservative, continuous offset of the backing and rail stock.
    plate=(plate_profile().offset2D(gap).extrude(PLATE_THICKNESS+2*gap).val()
           .translate((0,gap,0)))
    result=result.fuse(incline(plate))
    for sign in (-1,1):
        rail=cq.Solid.makeBox(9.2,17.1,57.6,cq.Vector(sign*25-4.6,65.4,80.2))
        result=result.fuse(incline(rail))
        rib=(rib_profile().offset2D(gap).extrude(RIB_WIDTH+2*gap).val()
             .translate((sign*24-RIB_WIDTH/2-gap,0,0)))
        result=result.fuse(rib)
    return bd.Compound(result.wrapped)


def relieve_shell(shell, label):
    """Retain the rounded skin around an open mount bay with explicit assembly gaps."""
    import cadquery as cq
    result=world(shell)-shell_clearance()-bd.Compound(thumb_approach().wrapped)
    # Widen only the hidden rear end of the service opening around the pack's
    # latch housing. The outer original skin profile remains unchanged.
    upper_relief=(cq.Workplane('XY',origin=(0,0,164))
        .polyline([(-24.5,34),(24.5,34),(30,44),(30,100),(-30,100),(-30,44)])
        .close().extrude(90).edges('|Z').fillet(2).val())
    result-=bd.Compound(upper_relief.wrapped)
    # Open the lower mounting bay fully: no old locating tongue or thin corner
    # pad can remain between the closer dock and the 3 mm shell wall.
    # The rounded opening stops inboard of the side skin and below its screws.
    bay=(cq.Workplane('XY').box(68,100,50).translate((0,94,115))
         .edges('|Z').fillet(4).val())
    result-=bd.Compound(bay.wrapped)
    split=cq.Solid.makeBox(2*SHELL_SPLIT_HALF_GAP,200,200,
                           cq.Vector(-SHELL_SPLIT_HALF_GAP,-50,50))
    result-=bd.Compound(split.wrapped)
    # Retain the connected shell; reject loose remnants rather than hiding them.
    # STEP cannot reliably round-trip the offset-curve extrusion left by
    # the dock clearance. Convert that surface representation before export;
    # this preserves the skin while retaining every closing face on import.
    from OCP.ShapeCustom import ShapeCustom
    result=bd.Compound(ShapeCustom.ConvertToBSpline_s(result.wrapped,True,False,True,False))
    return native(result,label)
