"""Exposed PAP 204 A1 backpack with a screwed two-part mount; all helper geometry in robot world mm.

Rear is +Y and up is +Z. Native trunk/shell factories use a frame rotated
+90 degrees about Z; adapters below return their original local frame.
The pack is a visual model: rail/latch fit and printed strength are unverified.
"""
from functools import lru_cache
from cadgen import build123d as bd

BATTERY_AT = ((0, 134.4, 126.815), ((1, -1, 1), 120))
BATTERY_BOTTOM_Z = 65.315
RIB_WIDTH = 18.0
ROOT_DEPTH = 10.0
PLATE_THICKNESS = 7.0
RAIL_CHEEK = 5.0
RAIL_LIP_NOMINAL = 4.0
THUMB_WIDTH = 36.0
SHELL_ALLOWANCE = 0.8
SHELL_SPLIT_HALF_GAP = 0.3
CORNER_SIZE = (16.0, 12.0, 18.0)
CORNER_RADIUS = 2.0
CORNER_TOP_Z = 136.0
CORNER_CENTRE = (29.0, 54.0, 136.5)
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
    return (cq.Workplane('XY').box(THUMB_WIDTH, 30, 54).translate((0, 57, 188))
            .edges('|Z').fillet(8).edges('|X or |Y').fillet(2).val())


# Four M3 x 12 socket-head screws, with M3 hex nuts slid in from the sides.
# Nominal hardware: 5.5 mm nut across flats / 2.4 mm thick; verify actual fit.
JOINT_X = (-22.0, 22.0)
JOINT_Z = (145.5, 157.5)
JOINT_FACE_Y = 60.0
SCREW_CLEARANCE_D = 3.4
HEAD_CLEARANCE_D = 6.4
HEAD_SEAT_Y = 63.8
NUT_SLOT_Y = (53.0, 55.8)
NUT_SLOT_HEIGHT = 5.8


def rib_profile():
    """Compact straight gusset with a continuous roof at the trunk top plane."""
    import cadquery as cq
    return (cq.Workplane('YZ').moveTo(27.75, 146.45).lineTo(34, 146.45)
            .lineTo(50, 139.45).lineTo(JOINT_FACE_Y, 139.45)
            .lineTo(JOINT_FACE_Y, 164.45).lineTo(27.75, 164.45).close())


@lru_cache(maxsize=1)
def frame_supports():
    import cadquery as cq
    arms = []
    for sign in (-1, 1):
        # In-profile rounds soften the load-path transitions without a separate pad.
        arm = (rib_profile().extrude(RIB_WIDTH).edges('|X').fillet(2).val()
               .translate((sign*22-RIB_WIDTH/2, 0, 0)))
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
                x0,x1 = ((18.6,32.0) if x>0 else (-32.0,-18.6))
                tools.append(cq.Solid.makeBox(x1-x0, NUT_SLOT_Y[1]-NUT_SLOT_Y[0],
                             NUT_SLOT_HEIGHT,cq.Vector(x0,NUT_SLOT_Y[0],z-NUT_SLOT_HEIGHT/2)))
    return cq.Compound.makeCompound(tools)


@lru_cache(maxsize=1)
def carrier():
    """Separate dock: flat mounting face, full plate and exposed battery rails."""
    import cadquery as cq
    from parts.parkside_pap_204_a1 import make as make_battery
    plate = (cq.Workplane('XZ', origin=(0,67,122.225)).rect(68,84.45)
             .extrude(PLATE_THICKNESS).edges('|Y').fillet(10).val())
    mount = plate
    for sign in (-1,1):
        cheek = (cq.Workplane('XY').box(RAIL_CHEEK,11.6,56)
                 .translate((sign*25,71.8,109)).edges('|Z').fillet(1).val())
        lip = cq.Workplane('XY').box(7.6,RAIL_LIP_NOMINAL,56).translate((sign*23.7,78.5,109)).val()
        mount = mount.fuse(cheek).fuse(lip)
    result = bd.Compound(mount.cut(thumb_approach()).wrapped)
    translation,(axis,angle) = BATTERY_AT
    battery = make_battery().moved(bd.Location(translation,axis,angle))
    for solid in battery.solids():
        if result.distance_to(solid)>BATTERY_ALLOWANCE: continue
        for delta in offsets(BATTERY_ALLOWANCE): result -= solid.moved(bd.Location(delta))
    from parts.parkside_pap_204_a1 import rail_half_width, rail_neck_half, rail_neck_top, rail_top
    rear = BATTERY_AT[0][1]
    flange_front = rear-rail_top-BATTERY_ALLOWANCE
    flange_back = rear-rail_neck_top+BATTERY_ALLOWANCE
    for half_width,front,back in (
            (rail_half_width+BATTERY_ALLOWANCE,flange_front,flange_back),
            (rail_neck_half+BATTERY_ALLOWANCE,flange_back,83.0)):
        result -= bd.Compound(cq.Solid.makeBox(2*half_width,back-front,58,
                              cq.Vector(-half_width,front,80)).wrapped)
    result -= bd.Compound(joint_cutters(include_head=True).wrapped)
    assert result.is_valid and len(result.solids())==1
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
    land = (cq.Workplane('XY').box(62,7.95,18).translate((0,31.725,155.45))
            .edges('|Z').fillet(1.5).val())
    result = world(base)+bd.Compound(land.wrapped)+frame_supports()
    for x in (-25,-18,18,25):
        result += bd.Compound(cq.Solid.makeCylinder(2.61,9,cq.Vector(x,31.7,155.45)).wrapped)
    switch = world(make_switch())
    for delta in offsets(SWITCH_ALLOWANCE): result -= switch.moved(bd.Location(delta))
    result -= bd.Compound(joint_cutters(include_nut=True).wrapped)
    result -= bd.Compound(thumb_approach().wrapped)
    return native(result,'trunk_base_native')


@lru_cache(maxsize=1)
def shell_clearance():
    """Continuous clearance tool; profile offset, not discrete translated copies.

    A padded plate envelope and offset rib profiles provide at least 0.8 mm
    around the dock. Omitting the rib's edge fillets is conservative.
    """
    import cadquery as cq
    gap = SHELL_ALLOWANCE
    result = (cq.Workplane('XY').box(68 + 2*gap, PLATE_THICKNESS + 2*gap, 84.45 + 2*gap)
              .val().translate((0, 63.5, 122.225)))
    for sign in (-1, 1):
        rib = (rib_profile().offset2D(gap).extrude(RIB_WIDTH + 2*gap).val()
               .translate((sign*22 - RIB_WIDTH/2 - gap, 0, 0)))
        result = result.fuse(rib)
    return bd.Compound(result.wrapped)


def relieve_shell(shell, label):
    """Reinforce both lower rear corners and leave explicit assembly gaps."""
    import cadquery as cq
    sign = 1 if label == 'left_shell_native' else -1
    x, y, z = CORNER_CENTRE
    corner = (cq.Workplane('XY').box(*CORNER_SIZE).edges().fillet(CORNER_RADIUS)
              .val().translate((sign*x, y, z)))
    # Stop before the clearance curve makes a thin feather at the upper end.
    corner = corner.intersect(cq.Solid.makeBox(120, 160, CORNER_TOP_Z - 100,
                                              cq.Vector(-60, -40, 100)))
    # Replace the old tongue rather than bury it under rounded stock: its
    # outermost half-millimetre otherwise survives below the new bottom round.
    old_tongue = cq.Solid.makeBox(10.5, 10, 8.51, cq.Vector(sign*26.25 - 5.25, 48, 127.49))
    result = (world(shell) - bd.Compound(old_tongue.wrapped)) + bd.Compound(corner.wrapped)
    result = result - shell_clearance() - bd.Compound(thumb_approach().wrapped)
    # Include the old rear patch: it previously left the two shells touching.
    split = cq.Solid.makeBox(2*SHELL_SPLIT_HALF_GAP, 200, 200,
                            cq.Vector(-SHELL_SPLIT_HALF_GAP, -50, 50))
    result -= bd.Compound(split.wrapped)
    # Remove the isolated donor rear patch below the release opening.
    rear_patch = cq.Solid.makeBox(25.6,14,13.42,cq.Vector(-12.8,48,147.59))
    result -= bd.Compound(rear_patch.wrapped)
    return native(result, label)
