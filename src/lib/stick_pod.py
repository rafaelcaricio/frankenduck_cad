"""PARKSIDE PAPK 12 A4 stick battery in a printed rump pod; robot world millimetres.

Rear is +Y and up is +Z. The stick stands cap down behind the trunk plate. Its
placement is a pure translation of the stick model frame
(src/parts/parkside_papk_12_a4.py): stick-local x, y and z are world X, Y - 58
and Z - 83. The pod is one printed part: a body-coloured fairing whose closed
top follows the back line, the stick socket with latch lips and a front tongue
screwed under the trunk plate. Its lower edge stays flat at the collar rim, so
nothing hangs below it when the stick is out.

APPROXIMATE: the stick outline, latch teeth and contacts come from the
photo-derived stick model. The socket clearance, latch lips, contact openings
and adapter spaces are nominal; check them against a real battery and a test
print before relying on the fit. No adapter or wiring is designed here.
"""
from functools import lru_cache
import math

from cadgen import build123d as bd

COLOR = '#99deef'
# Stick placement: identity rotation, translation to the cap-underside centre.
STICK_ORIGIN = (0.0, 58.0, 83.0)
STICK_AT = (STICK_ORIGIN, ('Z', 0.0))
# Removal: press both buttons and pull the stick straight down.
RELEASE_DIRECTION = (0.0, 0.0, -1.0)
RELEASE_TRAVEL = 95.0       # mm; the stick top ends 51 mm below the pod's lower edge
WALL = 2.5                  # pod and socket wall
CLEARANCE = 0.6             # stick outline to socket wall
SEAT_GAP = 0.3              # pod underside to the collar rim
FAIR_GROW = 0.8             # fairing starts this far outside the cap outline
SOCKET_TOP = 81.0           # socket wall top, 0.5 above the stick top
SHELL_GAP = 0.4             # body shells to pod
SHELL_SPLIT_HALF_GAP = 0.3  # each shell stops this far from the X = 0 centre plane
# Fairing sections inside and behind the body: (local z, half-width x,
# half-depth y, centre y) of superellipses with exponent SE_N. The loft ends in a
# 4 x 1.6 mm oval (TOP_SECTION) just under APEX, so the top is a low rounded
# dome; a single apex point upsets OCCT's point classifier.
SECTIONS = ((41.0, 30.0, 27.0, 1.0), (46.5, 30.0, 22.0, 7.0), (62.0, 31.0, 21.5, 7.5),
            (74.0, 30.5, 21.0, 8.0), (80.0, 28.5, 18.5, 9.0), (84.0, 25.0, 14.5, 9.5),
            (86.5, 18.0, 8.5, 10.0), (87.4, 10.0, 4.0, 10.0))
APEX = (0.0, 10.0, 87.8)
TOP_SECTION = (2.0, 0.8, 0.02)  # half-width, half-depth and drop below the apex
SE_N = 3.2
N_SEC = 96
# Latch: the stick's hooks catch on lips at the pod underside (APPROXIMATE).
LIP_GAP = 0.2               # lip top below the stick's catch
TOOTH_CLEAR = 0.4           # side and outer clearance round the hooks
TOOTH_FLEX = 1.5            # inward room for the hooks when the buttons are pressed
LIP_CLEAR = 0.2             # hook body (below the catch) to the lip's inner face
# Contact access (APPROXIMATE; stick-local). Slots open the + and - channels
# through the socket and the fairing's front wall into the body; depth is
# measured out from the stick side. The window opens the T contact at the back.
CONTACT_SLOT = dict(width=10.0, z0=60.0, depth=13.5)
T_WINDOW = dict(x=11.0, y0=14.0, y1=22.0, z0=58.0, z1=80.0)
# Hidden bay over the stick's front half under the body top skin, for a later
# terminal block; local y range and height above SOCKET_TOP.
TOP_BAY = dict(x=21.0, y0=-24.0, y1=-6.0, height=4.0)
# Wire passes from the pod's side spaces into the body, one each side.
WIRE_PASS = dict(x0=21.0, x1=31.0, y0=-18.0, y1=-6.0, z0=60.0, z1=70.0)
# Space (world boxes x0, x1, y0, y1, z0, z1) for a mated XT60 pair, nominally
# 8.2 x 15.5 x 23.5 mm with its axis vertical, on the +X side between the
# hip-yaw servo and the pod; lead room above it beside the trunk plate's rear
# edge and a channel under it toward the centre. No wiring is designed.
XT60_BAY = ((22.5, 31.1, 28.6, 44.6, 130.0, 154.0), (22.5, 31.1, 36.5, 44.6, 154.0, 163.0),
            (10.0, 31.1, 28.6, 34.0, 125.0, 130.0))
# Tongue under the trunk plate (world): two M3 x 10 socket-head screws from
# above through the plate into M3 x 5.7 heat-set inserts.
PLATE_BOTTOM_Z = 158.45
PLATE_TOP_Z = 164.45
TONGUE = dict(x=9.5, y0=21.0, y1=42.0, z0=150.45)
SCREWS = ((-5.0, 25.0), (5.0, 25.0))
SCREW_CLEARANCE_D = 3.4
INSERT_PILOT_D = 4.0
INSERT_DEPTH = 6.0
TIP_BORE_D = 3.0


def _stick():
    from parts import parkside_papk_12_a4 as stick
    return stick


def to_world(point):
    return tuple(p + o for p, o in zip(point, STICK_ORIGIN))


def local_to_world(shape):
    import cadquery as cq
    return shape.translate(cq.Vector(*STICK_ORIGIN))


def world(shape):
    return bd.Compound(shape.wrapped).moved(bd.Location((0, 0, 0), (0, 0, 1), -90))


def native(shape, label):
    assert shape.is_valid and len(shape.solids()) == 1, label
    result = shape.moved(bd.Location((0, 0, 0), (0, 0, 1), 90))
    result.label = label
    result.color = bd.Color(COLOR)
    return result


def cap_outline(grow, n=N_SEC):
    """Cap plan outline offset by grow, as (x, y) points; t runs from -y toward +x."""
    s = _stick()
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        r = sum(c * math.cos(k * t) for k, c in enumerate(s.cap_radius_terms))
        dr = -sum(k * c * math.sin(k * t) for k, c in enumerate(s.cap_radius_terms))
        dx, dy = dr * math.sin(t) + r * math.cos(t), -dr * math.cos(t) + r * math.sin(t)
        length = math.hypot(dx, dy)
        pts.append((r * math.sin(t) + grow * dy / length, s.cap_centre_y - r * math.cos(t) - grow * dx / length))
    return pts


def _spline_wire(pts, z):
    import cadquery as cq
    return cq.Wire.assembleEdges([cq.Edge.makeSpline([cq.Vector(x, y, z) for x, y in pts], periodic=True)])


def stick_wire(grow, z=0.0):
    """Rounded-triangle stick outline offset by grow, built as in the stick model."""
    import cadquery as cq
    s = _stick()
    lobes = [cq.Vector(x, y, 0) for x, y in s.stick_lobes]
    order = [0, 2, 1]
    radius = s.stick_radius + grow
    sides = []
    for k in range(3):
        i, j = order[k], order[(k + 1) % 3]
        d = (lobes[j] - lobes[i]).normalized()
        n = cq.Vector(d.y, -d.x, 0)
        sides.append((lobes[i] + n * radius, lobes[j] + n * radius, n))
    w = cq.Workplane('XY', origin=(0, 0, z)).moveTo(sides[0][0].x, sides[0][0].y)
    for k in range(3):
        _, b, n = sides[k]
        nxt = sides[(k + 1) % 3]
        w = w.lineTo(b.x, b.y)
        mid = (n + nxt[2]).normalized() * radius + lobes[order[(k + 1) % 3]]
        w = w.threePointArc((mid.x, mid.y), (nxt[0].x, nxt[0].y))
    return w.close().wire().val()


def prism(kind, grow, z0, z1):
    """Vertical prism of the stick or cap outline between local z0 and z1."""
    import cadquery as cq
    wire = stick_wire(grow, z0) if kind == 'stick' else _spline_wire(cap_outline(grow), z0)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), cq.Vector(0, 0, z1 - z0))


def _superellipse(a, b, cy):
    def power(u):
        return math.copysign(abs(u) ** (2.0 / SE_N), u)
    return [(a * power(math.sin(t)), cy - b * power(math.cos(t)))
            for t in (2 * math.pi * i / N_SEC for i in range(N_SEC))]


def _offset_profile(points, offset):
    """Miter offset of an open (r, z) profile; offset > 0 moves toward the inside (smaller r, lower top)."""
    out = []
    for i, (r, z) in enumerate(points):
        normals = []
        for a, b in ((i - 1, i), (i, i + 1)):
            if 0 <= a and b < len(points):
                dr, dz = points[b][0] - points[a][0], points[b][1] - points[a][1]
                length = math.hypot(dr, dz)
                normals.append((-dz / length, dr / length))
        if len(normals) == 1:
            nr, nz = normals[0]
        else:
            (r1, z1), (r2, z2) = normals
            scale = 1.0 / (1.0 + r1 * r2 + z1 * z2)
            nr, nz = (r1 + r2) * scale, (z1 + z2) * scale
        out.append((r + offset * nr, z + offset * nz))
    return out


def fairing_sections(offset=0.0):
    """SECTIONS and APEX offset normal to the side and depth profiles; offset > 0 is inward."""
    s = _stick()
    z0 = s.rim_height + SEAT_GAP
    cap = cap_outline(FAIR_GROW)
    half_width = max(abs(x) for x, _ in cap)
    half_depth = (max(y for _, y in cap) - min(y for _, y in cap)) / 2
    side = _offset_profile([(half_width, z0)] + [(a, z) for z, a, _, _ in SECTIONS] + [(0.0, APEX[2])], offset)
    depth = _offset_profile([(half_depth, z0)] + [(b, z) for z, _, b, _ in SECTIONS] + [(0.0, APEX[2])], offset)
    sections = [((za + zb) / 2, a, b, cy) for (a, za), (b, zb), (_, _, _, cy) in zip(side[1:-1], depth[1:-1], SECTIONS)]
    return sections, (APEX[0], APEX[1], (side[-1][1] + depth[-1][1]) / 2)


def fairing(offset=0.0):
    """Closed loft from the cap outline through the sections to the small top oval.

    offset > 0 gives the inner surface of a wall that thick; offset < 0 gives a
    clearance envelope. Offsetting the finished loft is slow and unreliable, so the
    sections are offset instead; checks measure the resulting walls and gaps.
    """
    import cadquery as cq
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    s = _stick()
    z0 = s.rim_height + SEAT_GAP + offset
    sections, apex = fairing_sections(offset)
    loft = BRepOffsetAPI_ThruSections(True, False, 1e-6)
    loft.AddWire(_spline_wire(cap_outline(FAIR_GROW - offset), z0).wrapped)
    for z, a, b, cy in sections:
        loft.AddWire(_spline_wire(_superellipse(a, b, cy), z).wrapped)
    a, b, drop = TOP_SECTION
    loft.AddWire(_spline_wire(_superellipse(a, b, apex[1]), apex[2] - drop).wrapped)
    loft.CheckCompatibility(False)
    loft.Build()
    solid = cq.Shape.cast(loft.Shape())
    # The approximated loft can sag a little below its first section; trim it there.
    keep = cq.Solid.makeBox(200, 200, 200, cq.Vector(-100, -100, z0))
    solid = solid.intersect(keep).Solids()[0]
    # Split the closed loft face into four patches: OCCT's point classifier
    # misjudges about 1 % of outside points against one closed face, which
    # breaks distance checks of the saved part.
    from OCP.ShapeUpgrade import ShapeUpgrade_ShapeDivideClosed
    divide = ShapeUpgrade_ShapeDivideClosed(solid.wrapped)
    divide.SetNbSplitPoints(3)
    divide.Perform()
    return cq.Shape.cast(divide.Result()).Solids()[0]


def _tooth_band(inner, outer, z0, z1):
    """Plan band between cap-outline offsets inner and outer (negative = inward), over the hook length in y."""
    import cadquery as cq
    s = _stick()
    band = prism('cap', outer, z0, z1).cut(prism('cap', inner, z0 - 1, z1 + 1))
    y0, y1 = s.tooth_y[0] - TOOTH_CLEAR, s.tooth_y[1] + TOOTH_CLEAR
    cutters = []
    for side in (-1, 1):
        x0, x1 = sorted((side * 5.0, side * 40.0))
        cutters.append(band.intersect(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))))
    return cutters[0].fuse(cutters[1])


def latch_pockets():
    """Hook pockets above the lips, and room for the hook body and its inward flex below them.

    Offsets from the cap outline follow the stick model: collar 0.7 inside, hook
    face 1.0 inside above the catch and 2.0 inside below it, hook back 4.0 inside.
    """
    s = _stick()
    back = -(s.collar_inset + s.tooth_depth[1]) - TOOTH_FLEX
    face = -(s.collar_inset + s.tooth_depth[0])
    notch_face = face - s.tooth_notch
    lip_top = s.tooth_catch - LIP_GAP
    hook = _tooth_band(back, face + TOOTH_CLEAR, lip_top, s.tooth_top + 2 * TOOTH_CLEAR)
    body = _tooth_band(back, notch_face + LIP_CLEAR, s.rim_height - 1.0, lip_top + 0.01)
    return hook.fuse(body)


def socket_cavity():
    """Stick clearance with latch pockets, contact slots and the T window (stick-local)."""
    import cadquery as cq
    s = _stick()
    # The cavity stops at the socket top so the fairing's dome stays closed over the stick.
    cavity = prism('stick', CLEARANCE, s.cup_floor, SOCKET_TOP).fuse(latch_pockets())
    t = T_WINDOW
    cavity = cavity.fuse(cq.Solid.makeBox(2 * t['x'], t['y1'] - t['y0'], t['z1'] - t['z0'],
                                          cq.Vector(-t['x'], t['y0'], t['z0'])))
    for name in ('plus', 'minus'):
        (cx, cy), (dx, dy), (nx, ny) = contact_frame(name)
        w = CONTACT_SLOT['width'] / 2
        depth = CONTACT_SLOT['depth']
        pts = [(cx - dx * w - nx, cy - dy * w - ny), (cx + dx * w - nx, cy + dy * w - ny),
               (cx + dx * w + nx * depth, cy + dy * w + ny * depth), (cx - dx * w + nx * depth, cy - dy * w + ny * depth)]
        z0, z1 = CONTACT_SLOT['z0'], SOCKET_TOP + 1.0
        face = cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(x, y, z0) for x, y in pts], close=True))
        cavity = cavity.fuse(cq.Solid.extrudeLinear(face, cq.Vector(0, 0, z1 - z0)))
    return cavity


def contact_frame(name):
    """Stick-local (centre, tangent, outward normal) of a contact channel on the stick side."""
    s = _stick()
    lobes = s.stick_lobes
    if name == 'T':
        return (s.t_channel_x, lobes[0][1] + s.stick_radius), (1.0, 0.0), (0.0, 1.0)
    i, j = (0, 2) if name == 'plus' else (2, 1)
    dx, dy = lobes[j][0] - lobes[i][0], lobes[j][1] - lobes[i][1]
    length = math.hypot(dx, dy)
    dx, dy = dx / length, dy / length
    nx, ny = dy, -dx
    sx, sy = lobes[2][0] + nx * s.stick_radius, lobes[2][1] + ny * s.stick_radius
    sign = 1 if name == 'minus' else -1
    distance = sign * s.channel_s[name]
    return (sx + dx * distance, sy + dy * distance), (dx, dy), (nx, ny)


def top_bay():
    """Space over the stick's front half for a later terminal block (stick-local)."""
    import cadquery as cq
    b = TOP_BAY
    return cq.Solid.makeBox(2 * b['x'], b['y1'] - b['y0'], b['height'], cq.Vector(-b['x'], b['y0'], SOCKET_TOP))


def wire_passes():
    import cadquery as cq
    w = WIRE_PASS
    boxes = []
    for side in (-1, 1):
        x0, x1 = sorted((side * w['x0'], side * w['x1']))
        boxes.append(cq.Solid.makeBox(x1 - x0, w['y1'] - w['y0'], w['z1'] - w['z0'], cq.Vector(x0, w['y0'], w['z0'])))
    return boxes[0].fuse(boxes[1])


def tongue():
    """Front tongue clamped under the trunk plate (world)."""
    import cadquery as cq
    t = TONGUE
    body = (cq.Workplane('XY', origin=(0, 0, t['z0'])).center(0, (t['y0'] + t['y1']) / 2)
            .rect(2 * t['x'], t['y1'] - t['y0']).extrude(PLATE_BOTTOM_Z - t['z0']).edges('|Z').fillet(2.0).val())
    return body


def insert_pilots():
    import cadquery as cq
    cuts = []
    for x, y in SCREWS:
        cuts.append(cq.Solid.makeCylinder(INSERT_PILOT_D / 2, INSERT_DEPTH + 0.5,
                                          cq.Vector(x, y, PLATE_BOTTOM_Z - INSERT_DEPTH), cq.Vector(0, 0, 1)))
        cuts.append(cq.Solid.makeCylinder(TIP_BORE_D / 2, 3.0, cq.Vector(x, y, PLATE_BOTTOM_Z - INSERT_DEPTH - 3.0 + 0.01),
                                          cq.Vector(0, 0, 1)))
    out = cuts[0]
    for c in cuts[1:]:
        out = out.fuse(c)
    return out


def plate_holes():
    """Screw clearance through the trunk plate (world)."""
    import cadquery as cq
    holes = [cq.Solid.makeCylinder(SCREW_CLEARANCE_D / 2, 20.0, cq.Vector(x, y, 150.0), cq.Vector(0, 0, 1)) for x, y in SCREWS]
    return holes[0].fuse(holes[1])


@lru_cache(maxsize=1)
def pod_local():
    """Hollow fairing and socket tube, cut for the stick (stick-local)."""
    s = _stick()
    outer = fairing()
    inner = fairing(WALL)
    tube = prism('stick', CLEARANCE + WALL, s.rim_height + SEAT_GAP, SOCKET_TOP)
    body = outer.cut(inner).fuse(tube)
    return body.cut(socket_cavity()).cut(top_bay()).cut(wire_passes()).clean()


@lru_cache(maxsize=1)
def pod():
    """The printed pod in world coordinates: fairing, socket and tongue as one solid."""
    import cadquery as cq
    body = local_to_world(pod_local()).fuse(tongue())
    # The tongue must not close the socket or the latch pockets.
    body = body.cut(local_to_world(socket_cavity())).cut(insert_pilots()).clean()
    solids = body.Solids()
    assert len(solids) == 1 and solids[0].isValid(), 'stick pod must be one valid solid'
    return cq.Shape.cast(solids[0].wrapped)


@lru_cache(maxsize=1)
def envelope(grow=SHELL_GAP):
    """Outer pod envelope grown by grow, for cutting the body shells (world)."""
    s = _stick()
    tube = prism('stick', CLEARANCE + WALL + grow, s.rim_height, SOCKET_TOP + grow)
    return local_to_world(fairing(-grow).fuse(tube))


def integrate_trunk(base):
    """Fill the four obsolete carrier bores and add the two pod screw clearances."""
    import cadquery as cq
    result = world(base)
    for x in (-25.0, -18.0, 18.0, 25.0):
        result += bd.Compound(cq.Solid.makeCylinder(2.61, PLATE_TOP_Z - 155.45, cq.Vector(x, 31.7, 155.45)).wrapped)
    result -= bd.Compound(plate_holes().wrapped)
    return native(result, 'trunk_base_native')


def relieve_shell(shell, label):
    """Cut a body shell round the pod with SHELL_GAP and open the centre split; the rest of the skin is unchanged."""
    import cadquery as cq
    split = cq.Solid.makeBox(2 * SHELL_SPLIT_HALF_GAP, 200, 200, cq.Vector(-SHELL_SPLIT_HALF_GAP, -50, 50))
    result = world(shell) - bd.Compound(envelope().wrapped) - bd.Compound(split.wrapped)
    # Keep the cut skin as B-splines so the saved shell stays one closed solid.
    from OCP.ShapeCustom import ShapeCustom
    result = bd.Compound(ShapeCustom.ConvertToBSpline_s(result.wrapped, True, False, True, False))
    return native(result, label)


def adapter_allowances():
    """Review volumes (world) kept free for a later contact adapter; never printed.

    plus/minus: strips in the + and - channels from the plate face out through the
    contact slots; t: the window behind the T contact; top: the hidden bay over the
    stick's front half; xt60: XT60_BAY. Positions follow the photo-derived stick
    and nominal connector sizes (APPROXIMATE).
    """
    import cadquery as cq
    s = _stick()
    out = {}
    for name in ('plus', 'minus'):
        (cx, cy), (dx, dy), (nx, ny) = contact_frame(name)
        w = s.plate_width / 2
        # Start 0.1 off the plate face so the volume does not share its face.
        n0, n1 = -s.plate_face_depth + 0.1, CLEARANCE + WALL + 2.0
        pts = [(cx - dx * w + nx * n0, cy - dy * w + ny * n0), (cx + dx * w + nx * n0, cy + dy * w + ny * n0),
               (cx + dx * w + nx * n1, cy + dy * w + ny * n1), (cx - dx * w + nx * n1, cy - dy * w + ny * n1)]
        z0, z1 = s.plate_z[0], s.stick_top
        face = cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(x, y, z0) for x, y in pts], close=True))
        out[name] = cq.Solid.extrudeLinear(face, cq.Vector(0, 0, z1 - z0))
    t = T_WINDOW
    back = s.stick_lobes[0][1] + s.stick_radius
    out['t'] = cq.Solid.makeBox(2 * t['x'] - 2, back + CLEARANCE + WALL + 2.0 - back, t['z1'] - t['z0'] - 2,
                                cq.Vector(-t['x'] + 1, back, t['z0'] + 1))
    b = TOP_BAY
    # The bay's front 1.5 mm stays off the trunk plate's rear edge.
    out['top'] = cq.Solid.makeBox(2 * b['x'] - 2, b['y1'] - b['y0'] - 2.5, b['height'] - 0.5,
                                  cq.Vector(-b['x'] + 1, b['y0'] + 1.5, s.stick_top + 0.5))
    out = {k: local_to_world(v) for k, v in out.items()}
    boxes = [cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0)) for x0, x1, y0, y1, z0, z1 in XT60_BAY]
    out['xt60'] = boxes[0].fuse(*boxes[1:]).clean()
    return out
