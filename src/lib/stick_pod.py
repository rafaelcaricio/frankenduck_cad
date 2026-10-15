"""PARKSIDE PAPK 12 A4 stick battery in a printed rump pod; robot world millimetres.

Rear is +Y and up is +Z. The stick stands cap down behind the trunk plate. Its
placement is a pure translation of the stick model frame
(src/parts/parkside_papk_12_a4.py): stick-local x, y and z are world X, Y - 58
and Z - 83. The pod is one printed part: a body-coloured fairing whose closed
top follows the back line and whose front opens into the body, the stick
socket with latch lips and a front tongue screwed under the trunk plate. Its
lower edge stays flat at the collar rim, so nothing hangs below it when the
stick is out.

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
EDGE_ROUND = 0.6            # round on outside edges: the seat edge, the front opening's outer edge, the tongue
CLEARANCE = 0.6             # stick outline to socket wall
SEAT_GAP = 0.3              # pod underside to the collar rim
FAIR_GROW = 0.8             # fairing starts this far outside the cap outline
SOCKET_TOP = 78.0           # socket wall top, 2.5 below the stick top so it stays 2.4 under the dome
SHELL_GAP = 0.4             # body shells to pod
SHELL_SPLIT_HALF_GAP = 0.3  # each shell stops this far from the X = 0 centre plane
# Fairing (stick-local). At the seat it follows the cap outline grown by
# FAIR_GROW; up to SKIRT_TOP it morphs into the box section (a superellipse of
# exponent SE_N) with one smootherstep, so its side, front and back move
# together; it rises straight to DOME_BASE and closes in a superellipse dome
# (exponent DOME_N) at APEX. One cubic B-spline surface interpolates the rows:
# periodic round the plan, clamped upward, with the apex as a pole. Its
# smallest radius is about 3.0 mm, so the 2.5 mm wall is a true normal offset.
SE_N = 3.2
BOX = dict(half_width=30.5, front=-13.5, back=29.0)
SKIRT_TOP = 50.0
DOME_BASE = 74.0
DOME_N = 2.6
APEX = (0.0, 10.0, 87.8)
N_SEC = 96                  # points round each row
ROW_STEP = 1.0              # row spacing along the profile
ROWS_BELOW = 3              # straight rows under the seat, so offsets reach below it
# Latch: the stick's hooks catch on lips at the pod underside (APPROXIMATE).
LIP_GAP = 0.2               # lip top below the stick's catch
TOOTH_CLEAR = 0.4           # side and outer clearance round the hooks
TOOTH_FLEX = 1.5            # inward room for the hooks when the buttons are pressed
LIP_CLEAR = 0.2             # hook body (below the catch) to the lip's inner face
# Front opening (stick-local): the fairing is open into the body in front of
# the plane y = FRONT_OPEN['y'] (taken on its outer surface) and above
# z = FRONT_OPEN['z0'], with corners rounded to FRONT_OPEN['corner'] along the
# wall. The body shells hide it. Its cut faces follow the wall normal, so every
# wall end is square. It gives the contact slots, the top bay and the wires a
# way into the body.
FRONT_OPEN = dict(y=-6.0, z0=58.0, corner=4.0)
# Contact access (APPROXIMATE; stick-local). Slots open the + and - channels
# through the socket wall from z0 up through its top; the window opens the T
# contact at the back, with sides on the socket's normals (x = +-x at its outer face).
CONTACT_SLOT = dict(width=10.0, z0=60.0)
T_WINDOW = dict(x=11.0, z0=58.0, z1=80.0)
# Bay over the stick's front half, under the body top skin, kept free for a
# later terminal block: local half-width, y range and height above the stick top.
TOP_BAY = dict(x=21.0, y0=-24.0, y1=-6.0, height=4.0)
# Space (world boxes x0, x1, y0, y1, z0, z1) for a mated XT60 pair, nominally
# 8.2 x 15.5 x 23.5 mm with its axis vertical, on the +X side between the
# hip-yaw servo and the pod; lead room above it beside the trunk plate's rear
# edge and a channel under it toward the centre. No wiring is designed.
XT60_BAY = ((22.5, 31.1, 28.6, 44.6, 130.0, 154.0), (22.5, 31.1, 36.5, 44.6, 154.0, 163.0),
            (10.0, 31.1, 28.6, 34.0, 125.0, 130.0))
# Tongue under the trunk plate (world): two M3 x 12 socket-head screws from
# above through the plate into M3 x 5.7 heat-set inserts. SCREW and INSERT name
# the bd_warehouse parts that checks/part_quality.py places and checks.
PLATE_BOTTOM_Z = 158.45
PLATE_TOP_Z = 164.45
TONGUE = dict(x=9.5, y0=21.0, y1=42.0, z0=150.45)
SCREWS = ((-5.0, 25.0), (5.0, 25.0))
SCREW_CLEARANCE_D = 3.4
INSERT_PILOT_D = 4.0
INSERT_DEPTH = 6.0
TIP_BORE_D = 3.0
SCREW = dict(size='M3-0.5', length=12.0, fastener_type='iso4762')   # ISO 4762 socket head
INSERT = dict(size='M3-0.5-5.7', fastener_type='CNCKitchen')      # heat-set insert


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


def _smootherstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * t * (10 - 15 * t + 6 * t * t)


def _angles():
    return [2 * math.pi * i / N_SEC for i in range(N_SEC)]


@lru_cache(maxsize=1)
def _cap_row():
    """Cap outline grown by FAIR_GROW at N_SEC polar angles about its box centre; angle 0 points to -y."""
    import numpy as np
    cap = np.array(cap_outline(FAIR_GROW, 1440))
    cy = (cap[:, 1].min() + cap[:, 1].max()) / 2
    angle = np.arctan2(cap[:, 0], cy - cap[:, 1]) % (2 * math.pi)
    radius = np.hypot(cap[:, 0], cap[:, 1] - cy)
    order = np.argsort(angle)
    angle, radius = angle[order], radius[order]
    r = np.interp(_angles(), np.concatenate([angle - 2 * math.pi, angle, angle + 2 * math.pi]), np.tile(radius, 3))
    return tuple((ri * math.sin(t), cy - ri * math.cos(t)) for ri, t in zip(r, _angles()))


def _box_row(scale=1.0, cy=None):
    a, b = BOX['half_width'], (BOX['back'] - BOX['front']) / 2
    cy = (BOX['back'] + BOX['front']) / 2 if cy is None else cy
    out = []
    for t in _angles():
        r = (abs(math.sin(t) / a) ** SE_N + abs(math.cos(t) / b) ** SE_N) ** (-1 / SE_N)
        out.append((scale * r * math.sin(t), cy - scale * r * math.cos(t)))
    return out


@lru_cache(maxsize=1)
def fairing_rows():
    """Rows of N_SEC points (rows, N_SEC, 3) from below the seat to the apex, spaced about ROW_STEP along the profile."""
    import numpy as np
    s = _stick()
    z0 = s.rim_height + SEAT_GAP
    cap, box = np.array(_cap_row()), np.array(_box_row())

    def row(xy, z):
        return np.column_stack([xy, np.full(len(xy), z)])

    rows = [row(cap, z0 - ROW_STEP * k) for k in range(ROWS_BELOW, 0, -1)]
    # Skirt rows by arc length of the column that moves furthest.
    z = np.linspace(z0, SKIRT_TOP, 2000)
    move = np.linalg.norm(box - cap, axis=1).max() * np.array([_smootherstep((v - z0) / (SKIRT_TOP - z0)) for v in z])
    length = np.concatenate([[0], np.cumsum(np.hypot(np.diff(z), np.diff(move)))])
    for l in np.linspace(0, length[-1], max(2, int(length[-1] / ROW_STEP) + 1))[:-1]:
        zr = float(np.interp(l, length, z))
        w = _smootherstep((zr - z0) / (SKIRT_TOP - z0))
        rows.append(row((1 - w) * cap + w * box, zr))
    for zr in np.linspace(SKIRT_TOP, DOME_BASE, max(2, int((DOME_BASE - SKIRT_TOP) / (2 * ROW_STEP)) + 1))[:-1]:
        rows.append(row(box, zr))
    # Dome rows by arc length of its side meridian.
    t = np.linspace(0, math.pi / 2, 4000)
    c, h = np.cos(t) ** (2 / DOME_N), np.sin(t) ** (2 / DOME_N)
    height = APEX[2] - DOME_BASE
    length = np.concatenate([[0], np.cumsum(np.hypot(np.diff(height * h), np.diff(BOX['half_width'] * c)))])
    cy = (BOX['back'] + BOX['front']) / 2
    for l in np.linspace(0, length[-1], max(2, int(length[-1] / ROW_STEP) + 1))[:-1]:
        tr = float(np.interp(l, length, t))
        cr, hr = math.cos(tr) ** (2 / DOME_N), math.sin(tr) ** (2 / DOME_N)
        rows.append(row(_box_row(cr, cy + (APEX[1] - cy) * hr), DOME_BASE + height * hr))
    rows.append(np.tile(APEX, (N_SEC, 1)))
    return np.array(rows)


def _surface(rows):
    """Cubic B-spline surface through rows (nv, nu, 3): periodic and uniform in u = i / nu, clamped in v = j / (nv - 1).

    Built here because OCCT's periodic GeomAPI_PointsToBSplineSurface swings about
    0.3 % of the coordinate size between the points.
    """
    import numpy as np
    from scipy.interpolate import make_interp_spline
    from OCP.Geom import Geom_BSplineSurface
    from OCP.TColgp import TColgp_Array2OfPnt
    from OCP.TColStd import TColStd_Array1OfReal, TColStd_Array1OfInteger
    from OCP.gp import gp_Pnt
    nv, nu = rows.shape[:2]
    # Periodic uniform cubic: (P[i - 1] + 4 P[i] + P[i + 1]) / 6 passes through point i.
    scale = (4 + 2 * np.cos(2 * np.pi * np.arange(nu) / nu)) / 6
    u_poles = np.real(np.fft.ifft(np.fft.fft(rows, axis=1) / scale[None, :, None], axis=1))
    spline = make_interp_spline(np.linspace(0, 1, nv), u_poles, k=3, axis=0)
    poles = spline.c
    knots, mults = np.unique(np.round(spline.t, 12), return_counts=True)
    grid = TColgp_Array2OfPnt(1, nu, 1, poles.shape[0])
    for j in range(poles.shape[0]):
        for i in range(nu):
            # OCCT's first periodic pole belongs to the span before u = 0.
            grid.SetValue(i + 1, j + 1, gp_Pnt(*poles[j, (i - 1) % nu]))
    uk, um = TColStd_Array1OfReal(1, nu + 1), TColStd_Array1OfInteger(1, nu + 1)
    for k in range(nu + 1):
        uk.SetValue(k + 1, k / nu)
        um.SetValue(k + 1, 1)
    vk, vm = TColStd_Array1OfReal(1, len(knots)), TColStd_Array1OfInteger(1, len(knots))
    for k, (knot, mult) in enumerate(zip(knots, mults)):
        vk.SetValue(k + 1, float(knot))
        vm.SetValue(k + 1, int(mult))
    return Geom_BSplineSurface(grid, uk, vk, um, vm, 3, 3, True, False)


@lru_cache(maxsize=1)
def _outer_surface():
    return _surface(fairing_rows())


def _offset_rows(surface, rows, offset):
    """Rows moved inward by offset (outward when negative) along the surface normal; the apex moves down."""
    import numpy as np
    from OCP.GeomLProp import GeomLProp_SLProps
    nv, nu = rows.shape[:2]
    z0 = _stick().rim_height + SEAT_GAP
    out = rows.copy()
    for j in range(nv - 1):
        for i in range(nu):
            props = GeomLProp_SLProps(surface, i / nu, j / (nv - 1), 1, 1e-9)
            n = np.array(props.Normal().Coord())
            if rows[j, i, 2] < z0 - 1e-9:   # rows below the seat are vertical by construction
                n[2] = 0.0
                n /= np.linalg.norm(n)
            out[j, i] = rows[j, i] - offset * n
    out[-1] = rows[-1] - (0.0, 0.0, offset)
    return out


def _closed_solid(surface, z_bottom, z_cut):
    """Solid bounded by the surface and a flat bottom at z_bottom, trimmed to z >= z_cut."""
    import cadquery as cq
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing
    from OCP.BRepLib import BRepLib
    from OCP.ShapeFix import ShapeFix_Solid
    from OCP.ShapeUpgrade import ShapeUpgrade_ShapeDivideClosed
    from OCP.TopoDS import TopoDS
    face = BRepBuilderAPI_MakeFace(surface, 1e-6).Face()
    rim = [e for e in cq.Face(face).Edges() if abs(e.Center().z - z_bottom) < 1e-3 and e.Length() > 1.0]
    assert len(rim) == 1, 'fairing surface needs one flat bottom edge'
    sew = BRepBuilderAPI_Sewing(1e-5)
    sew.Add(face)
    sew.Add(cq.Face.makeFromWires(cq.Wire.assembleEdges(rim)).wrapped)
    sew.Perform()
    fix = ShapeFix_Solid(BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(sew.SewedShape())).Solid())
    fix.Perform()
    solid = cq.Shape.cast(fix.Solid())
    solid = solid.intersect(cq.Solid.makeBox(200, 200, 200, cq.Vector(-100, -100, z_cut))).Solids()[0]
    # Split the closed face into four patches: OCCT's point classifier misjudges
    # some outside points against one closed face, which breaks distance checks.
    divide = ShapeUpgrade_ShapeDivideClosed(solid.wrapped)
    divide.SetNbSplitPoints(3)
    divide.Perform()
    result = divide.Result()
    BRepLib.SameParameter_s(result, 1e-5, True)
    return cq.Shape.cast(result).Solids()[0]


@lru_cache(maxsize=4)
def fairing(offset=0.0):
    """The fairing solid from the seat plane up; offset > 0 gives the inner surface of a wall that thick
    (from the seat plane plus offset up), offset < 0 a clearance envelope (from the seat plane minus the gap up).
    """
    rows = fairing_rows()
    surface = _outer_surface()
    if offset:
        rows = _offset_rows(surface, rows, offset)
        surface = _surface(rows)
    z0 = _stick().rim_height + SEAT_GAP
    return _closed_solid(surface, float(rows[0, 0, 2]), z0 + offset)


def _cap_frame(t):
    """Cap outline point at parameter t (as in cap_outline) and its unit outward normal."""
    s = _stick()
    r = sum(c * math.cos(k * t) for k, c in enumerate(s.cap_radius_terms))
    dr = -sum(k * c * math.sin(k * t) for k, c in enumerate(s.cap_radius_terms))
    dx, dy = dr * math.sin(t) + r * math.cos(t), -dr * math.cos(t) + r * math.sin(t)
    length = math.hypot(dx, dy)
    return (r * math.sin(t), s.cap_centre_y - r * math.cos(t)), (dy / length, -dx / length)


def _bisect(f, lo, hi):
    flo = f(lo)
    for _ in range(80):
        mid = (lo + hi) / 2
        if (f(mid) > 0) == (flo > 0):
            lo, flo = mid, f(mid)
        else:
            hi = mid
    return (lo + hi) / 2


def _side_params(side):
    return (0.02, math.pi - 0.02) if side > 0 else (math.pi + 0.02, 2 * math.pi - 0.02)


def _cap_param_at_y(grow, side, y):
    """Cap parameter on one side (x sign) where the outline offset by grow crosses y."""
    def f(t):
        (px, py), (nx, ny) = _cap_frame(t)
        return py + grow * ny - y
    return _bisect(f, *_side_params(side))


def _cap_param_through(point, side):
    """Cap parameter on one side whose normal line passes through point."""
    def f(t):
        (px, py), (nx, ny) = _cap_frame(t)
        return (point[0] - px) * ny - (point[1] - py) * nx
    return _bisect(f, *_side_params(side))


@lru_cache(maxsize=2)
def _pocket_plan(side):
    """Plan outline of one latch pocket, as (x, y) points.

    It holds the hook band (cap-outline offsets from the hook back less its flex
    to the hook face plus clearance, over the hook length in y) and opens into
    the stick clearance. Near the socket its ends are square to the socket's flat
    side; beyond the socket wall they lie on the cap outline's normals, so the
    lip and the fairing wall end square too.
    """
    import numpy as np
    from shapely.geometry import MultiPoint, Polygon
    s = _stick()
    (sx, sy), (dx, dy), (nx, ny) = contact_frame('minus' if side > 0 else 'plus')

    def flat(p):
        return ((p[0] - sx) * dx + (p[1] - sy) * dy, (p[0] - sx) * nx + (p[1] - sy) * ny)

    def plan(a, b):
        return (sx + a * dx + b * nx, sy + a * dy + b * ny)

    back = -(s.collar_inset + s.tooth_depth[1]) - TOOTH_FLEX
    outer = -(s.collar_inset + s.tooth_depth[0]) + TOOTH_CLEAR
    y0, y1 = s.tooth_y[0] - TOOTH_CLEAR, s.tooth_y[1] + TOOTH_CLEAR
    band = []
    for g in np.linspace(back, outer, 25):
        for y in np.linspace(y0, y1, 61):
            (px, py), (qx, qy) = _cap_frame(_cap_param_at_y(g, side, y))
            band.append((px + g * qx, py + g * qy))
    band = np.array(band)
    st = np.array([flat(p) for p in band])
    reach = CLEARANCE + WALL + 0.5     # the square ends stop 0.5 outside the socket wall
    inner = st[:, 1] <= reach
    a0, a1 = st[inner, 0].min() - 0.1, st[inner, 0].max() + 0.1
    # Beyond reach each end turns onto the cap normal through its corner. The ends move apart until those
    # normals hold the rest of the band, so each end is one kinked face with no step in it.
    params = [_cap_param_through(p, side) for p in band[~inner]]

    def normal_at(a):
        return _cap_param_through(plan(a, reach), side)

    rising = normal_at(a1) > normal_at(a0)
    low, high = (min(params), max(params)) if rising else (max(params), min(params))
    if (normal_at(a0) - low) * (1 if rising else -1) > 0:
        a0 = _bisect(lambda a: normal_at(a) - low, a0 - 20.0, a0)
    if (high - normal_at(a1)) * (1 if rising else -1) > 0:
        a1 = _bisect(lambda a: normal_at(a) - high, a1, a1 + 20.0)
    ends = []
    for a in (a1, a0):
        (px, py), (qx, qy) = _cap_frame(normal_at(a))
        ends.append((px + 8.0 * qx, py + 8.0 * qy))
    outline = Polygon([plan(a0, -12.0), plan(a1, -12.0), plan(a1, reach), *ends, plan(a0, reach)])
    assert outline.is_valid and outline.buffer(1e-6).contains(MultiPoint(band)), 'latch pocket plan must hold the hook band'
    return tuple(outline.exterior.coords[:-1])


def _tooth_band(inner, outer, z0, z1):
    """Plan band between cap-outline offsets inner and outer (negative = inward), over the hook length in y.

    checks/battery_paths.py uses it for the lip zone and the hooks' flex room.
    """
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
    The pockets reach into the stick clearance, so no thin socket wall stays
    between them (_pocket_plan).
    """
    s = _stick()
    face = -(s.collar_inset + s.tooth_depth[0])
    notch_face = face - s.tooth_notch
    lip_top = s.tooth_catch - LIP_GAP
    parts = []
    for side in (-1, 1):
        for grow, z0, z1 in ((face + TOOTH_CLEAR, lip_top, s.tooth_top + 2 * TOOTH_CLEAR),
                             (notch_face + LIP_CLEAR, s.rim_height - 1.0, lip_top + 0.01)):
            parts.append(prism('cap', grow, z0, z1).intersect(_plan_prism(_pocket_plan(side), z0 - 1, z1 + 1)))
    out = parts[0]
    for part in parts[1:]:
        out = out.fuse(part)
    return out


def _plan_prism(pts, z0, z1):
    import cadquery as cq
    face = cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(x, y, z0) for x, y in pts], close=True))
    return cq.Solid.extrudeLinear(face, cq.Vector(0, 0, z1 - z0))


def socket_cavity():
    """Stick clearance with latch pockets, contact slots and the T window (stick-local).

    Each opening runs from its floor up through the socket top, and its sides lie
    on the socket wall's normals, so the wall ends square.
    """
    s = _stick()
    top = SOCKET_TOP + 1.0
    cavity = prism('stick', CLEARANCE, s.cup_floor, top).fuse(latch_pockets())
    # T window: sides on the rear lobes' radii through x = +-T_WINDOW['x'] on the outer face.
    inner, outer = s.stick_radius + CLEARANCE - 1.0, s.stick_radius + CLEARANCE + WALL + 1.0
    pts = []
    for side, k in ((-1, 0), (1, 1)):
        cx, cy = s.stick_lobes[k]
        dx = side * T_WINDOW['x'] - cx
        ux, uy = dx / (s.stick_radius + CLEARANCE + WALL), math.sqrt(1 - (dx / (s.stick_radius + CLEARANCE + WALL)) ** 2)
        pts += [(cx + ux * inner, cy + uy * inner), (cx + ux * outer, cy + uy * outer)]
    cavity = cavity.fuse(_plan_prism([pts[0], pts[1], pts[3], pts[2]], T_WINDOW['z0'], top))
    for name in ('plus', 'minus'):
        (cx, cy), (dx, dy), (nx, ny) = contact_frame(name)
        w = CONTACT_SLOT['width'] / 2
        depth = CLEARANCE + WALL + 1.0
        pts = [(cx - dx * w - nx, cy - dy * w - ny), (cx + dx * w - nx, cy + dy * w - ny),
               (cx + dx * w + nx * depth, cy + dy * w + ny * depth), (cx - dx * w + nx * depth, cy - dy * w + ny * depth)]
        cavity = cavity.fuse(_plan_prism(pts, CONTACT_SLOT['z0'], top))
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


def _front_loop(count=480):
    """Closed boundary of the front opening on the fairing's outer surface: (count, 3) points evenly spaced.

    The sides and the arch follow the section y = FRONT_OPEN['y']; the floor
    follows the row at z0 round the front; quarter circles of the corner radius,
    laid out along the vertical wall, join them.
    """
    import numpy as np
    from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
    from OCP.gp import gp_Pnt, gp_Vec
    y_cut, z0, r = FRONT_OPEN['y'], FRONT_OPEN['z0'], FRONT_OPEN['corner']
    assert SKIRT_TOP < z0 - 1 and z0 + r < DOME_BASE, 'opening corners must lie on the straight wall'
    # Box row as arc length from its front centre to either side.
    a, b = BOX['half_width'], (BOX['back'] - BOX['front']) / 2
    cy = (BOX['back'] + BOX['front']) / 2
    t = np.linspace(0, math.pi, 20001)
    rr = (np.abs(np.sin(t) / a) ** SE_N + np.abs(np.cos(t) / b) ** SE_N) ** (-1 / SE_N)
    x, y = rr * np.sin(t), cy - rr * np.cos(t)
    sigma = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    side = float(np.interp(y_cut, y, sigma))   # sigma of the cut on the +x side (y rises with sigma here)

    def wall(s, z):   # point on the straight wall at signed arc length s (+x side for s > 0)
        return (math.copysign(float(np.interp(abs(s), sigma, x)), s), float(np.interp(abs(s), sigma, y)), z)

    # Arch: section y = y_cut, by rays in that plane from (0, y_cut, z0), each
    # solved by Newton's method from the previous hit.
    surface = _outer_surface()
    start = wall(side, z0 + r)
    u, v = GeomAPI_ProjectPointOnSurf(gp_Pnt(*start), surface).LowerDistanceParameters()
    arch = []
    for phi in np.linspace(math.atan2(r, start[0]), math.pi - math.atan2(r, start[0]), 1200):
        d = np.array([math.cos(phi), 0.0, math.sin(phi)])
        lam = float(np.dot(np.array(surface.Value(u, v).Coord()) - (0, y_cut, z0), d))
        for _ in range(30):
            p, su, sv = gp_Pnt(), gp_Vec(), gp_Vec()
            surface.D1(u, v, p, su, sv)
            f = np.array(p.Coord()) - (0, y_cut, z0) - lam * d
            if np.linalg.norm(f) < 1e-9:
                break
            du, dv, dl = np.linalg.solve(np.column_stack([su.Coord(), sv.Coord(), -d]), -f)
            u, v, lam = (u + du) % 1.0, v + dv, lam + dl
        assert np.linalg.norm(f) < 1e-6 and lam > 0, 'opening arch ray did not meet the fairing'
        arch.append((p.X(), y_cut, p.Z()))
    # +x side corner, floor round the front, -x side corner, then the arch from -x back to +x.
    quarter = np.linspace(0, math.pi / 2, 60)
    right = [wall(side - r + r * math.cos(q), z0 + r - r * math.sin(q)) for q in quarter]
    floor = [wall(s, z0) for s in np.linspace(side - r, -(side - r), 400)[1:-1]]
    left = [wall(-(side - r + r * math.cos(q)), z0 + r - r * math.sin(q)) for q in quarter[::-1]]
    loop = np.array(right + floor + left + arch[::-1][1:-1] if arch[0][0] > 0 else right + floor + left + arch[1:-1])
    # Even spacing by arc length.
    closed = np.vstack([loop, loop[:1]])
    length = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    target = np.linspace(0, length[-1], count, endpoint=False)
    return np.column_stack([np.interp(target, length, closed[:, k]) for k in range(3)])


@lru_cache(maxsize=1)
def _front_band():
    """Ruled band through the opening boundary along the outer surface normal, crossing the whole wall."""
    import numpy as np
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    from OCP.Extrema import Extrema_GenLocateExtPS
    from OCP.GeomAdaptor import GeomAdaptor_Surface
    from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
    from OCP.GeomLProp import GeomLProp_SLProps
    from OCP.gp import gp_Pnt
    surface = _outer_surface()
    loop = _front_loop()
    # Project each point from the previous one's parameters (the points are 0.3 mm apart).
    adaptor = GeomAdaptor_Surface(surface)   # the locator keeps only a reference to it
    locate = Extrema_GenLocateExtPS(adaptor, 1e-10, 1e-10)
    u, v = GeomAPI_ProjectPointOnSurf(gp_Pnt(*loop[0]), surface).LowerDistanceParameters()
    points, normals = [], []
    for p in loop:
        locate.Perform(gp_Pnt(*p), u, v)
        if not locate.IsDone() or locate.SquareDistance() > 0.05 ** 2:   # e.g. across the seam at u = 0
            locate.Perform(gp_Pnt(*p), *GeomAPI_ProjectPointOnSurf(gp_Pnt(*p), surface).LowerDistanceParameters())
        assert locate.IsDone() and locate.SquareDistance() < 0.05 ** 2, 'opening boundary must lie on the fairing'
        u, v = locate.Point().Parameter()
        props = GeomLProp_SLProps(surface, u, v, 1, 1e-9)
        points.append(props.Value().Coord())
        normals.append(props.Normal().Coord())
    points, normals = np.array(points), np.array(normals)
    # From 0.5 inside the inner wall to 1.0 outside; the fairing's smallest radius (about 3.0) bounds the inner reach.
    rows = np.array([points + t * normals for t in np.linspace(-(WALL + 0.5), 1.0, 4)])
    return BRepBuilderAPI_MakeFace(_surface(rows), 1e-6).Face()


def open_front(shell):
    """The fairing shell less its front opening: split along the band, keep the part behind it."""
    import cadquery as cq
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Splitter
    from OCP.TopTools import TopTools_ListOfShape
    split = BRepAlgoAPI_Splitter()
    arguments, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    arguments.Append(shell.wrapped)
    tools.Append(_front_band())
    split.SetArguments(arguments)
    split.SetTools(tools)
    split.Build()
    assert split.IsDone(), 'front opening split failed'
    pieces = cq.Shape.cast(split.Shape()).Solids()
    back = cq.Vector(0.0, BOX['back'] - WALL / 2, (SKIRT_TOP + DOME_BASE) / 2)
    keep = [p for p in pieces if p.isInside(back)]
    front = [p for p in pieces if not p.isInside(back)]
    assert len(keep) == 1 and len(front) == 1, f'front opening must split the shell in two, got {len(pieces)}'
    c = front[0].Center()
    assert c.y < FRONT_OPEN['y'] and c.z > FRONT_OPEN['z0'], 'front opening removed the wrong piece'
    return keep[0]


def round_outside(shell):
    """The opened shell with its seat edge and the opening's outer edge rounded to EDGE_ROUND.

    Both are whole loops here, before the socket tube joins the shell. The
    opening's inner edge stays square: it is hidden in the body, and OCCT
    cannot round it on the offset surface.
    """
    import cadquery as cq
    from OCP.BRep import BRep_Tool
    from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
    from OCP.gp import gp_Pnt
    band, outer = BRep_Tool.Surface_s(_front_band()), _outer_surface()

    def on(surface, edge):
        return all(GeomAPI_ProjectPointOnSurf(gp_Pnt(*edge.positionAt(t).toTuple()), surface).LowerDistance() < 1e-5
                   for t in (0.0, 0.5, 1.0))

    z0 = _stick().rim_height + SEAT_GAP
    seat = [f for f in shell.Faces() if f.geomType() == 'PLANE' and abs(f.Center().z - z0) < 1e-6]
    assert len(seat) == 1, 'shell needs one seat face'
    opening = [e for e in shell.Edges() if on(band, e) and on(outer, e)]
    assert opening and cq.Wire.assembleEdges(opening).IsClosed(), "front opening's outer edge must be one closed loop"
    rounded = shell.fillet(EDGE_ROUND, seat[0].outerWire().Edges() + opening)
    assert rounded.isValid() and len(rounded.Solids()) == 1, 'rounded shell must be one valid solid'
    return rounded.Solids()[0]


def tongue():
    """Front tongue clamped under the trunk plate (world)."""
    import cadquery as cq
    t = TONGUE
    body = (cq.Workplane('XY', origin=(0, 0, t['z0'])).center(0, (t['y0'] + t['y1']) / 2)
            .rect(2 * t['x'], t['y1'] - t['y0']).extrude(PLATE_BOTTOM_Z - t['z0']).edges('|Z').fillet(2.0)
            .faces('>Z or <Z').edges().fillet(EDGE_ROUND).val())
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
    shell = round_outside(open_front(fairing().cut(fairing(WALL)).Solids()[0]))
    tube = prism('stick', CLEARANCE + WALL, s.rim_height + SEAT_GAP, SOCKET_TOP)
    return shell.fuse(tube).cut(socket_cavity()).clean()


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
        # Up to 0.5 under the stick top: above the socket top, clear of the dome's inner face.
        z0, z1 = s.plate_z[0], s.stick_top - 0.5
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
