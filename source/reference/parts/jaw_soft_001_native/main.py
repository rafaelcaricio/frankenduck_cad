"""jaw_soft_001_native: soft (TPU) cover of the lower jaw, Frankenduck head.

Rebuilt natively from the Blender mesh jaw_soft.001 (robot world frame, Z up, mm).
Construction: a lower block (shallow cylindrical bottom, leaning flat sides and front,
4.32 mm bottom rounds) and an inset upper layer above a ledge, both tilted 0.50 deg
about Y like the mesh; rounds at the top edge and the ledge corner; the back is cut
flat at back_x, a 45 deg chamfer trims the top back edge and a spline notch is cut
through the full height at the back.
Raw CadQuery: the faces are tilted planes and a cylinder fitted to the mesh, which
the sketch/extrude feature form cannot express directly.
"""
import math
import cadquery as cq
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet

back_x = 69.661            # mm Back face plane x (world)
tilt = 0.00875             # mm Drop of top and ledge per mm of +x (0.50 deg about Y)
top_z_back = 270.722       # mm Top plane z at back_x
ledge_z_back = 267.7546    # mm Ledge plane z at back_x
bottom_z0 = 260.2133       # mm Bottom z at x=0, y=0 (z = bottom_z0 + bottom_kx*x + bottom_ky2*y^2)
bottom_kx = -0.0083741     # mm Bottom slope along x (mm per mm)
bottom_ky2 = 0.00025894    # mm Bottom curvature along y (1/mm; cylinder radius 1/(2k) = 1931 mm)
low_side_y = 57.4298       # mm Lower side half-width at back_x on the ledge plane
low_side_splay = 0.041516  # mm Lower side widening per mm of +x
low_side_lean = 0.054206   # mm Lower side widening per mm of +z
low_front_x = 113.2291     # mm Lower front x at ledge_z_back
low_front_lean = 0.135672  # mm Lower front forward shift per mm of +z
up_side_y = 53.8671        # mm Upper side half-width at back_x on the top plane
up_side_splay = 0.070089   # mm Upper side widening per mm of +x
up_side_lean = -0.00812    # mm Upper side widening per mm of +z
up_front_x = 111.2187      # mm Upper front x at top_z_back
up_front_lean = -0.00711   # mm Upper front forward shift per mm of +z
r_bottom = 4.32            # mm Bottom edge round (sides and front)
r_low_corner = 0.94        # mm Lower block vertical front corner round
r_up_corner = 1.75         # mm Upper block vertical front corner round
r_top = 0.945              # mm Top outer edge round
r_ledge = 0.945            # mm Concave round where the upper layer meets the ledge
back_chamfer = 1.35        # mm Top back edge chamfer (45 deg in the tilted frame)
notch_depth = 78.434       # mm Notch bottom x at y=0
notch_x_10 = 78.382        # mm Notch x at |y|=10
notch_x_17 = 78.168        # mm Notch x at |y|=17
notch_x_22 = 77.819        # mm Notch x at |y|=22
notch_x_26 = 77.248        # mm Notch x at |y|=26
notch_x_29 = 76.4          # mm Notch x at |y|=29.5
notch_x_32 = 75.233        # mm Notch x at |y|=32.5
notch_x_35 = 73.792        # mm Notch x at |y|=35
notch_x_37 = 72.181        # mm Notch x at |y|=37
notch_x_38 = 70.627        # mm Notch x at |y|=38.5
notch_end_y = 40.187       # mm Notch |y| where the curve (extended) reaches x=68.6

CENTRE = (90.0, 0.0, 265.0)
BIG = 400.0
EXT = 6.0  # build past the back face, cut after rounding


def _unit(v):
    l = math.sqrt(sum(c * c for c in v))
    return tuple(c / l for c in v)


def halfspace(p, n):
    """Large box on the inner side of plane (point p, outward normal n)."""
    n = _unit(n)
    t = sum(b * (a - c) for a, b, c in zip(p, n, CENTRE))
    o = tuple(c + t * b for c, b in zip(CENTRE, n))
    ref = (0, 0, 1) if abs(n[2]) < 0.9 else (1, 0, 0)
    xd = cq.Vector(ref).cross(cq.Vector(n)).normalized()
    return cq.Workplane(cq.Plane(origin=o, xDir=xd.toTuple(), normal=n)).rect(BIG, BIG).extrude(-BIG)


def plane_test(p, n, tol=0.02):
    n = _unit(n)
    d = sum(a * b for a, b in zip(n, p))
    return lambda q: abs(sum(a * b for a, b in zip(n, q)) - d) < tol


def samples(e, k=5):
    return [e.positionAt(i / (k - 1)).toTuple() for i in range(k)]


def pick(wp, pred):
    es = [e for e in wp.edges().vals() if pred(e)]
    assert es, "edge selection is empty"
    return wp.newObject(es)


def vertical_front_corner(e):
    d = e.endPoint() - e.startPoint()
    c = e.Center()
    return abs(d.z) > 0.8 * d.Length and c.x > 105 and abs(c.y) > 45


ext_x = back_x - EXT
# planes as (point, outward normal)
top_pl = ((back_x, 0.0, top_z_back), (tilt, 0.0, 1.0))
ledge_pl = ((back_x, 0.0, ledge_z_back), (tilt, 0.0, 1.0))
low_side_l = ((back_x, low_side_y, ledge_z_back), (-low_side_splay, 1.0, -low_side_lean))
low_side_r = ((back_x, -low_side_y, ledge_z_back), (-low_side_splay, -1.0, -low_side_lean))
low_front = ((low_front_x, 0.0, ledge_z_back), (1.0, 0.0, -low_front_lean))
up_side_l = ((back_x, up_side_y, top_z_back), (-up_side_splay, 1.0, -up_side_lean))
up_side_r = ((back_x, -up_side_y, top_z_back), (-up_side_splay, -1.0, -up_side_lean))
up_front = ((up_front_x, 0.0, top_z_back), (1.0, 0.0, -up_front_lean))

# bottom: cylinder along the tilted x direction
R = 1.0 / (2.0 * bottom_ky2)
ax_dir = cq.Vector(1, 0, bottom_kx).normalized()
ax_org = cq.Vector(0, 0, bottom_z0 + R * math.sqrt(1.0 + bottom_kx ** 2))
on_bottom = lambda q: abs(((cq.Vector(q) - ax_org) - ax_dir * (cq.Vector(q) - ax_org).dot(ax_dir)).Length - R) < 0.02
bottom_cyl = cq.Workplane(cq.Plane(origin=(ax_org + ax_dir * -150.0).toTuple(), xDir=(0, 1, 0),
                                   normal=ax_dir.toTuple())).circle(R).extrude(400)

# lower block
lower = cq.Workplane("XY").box(BIG, BIG, BIG).translate(CENTRE)
for p, n in [ledge_pl, low_side_l, low_side_r, low_front, ((ext_x, 0, 0), (-1, 0, 0))]:
    lower = lower.intersect(halfspace(p, n))
lower = lower.intersect(bottom_cyl)
# bottom rounds and vertical front corners in one fillet operation (two radii)
mk = BRepFilletAPI_MakeFillet(lower.val().wrapped)
for e in lower.edges().vals():
    if e.Center().x > ext_x + 0.5 and all(on_bottom(q) for q in samples(e)):
        mk.Add(r_bottom, e.wrapped)
    elif vertical_front_corner(e):
        mk.Add(r_low_corner, e.wrapped)
mk.Build()
lower = cq.Workplane("XY").newObject([cq.Shape.cast(mk.Shape())])

# upper layer (starts 0.6 mm below the ledge so the union overlaps)
upper = cq.Workplane("XY").box(BIG, BIG, BIG).translate(CENTRE)
for p, n in [top_pl, ((back_x, 0.0, ledge_z_back - 0.6), (-tilt, 0.0, -1.0)), up_side_l, up_side_r, up_front,
             ((ext_x - 1.0, 0, 0), (-1, 0, 0))]:
    upper = upper.intersect(halfspace(p, n))
upper = pick(upper, vertical_front_corner).fillet(r_up_corner)

body = lower.union(upper)
on_top = plane_test(*top_pl)
body = pick(body, lambda e: e.Center().x > ext_x + 0.5 and all(on_top(q) for q in samples(e))).fillet(r_top)
on_ledge = plane_test(*ledge_pl)
on_upper = [plane_test(*up_side_l), plane_test(*up_side_r), plane_test(*up_front)]


def ledge_corner(e):
    ps = samples(e)
    if e.Center().x < ext_x + 0.5 or not all(on_ledge(q) for q in ps):
        return False
    if e.geomType() == "LINE":
        return any(all(t(q) for q in ps) for t in on_upper)
    return e.Center().x < 111.5  # upper corner arcs; lower corner arcs sit at x > 112


body = pick(body, ledge_corner).fillet(r_ledge)

# back face, back chamfer, notch
body = body.cut(cq.Workplane("XY").box(40, 200, 40).translate((back_x - 20, 0, 265)))
th = math.atan(tilt)
ch_n = (-math.cos(math.pi / 4 + th), 0.0, math.sin(math.pi / 4 + th))
ch_p = (back_x + back_chamfer, 0.0, top_z_back - tilt * back_chamfer)
body = body.cut(halfspace(ch_p, (-ch_n[0], 0.0, -ch_n[2])))
half = [(notch_depth, 0.0), (notch_x_10, 10.0), (notch_x_17, 17.0), (notch_x_22, 22.0), (notch_x_26, 26.0),
        (notch_x_29, 29.5), (notch_x_32, 32.5), (notch_x_35, 35.0), (notch_x_37, 37.0), (notch_x_38, 38.5),
        (68.6, notch_end_y)]
pts = [(x, -y) for x, y in reversed(half)] + half[1:]
notch = (cq.Workplane("XY", origin=(0, 0, 250)).spline(pts, includeCurrent=False)
         .lineTo(60.0, notch_end_y).lineTo(60.0, -notch_end_y).close().extrude(30))
body = body.cut(notch)

from source_api import Model

# Flexible printed cover; TPU grade and hardness are not yet measured.
model = Model()
model.add("printed_body", body)
model.material("fdm-tpu", color="#f79507")
result = model.result()

