"""jaw (R05 = jaw_r01) - native rebuild of the Frankenduck jaw, robot world frame, mm, CAD pose.

Source: cad_blender/code_cad/head_internals/jaw_r01/build_jaw.py (build123d), used unchanged by head internals
R05. Same construction, same numbers, same coordinates:
 1. Envelope: floor cylinder (R 1945 bow) cut by the two tilted cheek planes, the tilted lip plane, a rear and a
    top plane; outer fillets r6.75 (floor-cheek, floor-lip) and r3.375 (lip-cheek).
 2. Shell: envelope minus the 2.3 mm inner envelope (open top and rear).
 3. Floor rear edge: spline cut in plan.
 4. Side profiles in (x, z): top edge, r10.8 blend, 60 deg arm band, hinge circle r12.15 (bearing side) or hub
    r15.2 with r3 junction fillets (servo side), r9.849 heel arc.
 5. Servo hub r15.2 (y 54.0..59.6) + horn boss r10 (y 53..54), 20 deg skirt into the arm, clip face r22 at y 59.6.
    Two M3 clearance holes d3.3 + d6.3 counterbores (head seat y 56), centre clearance d6.4 x 3.2.
 6. Bearing journal d7.80 (y -54..-48) for the jaw F688 inner race, land r5.4, shoulder r10.8 relieved 0.3,
    root fillet r1.0, axial d3.4 hole, M3 hex nut trap AF5.7.
The jaw has no heat-set insert pockets: its M3 screws pass through clearance holes into the servo horn and into a
captive hex nut, so no self-forming thread hole applies. The R05 F688 seat change (16.55 bore) and the M2 keeper
pilots are on the head frame, not on the jaw.
"""
import math
import cadquery as cq
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCP.gp import gp_Ax2, gp_Circ, gp_Dir, gp_Pnt, gp_Vec
from source_api import Model

# jaw axis (SG92R_jaw_axis): origin on the servo horn side, direction toward -Y with a 0.0025 tilt
axis_x = 35.235  # mm Jaw axis origin X
axis_y = 54.0  # mm Jaw axis origin Y (servo hub inner face)
axis_z = 284.630  # mm Jaw axis origin Z
axis_tilt = 0.0025  # mm Jaw axis Z change per mm along -Y
# envelope
cheek_d = 42.825  # mm Cheek outer plane offset (n.p with n = (-0.0415, +-0.9978, -0.0523))
lip_d = 78.954  # mm Lip outer plane offset (n.p with n = (0.9911, 0, -0.1332))
top_d = 270.106  # mm Top edge plane offset (n.p with n = (0.0087, 0, 1))
floor_r = 1945.0  # mm Floor bow radius
floor_slope = -0.0095  # mm Floor bottom centre line dz per mm of x
floor_z90 = 257.158  # mm Floor bottom z at x 90, y 0
wall = 2.3  # mm Shell thickness
r_bottom = 6.75  # mm Outer floor-cheek and floor-lip fillet
r_corner = 3.375  # mm Outer lip-cheek fillet
x_rear = 12.0  # mm Rear plane of the outer envelope (shell opening)
z_top = 304.0  # mm Top plane of the outer envelope (shell opening)
# side profile
r_hinge = 12.15  # mm Hinge circle radius (arm outline centre on the jaw axis)
r_servo_hub = 15.2  # mm Servo hub radius
r_top_blend = 10.8  # mm Top edge to upper arm edge blend
r_junction = 3.0  # mm Arm edge to servo hub fillet
r_heel = 9.849  # mm Arm lower-back edge heel arc radius
s_heel = 17.751  # mm Heel arc start, distance along the arm from the hinge tangent point
arm_angle = 30.0  # deg Arm band normal angle from +X (arm runs at 60 deg)
profile_x_far = 135.0  # mm Front end of the side profile slab
profile_z_low = 245.0  # mm Bottom of the side profile slab
# servo hub and horn
y_horn = 53.0  # mm Horn contact face Y
y_hub_in = 54.0  # mm Hub inner face Y
y_hub_out = 59.6  # mm Hub outer face Y
r_boss = 10.0  # mm Horn contact boss radius
hole_d = 3.3  # mm M3 clearance hole diameter (horn screws)
cbore_d = 6.3  # mm Horn screw counterbore diameter
y_seat = 56.0  # mm Horn screw head seat Y
centre_d = 6.4  # mm Centre output-screw clearance diameter
centre_depth = 3.2  # mm Centre output-screw clearance depth from the horn face
r_clip = 22.0  # mm Hub face clip radius at y 59.6
skirt_deg = 20.0  # deg Hub-to-arm conical skirt angle
# bearing journal
y_journal_end = -48.0  # mm Journal inner end Y
y_shoulder = -54.0  # mm Journal shoulder Y (F688 inner race land)
journal_d = 7.80  # mm Journal diameter (F688 bore)
r_land = 5.4  # mm Inner-race land radius
relief = 0.3  # mm Shoulder boss relief from the outer race
r_shoulder = 10.8  # mm Shoulder boss radius
y_boss_end = -58.0  # mm Shoulder boss outer end Y
root_fillet = 1.0  # mm Shoulder boss root fillet
axial_d = 3.4  # mm Axial M3 clearance hole diameter
hex_af = 5.7  # mm M3 hex nut trap across flats
y_hex_floor = -56.5  # mm Hex nut trap floor Y

model = Model()

C0 = cq.Vector(axis_x, axis_y, axis_z)
AX = cq.Vector(0.0, -1.0, axis_tilt).normalized()
HUB_C = (axis_x, axis_z)
CHEEK_N_P = cq.Vector(-0.0415, 0.9978, -0.0523)
CHEEK_N_M = cq.Vector(-0.0415, -0.9978, -0.0523)
LIP_N = cq.Vector(0.9911, 0.0, -0.1332)
TOP_N = cq.Vector(0.0087, 0.0, 1.0)
N_ARM = (math.cos(math.radians(arm_angle)), math.sin(math.radians(arm_angle)))
D_ARM = (math.sin(math.radians(arm_angle)), -math.cos(math.radians(arm_angle)))
HORN_HOLES = [cq.Vector(28.273, 53.023, 283.903), cq.Vector(42.197, 53.027, 285.366)]  # jaw_direct_horn_M3x6_+-7
FLOOR_REAR_EDGE = [(78.44, 0.0), (78.41, 8.1), (78.17, 17.4), (77.49, 24.7), (76.39, 29.6), (75.12, 32.8),
                   (73.59, 35.3), (71.62, 37.6), (69.17, 39.6), (66.83, 41.2), (63.76, 42.8), (59.43, 44.66),
                   (55.51, 46.08), (51.81, 47.27), (47.06, 48.7), (42.10, 50.1), (39.35, 50.86), (37.85, 51.6)]


# ---------------------------------------------------------------------------------------------- 2D/3D helpers
def _v2(a):
    return (a[0], a[1])


def _add(a, b, k=1.0):
    return (a[0] + k * b[0], a[1] + k * b[1])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _norm(a):
    return math.hypot(a[0], a[1])


def _clean(shape):
    return cq.Shape.cast(shape).clean()


def _common(a, b):
    return _clean(BRepAlgoAPI_Common(a.wrapped, b.wrapped).Shape())


def _cut(a, b):
    return _clean(BRepAlgoAPI_Cut(a.wrapped, b.wrapped).Shape())


def _fuse(a, b):
    return _clean(BRepAlgoAPI_Fuse(a.wrapped, b.wrapped).Shape())


def _ax2(o, z, x):
    return gp_Ax2(gp_Pnt(o.x, o.y, o.z), gp_Dir(z.x, z.y, z.z), gp_Dir(x.x, x.y, x.z))


def _cylinder(r, h, o, z, x):
    return cq.Solid(BRepPrimAPI_MakeCylinder(_ax2(o, z, x), r, h).Shape())


def _cone(r0, r1, h, o, z, x):
    return cq.Solid(BRepPrimAPI_MakeCone(_ax2(o, z, x), r0, r1, h).Shape())


def _prism(face, vec):
    return cq.Solid(BRepPrimAPI_MakePrism(face.wrapped, gp_Vec(vec.x, vec.y, vec.z)).Shape())


def axis_point(y):
    return C0 + AX * ((y - C0.y) / AX.y)


def axis_x_dir():
    return cq.Vector(0, axis_tilt, 1.0).normalized()


def cyl_between(r, y_a, y_b, through=None):
    """cylinder of radius r about the jaw axis direction between world y_a and y_b"""
    hi, lo = max(y_a, y_b), min(y_a, y_b)
    o = axis_point(hi) if through is None else through + AX * ((hi - through.y) / AX.y)
    return _cylinder(r, (hi - lo) / abs(AX.y), o, AX, axis_x_dir())


def halfspace(n, d, size=700.0):
    """{p: n.p <= d} as a large box"""
    nn = n.normalized()
    dd = d / n.Length
    centre = cq.Vector(60, 0, 280)
    o = centre - nn * (nn.dot(centre) - dd)
    xd = (nn.cross(cq.Vector(0, 0, 1)) if abs(nn.z) < 0.9 else nn.cross(cq.Vector(1, 0, 0))).normalized()
    zd = -nn
    yd = zd.cross(xd)
    o = o - xd * (size / 2) - yd * (size / 2)
    return cq.Solid(BRepPrimAPI_MakeBox(_ax2(o, zd, xd), size, size, size / 2).Shape())


def P(x, z):
    return cq.Vector(x, 0, z)


def ztop(x):
    return (top_d - TOP_N.x * x) / TOP_N.z


def line_circle_fillet(c, rc, off, rf, side):
    a = off + side * rf
    s = math.sqrt((rc + rf) ** 2 - a ** 2)
    q = _add(_add(c, N_ARM, a), D_ARM, s)
    t_line = _add(_add(c, N_ARM, off), D_ARM, s)
    qc = (q[0] - c[0], q[1] - c[1])
    t_circ = _add(c, qc, rc / _norm(qc))
    return q, t_line, t_circ


def arc3(p1, pc, p2, r):
    m = ((p1[0] + p2[0]) / 2 - pc[0], (p1[1] + p2[1]) / 2 - pc[1])
    m = _add(pc, m, r / _norm(m))
    return cq.Edge.makeThreePointArc(P(*p1), P(*m), P(*p2))


def _line(a, b):
    return cq.Edge.makeLine(P(*a), P(*b))


def side_profile(servo):
    """(x, z) outline of one cheek, arm and hub, in the y = 0 plane"""
    c = HUB_C
    t_u = _add(c, N_ARM, r_hinge)
    t_l = _add(c, N_ARM, -r_hinge)
    q_h = _add(_add(t_l, D_ARM, s_heel), N_ARM, r_heel)
    h1 = _add(t_l, D_ARM, s_heel)
    h2 = (q_h[0], q_h[1] - r_heel)
    p_bot = (h2[0], profile_z_low)
    tn = math.hypot(TOP_N.x, TOP_N.z)
    nt = (TOP_N.x / tn, TOP_N.z / tn)
    # solve [N_ARM; nt] q = b
    b0 = r_hinge + r_top_blend + _dot(c, N_ARM)
    b1 = top_d / tn + r_top_blend
    det = N_ARM[0] * nt[1] - N_ARM[1] * nt[0]
    qb = ((b0 * nt[1] - N_ARM[1] * b1) / det, (N_ARM[0] * b1 - nt[0] * b0) / det)
    tb_top = _add(qb, nt, -r_top_blend)
    tb_arm = _add(qb, N_ARM, -r_top_blend)
    pts = [p_bot, (profile_x_far, profile_z_low), (profile_x_far, ztop(profile_x_far)), tb_top]
    edges = [_line(a, b) for a, b in zip(pts[:-1], pts[1:])]
    edges.append(arc3(tb_top, qb, tb_arm, r_top_blend))
    if not servo:
        edges.append(_line(tb_arm, t_u))
        edges.append(cq.Edge.makeThreePointArc(P(*t_u), P(c[0], c[1] + r_hinge), P(*t_l)))
        edges.append(_line(t_l, h1))
    else:
        qu, tl_u, tc_u = line_circle_fillet(c, r_servo_hub, r_hinge, r_junction, +1)
        ql, tl_l, tc_l = line_circle_fillet(c, r_servo_hub, -r_hinge, r_junction, -1)
        edges.append(_line(tb_arm, tl_u))
        edges.append(arc3(tl_u, qu, tc_u, r_junction))
        a0 = math.atan2(tc_u[1] - c[1], tc_u[0] - c[0])
        a1 = math.atan2(tc_l[1] - c[1], tc_l[0] - c[0])
        if a1 < a0:
            a1 += 2 * math.pi
        am = (a0 + a1) / 2
        mid = (c[0] + r_servo_hub * math.cos(am), c[1] + r_servo_hub * math.sin(am))
        edges.append(cq.Edge.makeThreePointArc(P(*tc_u), P(*mid), P(*tc_l)))
        edges.append(arc3(tc_l, ql, tl_l, r_junction))
        edges.append(_line(tl_l, h1))
    edges.append(arc3(h1, q_h, h2, r_heel))
    edges.append(_line(h2, p_bot))
    return cq.Face.makeFromWires(cq.Wire.assembleEdges(edges))


def plan_trim():
    half = FLOOR_REAR_EDGE
    pts = [cq.Vector(x, -y, 0) for x, y in reversed(half[1:])] + [cq.Vector(x, y, 0) for x, y in half]
    spl = cq.Edge.makeSpline(pts)
    y_end = pts[-1].y
    yc = 51.6
    L = lambda a, b: cq.Edge.makeLine(cq.Vector(*a), cq.Vector(*b))
    e = [spl, L((37.85, y_end, 0), (0, yc, 0)), L((0, yc, 0), (0, 75, 0)), L((0, 75, 0), (140, 75, 0)),
         L((140, 75, 0), (140, -75, 0)), L((140, -75, 0), (0, -75, 0)), L((0, -75, 0), (0, -yc, 0)),
         L((0, -yc, 0), (37.85, -y_end, 0))]
    return cq.Face.makeFromWires(cq.Wire.assembleEdges(e))


def envelope(off, rb, rc, xr, zt):
    """tray envelope moved inward by off (0 = outer skin, wall = inner skin)"""
    d = cq.Vector(1.0, 0.0, floor_slope).normalized()
    x0 = xr - 2.0
    zc0 = floor_z90 + floor_slope * (x0 - 90.0) + floor_r * math.sqrt(1 + floor_slope ** 2)
    env = _cylinder(floor_r - off, 170.0 / d.x, cq.Vector(x0, 0, zc0), d, cq.Vector(0, 1, 0))
    env = _common(env, halfspace(CHEEK_N_P, cheek_d - off * CHEEK_N_P.Length))
    env = _common(env, halfspace(CHEEK_N_M, cheek_d - off * CHEEK_N_M.Length))
    env = _common(env, halfspace(LIP_N, lip_d - off * LIP_N.Length))
    env = _common(env, halfspace(cq.Vector(-1, 0, 0), -xr))
    env = _common(env, halfspace(cq.Vector(0, 0, 1), zt))
    env = env.Solids()[0]
    bottom = min(env.Faces(), key=lambda f: f.Center().z)
    eb = [e for e in bottom.Edges() if e.positionAt(0.5).x > xr + 2.0]
    ln = LIP_N.normalized()
    lipf = [f for f in env.Faces() if abs(f.normalAt(f.Center()).normalized().dot(ln) - 1) < 1e-4][0]
    corners = [e for e in lipf.Edges() if abs(e.positionAt(0.5).y) > 45 and abs(e.endPoint().z - e.startPoint().z) > 5]
    mk = BRepFilletAPI_MakeFillet(env.wrapped)
    for e in eb:
        mk.Add(rb, e.wrapped)
    for e in corners:
        mk.Add(rc, e.wrapped)
    mk.Build()
    return cq.Solid(mk.Shape())


def jaw_body():
    # 1-2. tray shell
    outer = envelope(0.0, r_bottom, r_corner, x_rear, z_top)
    inner = envelope(wall, r_bottom - wall, r_corner - wall, 0.0, 320.0)
    part = _cut(outer, inner)
    # 3. floor rear edge
    part = _common(part, _prism(plan_trim().moved(cq.Location(cq.Vector(0, 0, 240.0))), cq.Vector(0, 0, 70.0)))
    # 4. side profiles (servo side: profile minus the hub disc)
    fs, fb = side_profile(True), side_profile(False)
    circ = gp_Circ(gp_Ax2(gp_Pnt(HUB_C[0], 0, HUB_C[1]), gp_Dir(0, -1, 0), gp_Dir(1, 0, 0)), r_servo_hub)
    hub_disc = cq.Face.makeFromWires(cq.Wire.assembleEdges([cq.Edge(BRepBuilderAPI_MakeEdge(circ).Edge())]))
    ring_faces = fs.cut(hub_disc).Faces()
    trim = _prism(fb.moved(cq.Location(cq.Vector(0, -80.0, 0))), cq.Vector(0, 80.0, 0))
    ring = None
    for f in ring_faces:
        pr = _prism(f, cq.Vector(0, 80.0, 0))
        ring = pr if ring is None else _fuse(ring, pr)
    trim = _fuse(trim, ring)
    part = _common(part, trim)
    # 5. servo hub + horn boss, conical skirt into the arm, clip face
    part = _fuse(part, _fuse(cyl_between(r_servo_hub, y_hub_in, 61.0), cyl_between(r_boss, y_horn, y_hub_in + 1.0)))
    ta = math.tan(math.radians(skirt_deg))
    rad_at = lambda y: r_servo_hub + (y_hub_out + 0.2 - y) / ta
    y_b, y_t = y_hub_out - 3.0, y_hub_out + 0.3
    cone = _cone(rad_at(y_b), rad_at(y_t), (y_t - y_b) / abs(AX.y), axis_point(y_b), -AX, axis_x_dir())
    skirt = _common(cone, halfspace(-CHEEK_N_P, -(cheek_d - wall * CHEEK_N_P.Length)))
    skirt = _common(skirt, ring)
    part = _fuse(part, skirt)
    part = _cut(part, cyl_between(r_clip, y_hub_out, 70.0))
    # 6. bearing journal + shoulder root fillet
    part = _fuse(part, cyl_between(r_shoulder, y_shoulder - relief, y_boss_end))
    part = _fuse(part, cyl_between(r_land, y_shoulder, y_shoulder - relief - 0.15))
    part = _fuse(part, cyl_between(journal_d / 2, y_journal_end, y_shoulder - 0.15))
    a, n_in = AX, cq.Vector(-0.0415, -0.9978, -0.0523)

    def on_root(e):
        for i in range(7):
            p = e.positionAt(i / 6.0)
            v = p - C0
            if abs((v - a * v.dot(a)).Length - r_shoulder) > 0.02:
                return False
            if abs((n_in.dot(p) - 40.528) / n_in.Length) > 0.02:
                return False
        return True

    root = [e for e in part.Edges() if on_root(e)]
    return part.Solids()[0].fillet(root_fillet, root)


def hex_trap():
    o = axis_point(y_hex_floor)
    xd = axis_x_dir()
    yd = AX.cross(xd)
    rad = hex_af / math.sqrt(3)
    pts = [o + xd * (rad * math.cos(i * math.pi / 3)) + yd * (rad * math.sin(i * math.pi / 3)) for i in range(6)]
    face = cq.Face.makeFromWires(cq.Wire.makePolygon(pts, close=True))
    return _prism(face, AX * 8.0)


model.add("body", jaw_body(), body="jaw", name="Tray shell, side profiles, servo hub, bearing journal")
for i, h in enumerate(HORN_HOLES):
    model.add(f"horn_hole_{i + 1}", cyl_between(hole_d / 2, 52.0, y_seat + 0.05, through=h), op="cut", body="jaw",
              name="Horn screw M3 clearance")
    model.add(f"horn_cbore_{i + 1}", cyl_between(cbore_d / 2, y_seat, 66.0, through=h), op="cut", body="jaw",
              name="Horn screw head counterbore")
model.add("centre_hole", cyl_between(centre_d / 2, 52.0, y_horn + centre_depth), op="cut", body="jaw",
          name="Servo output screw clearance")
model.add("axial_hole", cyl_between(axial_d / 2, -46.0, y_hex_floor - 0.5), op="cut", body="jaw",
          name="Journal axial M3 clearance")
model.add("hex_trap", hex_trap(), op="cut", body="jaw", name="M3 hex nut trap")
# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#f96c0a")
result = model.result()

