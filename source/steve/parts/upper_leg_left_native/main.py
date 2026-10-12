# upper_leg_native: Frankenduck thigh (upper leg), one source for both sides.
# Native rebuild of the Blender meshes upper_leg_left.001 / upper_leg_right.001,
# robot world frame, mm, Z up. The right thigh is the exact mirror of the left
# about Y = 0 (measured: 0.000 mm surface distance), so SIDE is the only difference.
# The thigh is a 3 deg drafted tray that holds the rear covers of two STS3215
# servos: the hip-pitch servo (horn axis at X 5.4, Z 100.375) and the knee servo
# (horn axis at X -42.899, Z 70.675), both horn axes along Y, horn faces at
# |Y| = 51.975. Its outline is the hull of three equal circles, two of them on the
# servo axes; eight screw bosses meet the servo rear flanges (|Y| = 87.175).
# Every dimension below was measured from the mesh (sections, plane and circle fits).
import math
import cadquery as cq
from steve import Model

SIDE = 1  # 1 = left thigh (+Y), -1 = right thigh (mirror about Y = 0)

# outline: hull of three circles of equal radius, drafted along Y
hip_x = 5.4  # mm Hip-pitch servo horn axis X (outline circle A centre)
hip_z = 100.375  # mm Hip-pitch servo horn axis Z
knee_x = -42.899  # mm Knee servo horn axis X (outline circle B centre)
knee_z = 70.675  # mm Knee servo horn axis Z
corner_x = -41.938  # mm Top-left outline circle C centre X (Z = hip_z)
y_open = 57.375  # mm Open (horn-side) end of the wall extensions, |Y|
y_back = 95.175  # mm Outer face of the back plate, |Y|
r_open = 18.181  # mm Outline circle radius at y_open (outer surface)
draft = 3.0  # deg Wall draft angle; the outline shrinks toward the back plate
wall = 1.35  # mm Wall and back plate thickness
back_r = 5.0  # mm Outer round between the back plate and the side walls
# skirt ends and wall extensions
skirt_y = 83.975  # mm Skirt end |Y| around the two servo arcs
diag_y = 84.975  # mm Skirt end |Y| along the middle of the diagonal wall
diag_x0 = -19.88  # mm Diagonal notch start X at skirt_y
diag_x1 = -17.36  # mm Diagonal notch start X at diag_y
diag_x2 = 7.24  # mm Diagonal notch end X at diag_y
diag_x3 = 9.96  # mm Diagonal notch end X at skirt_y
ext_slope = 0.36397  # mm Edge slope of both wall extensions per mm of |Y| (tan 20 deg)
top_edge_x0 = -26.836  # mm Top extension edge: x = top_edge_x0 + ext_slope * |y|
left_edge_z0 = 104.798  # mm Left extension edge: z = left_edge_z0 - ext_slope * |y|
left_tip_r = 13.502  # mm Left extension round between the edge and the open end
left_tip_z = 93.375  # mm Left extension round tangent point on the open end
top_tip_x = -15.6  # mm Top extension: edge curve start on the open end
tail_r = 1.7  # mm Round where both extension edges meet the skirt end
# back plate details
pad_r = 6.747  # mm Raised pads on the plate inside, on both servo axes
pad_t = 0.15  # mm Pad height
boss_r = 3.243  # mm Screw boss outer radius
boss_y = 87.175  # mm Screw boss tip |Y| (servo rear flange face)
boss_fillet = 1.25  # mm Screw boss root round
hole_r = 1.1  # mm Screw clearance hole radius
cb_r = 2.2  # mm Screw head counterbore radius (open on the outside)
cb_y = 90.175  # mm Counterbore floor |Y|
rib_y = 84.475  # mm Locating rib tip |Y|
# boss axes: the rear-flange holes of both servos
bosses = [(-32.649, 78.975), (-53.149, 78.975), (-32.649, 103.425), (-53.149, 103.425),
          (-2.9, 110.625), (-2.9, 90.125), (-27.35, 110.625), (-27.35, 90.125)]
# locating ribs (x0, x1, z0, z1), plate to rib_y; they bear on the servo cases
ribs = [(-57.111, -55.511, 97.175, 102.175), (-57.111, -55.511, 64.675, 69.675),
        (-45.899, -39.899, 106.037, 107.637), (-45.899, -39.899, 58.713, 60.313),
        (-31.562, -29.962, 107.375, 111.375), (-30.287, -28.687, 76.675, 80.675),
        (-30.287, -28.687, 64.675, 69.675), (-26.1, -21.1, 86.163, 87.763),
        (-26.1, -21.1, 112.987, 114.587), (6.4, 11.4, 86.163, 87.763),
        (6.4, 11.4, 112.987, 114.587), (15.762, 17.362, 97.375, 103.375)]
# top extension edge curve, measured points (x, |y|), tangent to the edge line at the end
top_curve = [(-12.576, 57.677), (-8.382, 59.35), (-4.637, 62.755), (-2.839, 65.97), (-2.09, 67.989)]

model = Model()


def side(s):
    return s.mirror("XZ") if SIDE < 0 else s


def y_cyl(r, y0, y1, x, z):
    return cq.Workplane().add(cq.Solid.makeCylinder(r, y1 - y0, cq.Vector(x, y0, z), cq.Vector(0, 1, 0)))


# drafted outline: hull of circles A, C, B (counter-clockwise in X-Z), on the open-end plane
circ = [(hip_x, hip_z), (corner_x, hip_z), (knee_x, knee_z)]


def hull_wire(w, r):
    n = len(circ)
    tang = []
    for i in range(n):
        (px, pz), (qx, qz) = circ[i], circ[(i + 1) % n]
        L = math.hypot(qx - px, qz - pz)
        nx, nz = (qz - pz) / L, -(qx - px) / L  # outward normal of a CCW hull
        tang.append(((px + r * nx, pz + r * nz), (qx + r * nx, qz + r * nz), (nx, nz)))
    loc = lambda p: (-p[0], p[1])  # plane xDir = -X, yDir = +Z
    w = w.moveTo(*loc(tang[0][0]))
    for i in range(n):
        a, b, nrm = tang[i]
        w = w.lineTo(*loc(b))
        nxt = tang[(i + 1) % n]
        cx, cz = circ[(i + 1) % n]
        mx, mz = nrm[0] + nxt[2][0], nrm[1] + nxt[2][1]
        k = math.hypot(mx, mz)
        w = w.threePointArc(loc((cx + r * mx / k, cz + r * mz / k)), loc(nxt[0]))
    return w.close()


open_plane = cq.Plane(origin=(0, y_open, 0), xDir=(-1, 0, 0), normal=(0, 1, 0))
outer = hull_wire(cq.Workplane(open_plane), r_open).extrude(y_back - y_open, taper=draft)
outer = outer.faces(">Y").edges().fillet(back_r)
tray = outer.faces("<Y").shell(-wall)
model.add("tray", side(tray))

# trim: everything below the skirt end goes, except the top and left wall extensions
s20 = 1 / math.hypot(ext_slope, 1)
t_top = (ext_slope * s20, s20)  # top edge direction, increasing |y|
c_top = (top_edge_x0 + ext_slope * skirt_y, skirt_y)
L = tail_r / math.tan(math.radians(55))
bis = ((-t_top[0] + 1) / math.hypot(-t_top[0] + 1, -t_top[1]), -t_top[1] / math.hypot(-t_top[0] + 1, -t_top[1]))
cc = (c_top[0] + bis[0] * tail_r / math.sin(math.radians(55)), c_top[1] + bis[1] * tail_r / math.sin(math.radians(55)))
keep_top = (cq.Workplane("XY", origin=(0, 0, 100)).moveTo(-80, 50).lineTo(top_tip_x, 50).lineTo(top_tip_x, y_open)
            .spline(top_curve, tangents=[(1, 0), t_top], includeCurrent=True)
            .lineTo(c_top[0] - L * t_top[0], c_top[1] - L * t_top[1])
            .threePointArc((cc[0] - bis[0] * tail_r, cc[1] - bis[1] * tail_r), (c_top[0] + L, c_top[1]))
            .lineTo(40, skirt_y).lineTo(40, 90).lineTo(-80, 90).close().extrude(25))
t_l = (s20, -ext_slope * s20)  # left edge direction (y, z), increasing |y|
c_l = (skirt_y, left_edge_z0 - ext_slope * skirt_y)
bl = ((-t_l[0]) / math.hypot(-t_l[0], -t_l[1] - 1), (-t_l[1] - 1) / math.hypot(-t_l[0], -t_l[1] - 1))
cl = (c_l[0] + bl[0] * tail_r / math.sin(math.radians(55)), c_l[1] + bl[1] * tail_r / math.sin(math.radians(55)))
tip_c = (y_open + left_tip_r, left_tip_z)
foot = (tip_c[0] - left_tip_r * ext_slope * s20, tip_c[1] - left_tip_r * s20)
tip_m = (tip_c[0] - left_tip_r * math.cos(math.radians(55)), tip_c[1] - left_tip_r * math.sin(math.radians(55)))
keep_left = (cq.Workplane("YZ", origin=(-75, 0, 0)).moveTo(50, 140).lineTo(50, left_tip_z).lineTo(y_open, left_tip_z)
             .threePointArc(tip_m, foot)
             .lineTo(c_l[0] - L * t_l[0], c_l[1] - L * t_l[1])
             .threePointArc((cl[0] - bl[0] * tail_r, cl[1] - bl[1] * tail_r), (c_l[0], c_l[1] - L))
             .lineTo(skirt_y, 30).lineTo(90, 30).lineTo(90, 140).close().extrude(27))
trim = cq.Workplane("XY").box(200, skirt_y - 40, 200, centered=(True, False, True)).translate((0, 40, 100))
trim = trim.cut(keep_top).cut(keep_left)
model.add("trim", side(trim), op="cut")

# diagonal skirt notch up to diag_y; cutter plane through the diagonal wall
# (common tangent of circles A and B: slope (hip_z - knee_z) / (hip_x - knee_x), Z 77.472 at X 0)
dg = math.atan2(hip_z - knee_z, hip_x - knee_x)
diag_plane = cq.Plane(origin=(0, 0, 77.472), xDir=(math.cos(dg), 0, math.sin(dg)), normal=(-math.sin(dg), 0, math.cos(dg)))
s_ = lambda x: x / math.cos(dg)
notch = (cq.Workplane(diag_plane).polyline([(s_(diag_x0), 83.0), (s_(diag_x0), skirt_y), (s_(diag_x1), diag_y),
                                            (s_(diag_x2), diag_y), (s_(diag_x3), skirt_y), (s_(diag_x3), 83.0)])
         .close().extrude(4, both=True))
model.add("diag_notch", side(notch), op="cut")

# plate details: pads, ribs, bosses with root rounds (kept inside the outer envelope)
plate_in = y_back - wall
feat = y_cyl(pad_r, plate_in - pad_t, plate_in + 0.2, hip_x, hip_z).union(
    y_cyl(pad_r, plate_in - pad_t, plate_in + 0.2, knee_x, knee_z))
for x0, x1, z0, z1 in ribs:
    feat = feat.union(cq.Workplane("XY").box(x1 - x0, plate_in + 0.2 - rib_y, z1 - z0, centered=False)
                      .translate((x0, rib_y, z0)))
f = boss_fillet
ring_prof = (cq.Workplane("XY").moveTo(boss_r, plate_in + 0.05).lineTo(boss_r, plate_in - f)
             .threePointArc((boss_r + f - f * math.sqrt(0.5), plate_in - f + f * math.sqrt(0.5)), (boss_r + f, plate_in))
             .lineTo(boss_r + f, plate_in + 0.05).close())
ring = ring_prof.revolve(360, (0, 0, 0), (0, 1, 0))
for bx, bz in bosses:
    feat = feat.union(y_cyl(boss_r, boss_y, plate_in + 0.2, bx, bz))
    feat = feat.union(ring.translate((bx, 0, bz)))
feat = feat.intersect(outer)
model.add("plate_features", side(feat), op="join")

# screw holes: clearance hole from the boss tip, counterbore open on the outside
holes = None
for bx, bz in bosses:
    h = y_cyl(hole_r, boss_y - 1, cb_y + 0.1, bx, bz).union(y_cyl(cb_r, cb_y, y_back + 1, bx, bz))
    holes = h if holes is None else holes.union(h)
model.add("screw_holes", side(holes), op="cut")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#99deef")
result = model.result()

