# neck_001_native: passive-side plate of the upper neck servo bracket (Frankenduck).
# Native rebuild of the Blender mesh neck.001, robot world frame, mm, Z up.
# Every dimension was measured from the mesh (sections, circle fits).
# Raw CadQuery: the part needs tapered (drafted) cuts and region-selected fillets.
import math
import cadquery as cq

cx = 35.1  # mm Plate centre X (upper neck servo horn axis)
cz = 199.769  # mm Plate centre Z
plate_w = 28.6  # mm Plate width along X
plate_h = 18.0  # mm Plate height along Z
corner_r = 4.5  # mm Plate outline corner radius (XZ)
y_back = -22.4  # mm Back face (passive side, -Y)
y_front = -18.0  # mm Front face (toward the servo)
back_r = 0.7  # mm Back face perimeter fillet
bevel_d = 0.5  # mm Front perimeter bevel depth along Y
bevel_in = 0.2  # mm Front perimeter bevel inset in X/Z
rec_x0 = 25.5  # mm Recess side wall X at the front face (mirrored about cx)
rec_z0 = 205.67  # mm Recess end wall Z at the front face (mirrored about cz)
rec_floor = -19.9  # mm Recess floor Y
rec_draft = 12.0  # deg Recess wall draft
rec_fillet = 0.6  # mm Recess floor and wall-corner fillet
hole_dx = 10.25  # mm Hole offset from cx along X
hole_dz = 3.25  # mm Hole offset from cz along Z
cb_d = 4.4  # mm Counterbore diameter (from the back face)
cb_y = -19.5  # mm Counterbore floor Y
hole_d = 2.2  # mm Through hole diameter
boss_r = 2.6  # mm Front boss radius around each hole
boss_y = -17.6  # mm Front boss top Y

holes = [(cx + sx * hole_dx, cz + sz * hole_dz) for sx in (-1, 1) for sz in (-1, 1)]


def xz(y):
    # XZ workplane at world Y = y: local x = X, local y = Z, normal -Y
    return cq.Workplane("XZ", origin=(0, y, 0))


# plate body: back part with the back fillet, then a tapered 0.5 mm front bevel
y_mid = y_front - bevel_d
base = xz(y_mid).center(cx, cz).rect(plate_w, plate_h).extrude(y_mid - y_back)
base = base.edges("|Y").fillet(corner_r).faces("<Y").edges().fillet(back_r)
bevel_ang = math.degrees(math.atan(bevel_in / bevel_d))
front_pl = cq.Plane(origin=(0, y_mid, 0), xDir=(1, 0, 0), normal=(0, 1, 0))  # local y = -Z
bevel = (cq.Workplane(front_pl).center(cx, -cz).sketch().rect(plate_w, plate_h).vertices().fillet(corner_r)
         .finalize().extrude(bevel_d, taper=bevel_ang))
body = base.union(bevel)
body = body.union(xz(boss_y).pushPoints(holes).circle(boss_r).extrude(boss_y - y_front))

# top and bottom drafted recesses in the front face
rec_w = 2 * (cx - rec_x0)
rec_len = 8.0
for s in (1, -1):
    zc = cz + s * (rec_z0 - cz + rec_len / 2)
    cut = xz(y_front).center(cx, zc).rect(rec_w, rec_len).extrude(y_front - rec_floor, taper=rec_draft)
    body = body.cut(cut)

floor_edges = [e for e in body.faces(cq.selectors.NearestToPointSelector((cx, rec_floor, cz + 7.5))).edges().vals()]
floor_edges += [e for e in body.faces(cq.selectors.NearestToPointSelector((cx, rec_floor, cz - 7.5))).edges().vals()]
floor_edges = [e for e in floor_edges if abs(abs(e.Center().z - cz) - plate_h / 2) > 0.05]
corner_edges = [e for e in body.edges().vals()  # drafted wall-to-wall corners of both recesses
                if rec_floor < e.Center().y < y_front and abs(abs(e.Center().x - cx) - (cx - rec_x0)) < 0.6
                and abs(abs(e.Center().z - cz) - (rec_z0 - cz)) < 0.6]
body = body.newObject(floor_edges + corner_edges).fillet(rec_fillet)

# counterbored through holes (counterbore from the back face)
body = body.cut(xz(cb_y).pushPoints(holes).circle(cb_d / 2).extrude(cb_y - y_back + 1))
body = body.cut(xz(boss_y + 1).pushPoints(holes).circle(hole_d / 2).extrude(boss_y - y_back + 2))

from steve import Model

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model = Model()
model.add("printed_body", body)
model.material("pla", color="#8a8a90")
result = model.result()

