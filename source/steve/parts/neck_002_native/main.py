# neck_002_native: driven-side (+Y) plate of the neck servo bracket (Frankenduck).
# Native rebuild of the Blender mesh neck.002, robot world frame, mm, Z up.
# Every dimension below was measured from the mesh (sections, circle and fillet fits).
# Raw CadQuery: the underside pocket needs a filleted cutting tool and a faced-off slab.
import math
import cadquery as cq

x_min = 20.8  # mm Plate X minimum
x_max = 49.4  # mm Plate X maximum
z_min = 187.019  # mm Plate Z minimum
z_max = 212.519  # mm Plate Z maximum
corner_r = 4.5  # mm Plate outline corner radius (XZ)
y_face = 14.4  # mm Washer boss face (slab bottom before facing)
y_pad = 14.8  # mm Side pad and bar face after facing
y_floor = 15.9  # mm Underside pocket floor
y_top = 20.3  # mm Outer (+Y) face
edge_r = 0.65  # mm Outline edge fillet on both faces (bottom one truncated by the facing)
hole_x0 = 24.85  # mm Left hole column X
hole_x1 = 45.35  # mm Right hole column X
hole_z0 = 192.769  # mm Lower hole row Z
hole_z1 = 206.769  # mm Upper hole row Z
hole_r = 1.1  # mm Through hole radius
cb_r = 2.2  # mm Counterbore radius (from the +Y face)
cb_y = 16.1  # mm Counterbore floor
boss_r = 2.6  # mm Washer boss radius (Y 14.4..14.8)
slot_x0 = 27.7  # mm Pocket left wall X at the pad face
slot_x1 = 42.5  # mm Pocket right wall X at the pad face
wall_draft = 8.93  # deg Pocket side wall draft (pocket narrows toward the floor)
bar_z = 199.769  # mm Cross bar centre Z
bar_hw = 1.437  # mm Cross bar half width at the pad face
bar_draft = 22.5  # deg Cross bar flank draft
root_r = 0.75  # mm Pocket root fillet (walls and bar flanks to the floor)
tp_hx = 5.322  # mm Thin floor pad half width in X (about X 35.1)
tp_hz = 5.689  # mm Thin floor pad inner edge distance from the bar centre
tp_y = 15.75  # mm Thin floor pad face

xc = (x_min + x_max) / 2
zc = (z_min + z_max) / 2
pts = [(hole_x0, hole_z0), (hole_x0, hole_z1), (hole_x1, hole_z0), (hole_x1, hole_z1)]


def prism_xz(profile, y0, y1):
    # XZ workplane: local x = X, local y = Z, normal -Y; extrude from y1 down to y0
    return profile(cq.Workplane("XZ", origin=(0, y1, 0))).extrude(y1 - y0)


# slab with rounded outline and edge fillets on both faces
body = prism_xz(lambda w: w.center(xc, zc).rect(x_max - x_min, z_max - z_min), y_face, y_top)
body = body.edges("|Y").fillet(corner_r).faces(">Y or <Y").edges().fillet(edge_r)

# face the bottom down to the pad level, leaving the four washer bosses
facing = prism_xz(lambda w: w.center(xc, zc).rect(60, 60), y_face - 1, y_pad)
facing = facing.cut(prism_xz(lambda w: w.pushPoints(pts).circle(boss_r), y_face - 1, y_pad))
body = body.cut(facing)

# underside pocket: drafted side walls, drafted cross bar, filleted root
kw = math.tan(math.radians(wall_draft))
kb = math.tan(math.radians(bar_draft))
y_lo = y_pad - 0.8
trap = (cq.Workplane("XY", origin=(0, 0, z_min - 1))
        .polyline([(slot_x0 - kw * (y_pad - y_lo), y_lo), (slot_x1 + kw * (y_pad - y_lo), y_lo),
                   (slot_x1 - kw * (y_floor - y_pad), y_floor), (slot_x0 + kw * (y_floor - y_pad), y_floor)])
        .close().extrude(z_max - z_min + 2))
bar = (cq.Workplane("YZ", origin=(x_min - 1, 0, 0))
       .polyline([(y_lo, bar_z - bar_hw + kb * (y_pad - y_lo)), (y_lo, bar_z + bar_hw - kb * (y_pad - y_lo)),
                  (y_floor + 0.5, bar_z + bar_hw + kb * (y_floor + 0.5 - y_pad)),
                  (y_floor + 0.5, bar_z - bar_hw - kb * (y_floor + 0.5 - y_pad))])
       .close().extrude(x_max - x_min + 2))
pocket = trap.cut(bar)
pocket = pocket.edges(cq.selectors.BoxSelector((0, y_pad - 0.3, z_min - 0.5), (100, 30, z_max + 0.5), boundingbox=False)).fillet(root_r)
body = body.cut(pocket)

# thin floor pads at both pocket ends
for z0, z1 in ((z_min, bar_z - tp_hz), (bar_z + tp_hz, z_max)):
    body = body.union(cq.Workplane("XY", origin=(0, 0, z0)).center(35.1, (tp_y + y_floor + 0.05) / 2)
                      .rect(2 * tp_hx, y_floor + 0.05 - tp_y).extrude(z1 - z0))

# through holes and counterbores from the +Y face
body = body.cut(prism_xz(lambda w: w.pushPoints(pts).circle(hole_r), y_face - 1, y_top + 1))
body = body.cut(prism_xz(lambda w: w.pushPoints(pts).circle(cb_r), cb_y, y_top + 1))

from steve import Model

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model = Model()
model.add("printed_body", body)
model.material("pla", color="#8a8a90")
result = model.result()

