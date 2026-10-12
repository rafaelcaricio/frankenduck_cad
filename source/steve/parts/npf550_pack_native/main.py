# npf550_pack_native: SmallRig NP-F550 battery pack envelope plus the reused donor contact block
# (purchased), two bodies. Robot world frame, mm, Z up. Pack back face sits on the carrier floor X -39.8.
import cadquery as cq
from steve import Model

pack_x0 = -62.6  # mm pack front face X
pack_x1 = -39.8  # mm pack back face X (carrier seating floor)
pack_hy = 19.25  # mm pack half width
pack_z0 = 75.0  # mm pack bottom
pack_z1 = 145.6  # mm pack top
pack_edge_r = 2.0  # mm pack vertical edge radius
wall_x0 = -39.3  # mm donor block wall front X
wall_x1 = -37.3  # mm donor block wall back X (carrier seat back)
wall_hy = 25.0  # mm donor block half width
wall_z0 = 141.8  # mm donor block bottom
wall_z1 = 153.17  # mm donor block top
ear_x0 = -45.36  # mm donor block ear front X
ear_y0 = 23.0  # mm donor block ear inner |Y|
ear_z0 = 145.8  # mm donor block ear bottom
ear_z1 = 152.0  # mm donor block ear top
relief_x = -40.1  # mm gold pin relief axis X
relief_y = 17.25  # mm gold pin relief axis |Y|
relief_r = 2.0  # mm gold pin relief radius
relief_z1 = 149.5  # mm gold pin relief top
hole_y = 10.0  # mm M2.5 hole axis |Y|
hole_z = 145.0  # mm M2.5 hole axis Z
hole_r = 1.35  # mm M2.5 hole radius

model = Model()
pack = cq.Workplane("XY").box(pack_x1 - pack_x0, 2 * pack_hy, pack_z1 - pack_z0, centered=False) \
    .translate((pack_x0, -pack_hy, pack_z0)).edges("|Z").fillet(pack_edge_r)
model.add("pack", pack, op="new", body="pack")

model.box("wall", length=wall_x1 - wall_x0, width=2 * wall_hy, height=wall_z1 - wall_z0,
          at=(wall_x0, -wall_hy, wall_z0), centered=False, op="new", body="donor_block")
for s in (-1, 1):
    model.box(f"ear_{s + 1}", length=wall_x0 - ear_x0, width=wall_hy - ear_y0, height=ear_z1 - ear_z0,
              at=(ear_x0, min(s * ear_y0, s * wall_hy), ear_z0), centered=False, op="join", body="donor_block")
for s in (-1, 1):
    model.cylinder(f"pin_relief_{s + 1}", radius=relief_r, height=relief_z1 - wall_z0 + 0.5, at=(relief_x, s * relief_y, wall_z0 - 0.5),
                   op="cut", body="donor_block")
    model.cylinder(f"m25_hole_{s + 1}", radius=hole_r, height=wall_x1 - wall_x0 + 1, at=(wall_x0 - 0.5, s * hole_y, hole_z),
                   direction=(1, 0, 0), op="cut", body="donor_block")
result = model.result()

