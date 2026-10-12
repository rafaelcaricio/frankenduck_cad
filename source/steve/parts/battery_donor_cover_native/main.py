# battery_donor_cover_native: printed U plate over the SmallRig NP-F550 donor contact block (Frankenduck), revision r02.
# Native port of cad_blender/code_cad/battery_carrier/build.py make_cover(COVER_PARAMS("r02")), robot world frame, mm, Z up.
# Sits on the donor block top and ears, holds the donor PCB down, fixed by two M3 screws into the carrier nut bosses.
import cadquery as cq
from steve import Model

x0 = -48.6  # mm Cover rear X
x1 = -37.45  # mm Cover front X (0.15 mm from the carrier upper wall at -37.3)
half_y = 25.1  # mm Body half width (between the carrier ears at |Y| 25.3)
z_body = 153.17  # mm Body underside (donor block wall top)
z_top = 158.0  # mm Cover top
wing_x0 = -48.0  # mm Screw wing rear X
wing_x1 = -38.6  # mm Screw wing front X
wing_y1 = 34.5  # mm Screw wing outer |Y|
wing_z = 155.0  # mm Screw wing underside
outer_r = 1.5  # mm Wing outer plan corner radius
ear_pad_x0 = -44.8  # mm Ear pad rear X
ear_pad_x1 = -39.55  # mm Ear pad front X
ear_pad_y0 = 23.0  # mm Ear pad inner |Y|
ear_pad_y1 = 25.0  # mm Ear pad outer |Y|
ear_pad_z = 152.0  # mm Ear pad underside (donor block ear tops)
foot_x0 = -44.5  # mm PCB foot rear X
foot_x1 = -39.55  # mm PCB foot front X
foot_y0 = 6.0  # mm PCB foot inner |Y|
foot_y1 = 21.5  # mm PCB foot outer |Y|
foot_z = 149.0  # mm PCB foot underside (PCB top 148.8 + 0.2 gap)
window_x0 = -45.6  # mm Plug window rear X (open toward +X)
window_y = 17.5  # mm Plug window half width (r02)
window_z0 = 147.8  # mm Plug window cutter bottom
screw_x = -43.3  # mm M3 screw axis X
screw_y = 31.0  # mm M3 screw axis |Y|
screw_r = 1.7  # mm M3 clearance hole radius

SIDES = (1, -1)


def box(xa, xb, ya, yb, za, zb):
    return cq.Solid.makeBox(xb - xa, yb - ya, zb - za, cq.Vector(xa, ya, za))


def span(s, a, b):
    return min(s * a, s * b), max(s * a, s * b)


def wing(s):
    # screw wing with its two outer vertical corners rounded
    w = cq.Workplane().add(box(wing_x0, wing_x1, *span(s, half_y, wing_y1), wing_z, z_top))
    return w.edges("|Z").edges(">Y" if s > 0 else "<Y").fillet(outer_r).val()


model = Model()
model.add("body", box(x0, x1, -half_y, half_y, z_body, z_top))
for s in SIDES:
    n = "p" if s > 0 else "n"
    model.add(f"wing_{n}", wing(s), op="join")
    model.add(f"ear_pad_{n}", box(ear_pad_x0, ear_pad_x1, *span(s, ear_pad_y0, ear_pad_y1), ear_pad_z, z_body + 0.5), op="join")
    model.add(f"pcb_foot_{n}", box(foot_x0, foot_x1, *span(s, foot_y0, foot_y1), foot_z, z_body + 0.5), op="join")
model.add("plug_window", box(window_x0, x1 + 1, -window_y, window_y, window_z0, z_top + 1), op="cut")
for s in SIDES:
    model.cylinder(f"screw_hole_{'p' if s > 0 else 'n'}", radius=screw_r, height=z_top - wing_z + 2,
                   at=(screw_x, s * screw_y, wing_z - 1), op="cut")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#1d1d1f")
result = model.result()

