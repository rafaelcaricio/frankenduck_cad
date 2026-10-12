"""Experimental face for MightyCamera, copied from face_part_native revision 6. Canonical untouched."""
import math
from steve import Model

# Frankenduck face plate, rebuilt natively from the Blender mesh (robot world frame, Z up).
# Plate normal is +X; sketches on YZ planes use local (Y, Z) and extrude toward +X.
sym_y = -0.12336  # mm Symmetry plane Y of the outline, standoffs and tab slots
ring_z = 301.50477  # mm Nose ring axis Z (tab slot centre)
x_back = 108.14  # mm Plate back face X
plate_t = 1.75  # mm Plate thickness (front face at x_back + plate_t)
half_w_back = 59.0426  # mm Half width of the back face at the bottom edge
half_w_front = 59.1636  # mm Half width of the front face at the bottom edge (edge draft 0.121 mm)
z_arch = 274.4997  # mm Z where the vertical side edges meet the arch
z_bottom = 273.425  # mm Flat bottom edge Z
lens_z = 301.50477  # mm Lens opening centre Z (at Y 0)
lens_r = 8.5  # mm Lens opening radius
small_z = 296.05  # mm Small round opening centre Z (at Y 0)
small_r = 2.85  # mm Small round opening radius
slot_y = 30.252  # mm Stadium opening centre Y
slot_z = 298.805  # mm Stadium opening centre Z
slot_half = 3.2065  # mm Half distance between the stadium arc centres
slot_r = 4.05  # mm Stadium opening end radius
tab_ri = 11.4  # mm Tab slot inner radius
tab_ro = 12.885  # mm Tab slot outer radius
tab_w = 20.68  # deg Tab slot angular width
so_dy = 39.042  # mm Standoff Y offset from the symmetry plane
so_dz = 15.4375  # mm Standoff Z offset from the ring axis
so_x0 = 93.015  # mm Standoff free end X
so_r = 4.05  # mm Standoff radius
so_ch = 0.67  # mm 45 deg chamfer on the standoff free end
so_cb_r = 2.0  # mm Standoff counterbore radius
so_cb_x = 98.22  # mm Standoff counterbore bottom X
so_pilot_r = 1.39  # mm Standoff pilot hole radius
so_pilot_x = 99.77  # mm Standoff pilot hole end X (start of the 118 deg drill tip)
so_tip_x = 100.6  # mm Standoff drill tip apex X
boss_y = 10.5  # mm Screw boss Y offset from Y 0
boss_z_hi = 317.0  # mm Upper screw boss pair Z
boss_z_lo = 304.5  # mm Lower screw boss pair Z
boss_x0 = 102.9  # mm Screw boss end X
boss_r = 2.8  # mm Screw boss radius
boss_collar_r = 3.3  # mm Screw boss root collar radius
boss_collar_x = 107.8  # mm Screw boss root collar start X
boss_fr = 0.5  # mm Fillet between boss and collar
boss_hole_r = 0.8  # mm Screw boss pilot radius
boss_hole_x = 108.3  # mm Screw boss pilot bottom X (inside the plate)
tof_board_length = 24.81  # mm Physical board measured by caliper, including mounting ears
tof_board_width = 10.16  # mm Physical PCB width, excludes header pins
tof_mount_pitch = 18.72  # mm User-confirmed mounting-hole centre distance
tof_sensor_length = 7.02  # mm Measured bare sensor length; no cover dimensions supplied
ear_l_y = slot_y - tof_mount_pitch / 2  # mm Left ToF screw centre
ear_r_y = slot_y + tof_mount_pitch / 2  # mm Right ToF screw centre
ear_z = 298.805  # mm Camera ear centre Z
tof_post_length = 2.5  # mm Shorter ToF posts move the board toward the outside of the face
ear_x0 = x_back - tof_post_length  # mm ToF PCB front seating plane
ear_r = 3.0  # mm ToF M3 boss seat radius, within measured mounting ear envelope
ear_fr = 0.6  # mm Camera ear root fillet radius
ear_hole_r = 1.2  # mm Provisional M3 self-tapping pilot radius; validate exact screw in print
tof_pilot_depth = 3.0  # mm Provisional blind M3 pilot; actual screw length remains to be checked
ear_hole_x = ear_x0 + tof_pilot_depth  # mm Leaves 1.25 mm of material to the face front
rim_x0 = 106.785  # mm Stadium rim end X
rim_r = 5.4  # mm Stadium rim outer end radius
rim_hole_r = 4.117  # mm Stadium rim inner end radius
notch_z = 301.505  # mm Rim notch circle centre Z (at the ear Y positions)
notch_r = 4.85  # mm Rim notch circle radius
clip_x0 = 105.5  # mm Camera clip end X
clip_lip_x = 106.2  # mm Camera clip hook lip X
clip_pad_x = 107.6  # mm Camera clip root pad start X
clip_z0 = 290.75  # mm Camera clip bottom Z
clip_z1 = 301.4  # mm Camera clip top Z

model = Model()

# ---------------- plate ----------------
model.plane("plane_back", on="YZ", offset=x_back, name="Plate back plane")

with model.sketch("sketch_plate_back", on="plane_back", name="Plate outline, back face") as s:
    s.line("edge_r", (sym_y + half_w_back, z_bottom), (sym_y + half_w_back, z_arch))
    s.spline("arch", [(sym_y + half_w_back, z_arch), (58.7563, 279.2580), (57.4053, 289.6711), (53.8059, 303.5096), (47.7028, 317.8332), (37.0203, 328.2220), (21.9259, 332.4900), (7.5648, 333.5904), (-7.8115, 333.5904), (-22.1727, 332.4900), (-37.2670, 328.2220), (-47.9495, 317.8332), (-54.0526, 303.5096), (-57.6520, 289.6711), (-59.0030, 279.2580), (sym_y - half_w_back, z_arch)])
    s.line("edge_l", (sym_y - half_w_back, z_arch), (sym_y - half_w_back, z_bottom))
    s.line("bottom", (sym_y - half_w_back, z_bottom), (sym_y + half_w_back, z_bottom))

model.plane("plane_front", on="YZ", offset=x_back + plate_t, name="Plate front plane")

with model.sketch("sketch_plate_front", on="plane_front", name="Plate outline, front face") as s:
    s.line("edge_r", (sym_y + half_w_front, z_bottom), (sym_y + half_w_front, z_arch))
    s.spline("arch", [(sym_y + half_w_front, z_arch), (58.8631, 279.3734), (57.5347, 289.5824), (53.9258, 303.5737), (47.7588, 317.9338), (37.1072, 328.3004), (21.9672, 332.6177), (7.5449, 333.7116), (-7.7917, 333.7116), (-22.2140, 332.6177), (-37.3539, 328.3004), (-48.0055, 317.9338), (-54.1726, 303.5737), (-57.7814, 289.5824), (-59.1098, 279.3734), (sym_y - half_w_front, z_arch)])
    s.line("edge_l", (sym_y - half_w_front, z_arch), (sym_y - half_w_front, z_bottom))
    s.line("bottom", (sym_y - half_w_front, z_bottom), (sym_y + half_w_front, z_bottom))

model.loft("loft_plate", ["sketch_plate_back", "sketch_plate_front"], ruled=True, name="Plate (drafted edge)")

with model.sketch("sketch_openings", on="plane_back", name="Plate openings") as s:
    s.circle("lens", (sym_y, lens_z), lens_r)
    s.arc("st_r", (slot_y + slot_half, slot_z), slot_r, -90.0, 90.0)
    s.line("st_t", (slot_y + slot_half, slot_z + slot_r), (slot_y - slot_half, slot_z + slot_r))
    s.arc("st_l", (slot_y - slot_half, slot_z), slot_r, 90.0, 270.0)
    s.line("st_b", (slot_y - slot_half, slot_z - slot_r), (slot_y + slot_half, slot_z - slot_r))
    for k, a0 in enumerate((90.0, 210.0, 330.0)):
        a1 = math.radians(a0 - tab_w / 2)
        a2 = math.radians(a0 + tab_w / 2)
        s.arc(f"tab_o{k}", (sym_y, ring_z), tab_ro, a0 - tab_w / 2, a0 + tab_w / 2)
        s.arc(f"tab_i{k}", (sym_y, ring_z), tab_ri, a0 - tab_w / 2, a0 + tab_w / 2)
        s.line(f"tab_a{k}", (sym_y + tab_ri * math.cos(a1), ring_z + tab_ri * math.sin(a1)), (sym_y + tab_ro * math.cos(a1), ring_z + tab_ro * math.sin(a1)))
        s.line(f"tab_b{k}", (sym_y + tab_ri * math.cos(a2), ring_z + tab_ri * math.sin(a2)), (sym_y + tab_ro * math.cos(a2), ring_z + tab_ro * math.sin(a2)))

model.extrude("cut_openings", "sketch_openings", distance=plate_t, op="cut", name="Plate openings")

# ---------------- measured ToF screw bosses, reinforced with root fillets ----------------
# Comment-driven trial: posts shortened from 4.055 to 2.5 mm; board moves forward 1.555 mm.
# Optical cover, PCB thickness and sensor height are not physically measured.
# Old rear locating rim and upper bracket removed to leave access around the smaller PCB.
model.plane("plane_ear", on="XY", offset=ear_z, name="Ear axis plane")
model.axis("axis_ear_l", through=(0.0, ear_l_y, ear_z), direction=(1, 0, 0), name="Left ear axis")
model.axis("axis_ear_r", through=(0.0, ear_r_y, ear_z), direction=(1, 0, 0), name="Right ear axis")

for side, ey in (("l", ear_l_y), ("r", ear_r_y)):
    with model.sketch(f"sketch_ear_{side}", on="plane_ear", name=f"Ear profile {side}") as s:
        s.line("e1", (ear_x0, ey), (ear_x0, ey + ear_r))
        s.line("e2", (ear_x0, ey + ear_r), (x_back - ear_fr, ey + ear_r))
        s.arc("e3", (x_back - ear_fr, ey + ear_r + ear_fr), ear_fr, -90.0, 0.0)
        s.line("e4", (x_back, ey + ear_r + ear_fr), (x_back, ey))
        s.line("e5", (x_back, ey), (ear_x0, ey))
    model.revolve(f"revolve_ear_{side}", f"sketch_ear_{side}", axis=f"axis_ear_{side}", op="join", name=f"Ear {side}")

model.plane("plane_ear_end", on="YZ", offset=ear_x0, name="Ear and bracket end plane")

# ---- MightyCamera board mounts; measured import holes: (2,2), (31,2), (2,28), (31,28).
# Board +Z faces the face; board X runs across Y and board Y runs up Z. USB-C at -Y is downward.
camera_pcb_back_x = 98.2  # mm Camera PCB rear face; connector rear extent at 94.834
camera_pcb_t = 1.5162  # mm Measured PCB thickness
mount_seat_x = camera_pcb_back_x + camera_pcb_t  # mm Front PCB seating surface
mount_r = 2.0  # mm Seat radius, fits the 2 mm corner land around each 2.2 mm hole
mount_root_r = 3.0  # mm Wider root reinforcing the face
mount_pilot_r = 0.8  # mm Provisional M2 self-tapping pilot; tune for printing
mount_pilot_depth = 5.0  # mm Blind pilot leaves 5.174 mm to face front
model.plane("plane_mighty_seat", on="YZ", offset=mount_seat_x)
with model.sketch("sketch_mighty_seats", on="plane_mighty_seat") as s:
    for k, (yy, zz) in enumerate(((sym_y - 14.5, ring_z + 12.5),
                                  (sym_y - 14.5, ring_z - 13.5),
                                  (sym_y + 14.5, ring_z + 12.5),
                                  (sym_y + 14.5, ring_z - 13.5))):
        s.circle(f"seat{k}", (yy, zz), mount_r)
model.extrude("mighty_mount_posts", "sketch_mighty_seats", distance=x_back - mount_seat_x, op="join")
model.plane("plane_mighty_roots", on="YZ", offset=x_back - 1.5)
with model.sketch("sketch_mighty_roots", on="plane_mighty_roots") as s:
    for k, (yy, zz) in enumerate(((sym_y - 14.5, ring_z + 12.5),
                                  (sym_y - 14.5, ring_z - 13.5),
                                  (sym_y + 14.5, ring_z + 12.5),
                                  (sym_y + 14.5, ring_z - 13.5))):
        s.circle(f"root{k}", (yy, zz), mount_root_r)
model.extrude("mighty_mount_roots", "sketch_mighty_roots", distance=1.5, op="join")
with model.sketch("sketch_mighty_pilots", on="plane_mighty_seat") as s:
    for k, (yy, zz) in enumerate(((sym_y - 14.5, ring_z + 12.5),
                                  (sym_y - 14.5, ring_z - 13.5),
                                  (sym_y + 14.5, ring_z + 12.5),
                                  (sym_y + 14.5, ring_z - 13.5))):
        s.circle(f"pilot{k}", (yy, zz), mount_pilot_r)
model.extrude("mighty_mount_pilots", "sketch_mighty_pilots", distance=mount_pilot_depth, op="cut")

# ---------------- standoffs (revolved, with chamfer, counterbore, pilot and drill tip) ----------------
for zname, sz in (("hi", ring_z + so_dz), ("lo", ring_z - so_dz)):
    model.plane(f"plane_so_{zname}", on="XY", offset=sz, name=f"Standoff plane {zname}")
    for side, sy in (("l", sym_y - so_dy), ("r", sym_y + so_dy)):
        model.axis(f"axis_so_{zname}_{side}", through=(0.0, sy, sz), direction=(1, 0, 0), name=f"Standoff axis {zname} {side}")
        with model.sketch(f"sketch_so_{zname}_{side}", on=f"plane_so_{zname}", name=f"Standoff profile {zname} {side}") as s:
            s.line("s1", (so_x0, sy + so_cb_r), (so_x0, sy + so_r - so_ch))
            s.line("s2", (so_x0, sy + so_r - so_ch), (so_x0 + so_ch, sy + so_r))
            s.line("s3", (so_x0 + so_ch, sy + so_r), (x_back, sy + so_r))
            s.line("s4", (x_back, sy + so_r), (x_back, sy))
            s.line("s5", (x_back, sy), (so_tip_x, sy))
            s.line("s6", (so_tip_x, sy), (so_pilot_x, sy + so_pilot_r))
            s.line("s7", (so_pilot_x, sy + so_pilot_r), (so_cb_x, sy + so_pilot_r))
            s.line("s8", (so_cb_x, sy + so_pilot_r), (so_cb_x, sy + so_cb_r))
            s.line("s9", (so_cb_x, sy + so_cb_r), (so_x0, sy + so_cb_r))
        model.revolve(f"revolve_so_{zname}_{side}", f"sketch_so_{zname}_{side}", axis=f"axis_so_{zname}_{side}", op="join", name=f"Standoff {zname} {side}")

# ---------------- measured ToF mount, blind M3 self-tapping pilots ----------------
with model.sketch("sketch_ear_holes", on="plane_ear_end", name="Ear pilot holes") as s:
    s.circle("eh1", (ear_l_y, ear_z), ear_hole_r)
    s.circle("eh2", (ear_r_y, ear_z), ear_hole_r)

model.extrude("cut_ear_holes", "sketch_ear_holes", distance=ear_hole_x - ear_x0, op="cut", name="Ear pilot holes")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#9b9891")
result = model.result()

