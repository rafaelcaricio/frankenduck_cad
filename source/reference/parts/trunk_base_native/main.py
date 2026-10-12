# trunk_base_native: printed trunk base (the deck) of the Frankenduck, revision r02 (canonical since 2026-09-28).
# Native port of cad_blender/code_cad/trunk_base/build.py make_base(REV_R02()), robot world frame, mm, Z up.
# Carries the two hip-yaw STS3215 cases on 8 lands, the neck_internal_yoke on 4 M3x4x5 inserts,
# the body shells (PA2x8 pilots + front lugs) and the battery carrier (4 x M3).
import cadquery as cq
from source_api import Model

# deck plate
plate_x0 = -35.0  # mm Plate rear face X
plate_x1 = 24.0  # mm Plate front face X
plate_half_y = 37.0  # mm Plate half width
plate_z0 = 158.45  # mm Plate underside Z
plate_z1 = 164.45  # mm Deck top Z (yoke and shell lips sit here)
plate_corner_r = 6.0  # mm Plate plan corner radius
front_strip_x0 = 5.0  # mm From this X forward the plate is narrower
front_half_y = 34.5  # mm Plate half width for X >= front_strip_x0
# battery carrier pads under the rear corners
pad_x0 = -35.7  # mm Pad rear X
pad_x1 = -27.7  # mm Pad front X
pad_y0 = 12.0  # mm Pad inner |Y|
pad_y1 = 29.0  # mm Pad outer |Y|
pad_z0 = 155.45  # mm Pad bottom (carrier deck top)
pad_z1 = 159.0  # mm Pad top
pad_corner_r = 2.0  # mm Pad plan corner radius
# servo-case lands (8)
land_x_a = -24.65  # mm Rear land column X
land_x_b = -0.2  # mm Front land column X
land_y_a = 13.375  # mm Inner land |Y|
land_y_b = 33.875  # mm Outer land |Y|
land_boss_r = 2.75  # mm Land boss radius (r02, was 2.4)
land_boss_z0 = 155.45  # mm Land boss bottom
land_boss_z1 = 158.25  # mm Land boss top / collar bottom
land_collar_r = 2.85  # mm Land collar radius
land_collar_z1 = 162.45  # mm Land collar top
land_bore_r = 1.1  # mm M2 case-screw bore radius
land_well_r = 2.7  # mm Head well radius above the seat
land_seat_z = 162.45  # mm Screw-head seat Z
# openings
window_x0 = -17.0  # mm Central window rear X
window_x1 = 12.0  # mm Central window front X
window_half_y = 13.0  # mm Central window half width
window_r = 5.0  # mm Central window corner radius
slot_x0 = -10.9  # mm Side slot rear X
slot_x1 = -2.65  # mm Side slot front X
slot_y0 = 13.125  # mm Side slot inner |Y|
slot_y1 = 34.125  # mm Side slot outer |Y|
slot_r = 2.5  # mm Side slot corner radius
big_hole_x = 8.1  # mm Front cable hole X
big_hole_y = 23.625  # mm Front cable hole |Y|
big_hole_r = 5.0  # mm Front cable hole radius
# neck_internal_yoke M3x4x5 insert pilots
yoke_x_a = -16.0  # mm Rear insert pilot X
yoke_x_b = 18.0  # mm Front insert pilot X
yoke_y = 28.0  # mm Insert pilot |Y|
yoke_pilot_r = 2.35  # mm Insert pilot radius (4.7 diameter)
yoke_floor_z = 159.95  # mm Insert pilot floor
pilot_pad_r = 3.35  # mm Pad under each insert pilot (r02)
pilot_pad_z0 = 158.25  # mm Pad underside (r02)
# battery carrier fixing
batt_x = -31.7  # mm Battery screw X
batt_y_a = 18.0  # mm Inner battery screw |Y|
batt_y_b = 25.0  # mm Outer battery screw |Y|
batt_hole_r = 1.6  # mm M3 through hole radius
batt_cb_r = 2.6  # mm Counterbore radius (from the underside up)
# shell PA2x8 pilots (along Y) and front screw lugs
shell_pilot_x_a = -20.0  # mm Rear shell pilot X
shell_pilot_x_b = 10.0  # mm Front shell pilot X
shell_pilot_z = 162.0  # mm Shell pilot axis Z
shell_pilot_r = 0.8  # mm Shell pilot radius
shell_pilot_y0 = 31.0  # mm Shell pilot floor |Y|
lug_x = 10.0  # mm Front lug axis X
lug_z = 162.0  # mm Front lug axis Z
lug_r = 3.0  # mm Front lug radius
lug_y1 = 37.0  # mm Front lug outer |Y|
lug_end_fillet = 0.25  # mm Front lug end edge round
# front head-well wall tabs (r02)
well_tab_r = 4.4  # mm Tab radius about the outer front land axis
well_tab_y0 = 36.0  # mm Tab starts at this |Y|
# front neck-servo notch (inclined plane over a Y range)
chamfer_y0 = -20.6  # mm Notch Y start
chamfer_y1 = 17.6  # mm Notch Y end
chamfer_top_x = 6.137  # mm Notch line X at the deck top
chamfer_bottom_x = 14.3908  # mm Notch line X at the underside
# clearances under the yoke front
recess_z = 164.25  # mm Recess floor (0.2 below the yoke)
recess_x0 = 20.75  # mm Recess rear X
recess_half_y = 27.0  # mm Recess box half width
recess_disc_x = 18.0  # mm Recess disc centre X
recess_disc_r = 7.0  # mm Recess disc radius
neg_notch_x0 = 23.6  # mm -Y notch rear X
neg_notch_y0 = -25.7  # mm -Y notch Y start
neg_notch_y1 = -20.6  # mm -Y notch Y end
neg_notch_z = 160.369  # mm -Y notch floor
pocket_a_x = 24.0898  # mm +Y pocket disc centre X
pocket_a_y = 21.1209  # mm +Y pocket disc centre Y
pocket_a_r = 3.511  # mm +Y pocket disc radius
pocket_b_x = 24.435  # mm +Y pocket island centre X
pocket_b_y = 23.7614  # mm +Y pocket island centre Y
pocket_b_r = 2.1914  # mm +Y pocket island radius
pocket_floor_x = 20.4429  # mm +Y pocket floor cylinder axis X
pocket_floor_z = 160.995  # mm +Y pocket floor cylinder axis Z
pocket_floor_r = 3.8283  # mm +Y pocket floor cylinder radius

SIDES = (1, -1)
LANDS = [(x, s * y) for x in (land_x_a, land_x_b) for y in (land_y_a, land_y_b) for s in SIDES]


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def cylz(x, y, r, z0, z1):
    return cq.Solid.makeCylinder(r, z1 - z0, cq.Vector(x, y, z0), cq.Vector(0, 0, 1))


def cyly(x, z, r, y0, y1):
    return cq.Solid.makeCylinder(r, y1 - y0, cq.Vector(x, y0, z), cq.Vector(0, 1, 0))


def rrect(x0, x1, y0, y1, r, z0, z1):
    return (cq.Workplane("XY", origin=(0, 0, z0)).center((x0 + x1) / 2, (y0 + y1) / 2)
            .rect(x1 - x0, y1 - y0).extrude(z1 - z0).edges("|Z").fillet(r).val())


def group(solids):
    out = solids[0]
    for s in solids[1:]:
        out = out.fuse(s)
    return out.clean()


model = Model()

# deck plate with rounded corners; narrower front strip
model.add("plate", rrect(plate_x0, plate_x1, -plate_half_y, plate_half_y, plate_corner_r, plate_z0, plate_z1))
model.add("front_strips", group([box(front_strip_x0, plate_x1 + 1, s * front_half_y if s > 0 else -plate_half_y - 1,
                                     plate_half_y + 1 if s > 0 else -front_half_y, plate_z0 - 1, plate_z1 + 1) for s in SIDES]), op="cut")
# battery carrier pads
model.add("battery_pads", group([rrect(pad_x0, pad_x1, *sorted((s * pad_y0, s * pad_y1)), pad_corner_r, pad_z0, pad_z1)
                                 for s in SIDES]), op="join")
# through openings: window, side slots, front cable holes
openings = [rrect(window_x0, window_x1, -window_half_y, window_half_y, window_r, 150, 170)]
for s in SIDES:
    openings.append(rrect(slot_x0, slot_x1, *sorted((s * slot_y0, s * slot_y1)), slot_r, 150, 170))
    openings.append(cylz(big_hole_x, s * big_hole_y, big_hole_r, 150, 170))
model.add("openings", group(openings), op="cut")
# servo-case lands: boss under the plate and collar
model.add("land_bosses", group([cylz(x, y, land_boss_r, land_boss_z0, land_boss_z1) for x, y in LANDS]
                               + [cylz(x, y, land_collar_r, land_boss_z1, land_collar_z1) for x, y in LANDS]), op="join")
# front screw lugs: cylinder along Y, rounded end edge, top cut flat at the deck top
lugs = []
for s in SIDES:
    if s > 0:
        wp = cq.Workplane(cq.Plane(origin=(lug_x, lug_y1, lug_z), xDir=(1, 0, 0), normal=(0, -1, 0)))
        lug = wp.circle(lug_r).extrude(lug_y1 - front_half_y + 0.5).faces(">Y").edges().fillet(lug_end_fillet).val()
    else:
        wp = cq.Workplane(cq.Plane(origin=(lug_x, -lug_y1, lug_z), xDir=(1, 0, 0), normal=(0, 1, 0)))
        lug = wp.circle(lug_r).extrude(lug_y1 - front_half_y + 0.5).faces("<Y").edges().fillet(lug_end_fillet).val()
    lugs.append(lug.intersect(box(lug_x - 5, lug_x + 5, -40, 40, 150, plate_z1)))
model.add("front_lugs", group(lugs), op="join")
# r02: pads under the insert pilots, D-tabs outside the outer front head wells
model.add("pilot_pads", group([cylz(x, s * yoke_y, pilot_pad_r, pilot_pad_z0, plate_z0 + 0.5)
                               for x in (yoke_x_a, yoke_x_b) for s in SIDES]), op="join")
model.add("well_tabs", group([cylz(land_x_b, s * land_y_b, well_tab_r, plate_z0, plate_z1).intersect(
    box(land_x_b - 5, land_x_b + 5, *sorted((s * well_tab_y0, s * (land_y_b + well_tab_r + 1))), plate_z0 - 1, plate_z1 + 1))
    for s in SIDES]), op="join")
# land bores and head wells
model.add("land_holes", group([cylz(x, y, land_bore_r, 150, land_seat_z + 0.5) for x, y in LANDS]
                              + [cylz(x, y, land_well_r, land_seat_z, 170) for x, y in LANDS]), op="cut")
# yoke insert pilots
model.add("yoke_pilots", group([cylz(x, s * yoke_y, yoke_pilot_r, yoke_floor_z, 170)
                                for x in (yoke_x_a, yoke_x_b) for s in SIDES]), op="cut")
# battery carrier holes and counterbores
model.add("battery_holes", group([cylz(batt_x, s * y, batt_hole_r, 150, 170) for y in (batt_y_a, batt_y_b) for s in SIDES]
                                 + [cylz(batt_x, s * y, batt_cb_r, plate_z0, 170) for y in (batt_y_a, batt_y_b) for s in SIDES]), op="cut")
# shell PA2x8 pilots along Y
model.add("shell_pilots", group([cyly(x, shell_pilot_z, shell_pilot_r, shell_pilot_y0, lug_y1 + 1) for x in (shell_pilot_x_a, shell_pilot_x_b)]
                                + [cyly(x, shell_pilot_z, shell_pilot_r, -lug_y1 - 1, -shell_pilot_y0) for x in (shell_pilot_x_a, shell_pilot_x_b)]),
          op="cut")
# front neck-servo notch: inclined plane through the top and bottom lines
k = (chamfer_bottom_x - chamfer_top_x) / (plate_z1 - plate_z0)
xz = lambda z: chamfer_top_x + (plate_z1 - z) * k
zlo, zhi = plate_z0 - 2, plate_z1 + 2
prof = cq.Wire.makePolygon([cq.Vector(xz(zhi), chamfer_y0, zhi), cq.Vector(xz(zlo), chamfer_y0, zlo),
                            cq.Vector(40, chamfer_y0, zlo), cq.Vector(40, chamfer_y0, zhi)], close=True)
model.add("front_notch", cq.Solid.extrudeLinear(cq.Face.makeFromWires(prof), cq.Vector(0, chamfer_y1 - chamfer_y0, 0)), op="cut")
# clearances under the yoke front: recess, -Y notch, +Y pocket
rec = [box(recess_x0, 30, -recess_half_y, recess_half_y, recess_z, 170)]
rec += [cylz(recess_disc_x, s * recess_half_y, recess_disc_r, recess_z, 170).intersect(box(recess_x0, 30, -40, 40, recess_z, 170)) for s in SIDES]
rec.append(box(neg_notch_x0, 30, neg_notch_y0, neg_notch_y1, neg_notch_z, 170))
pocket = (cylz(pocket_a_x, pocket_a_y, pocket_a_r, pocket_floor_z, 170)
          .cut(cylz(pocket_b_x, pocket_b_y, pocket_b_r, pocket_floor_z - 1, 171))
          .cut(cyly(pocket_floor_x, pocket_floor_z, pocket_floor_r, 10, 30)))
rec.append(pocket)
model.add("yoke_clearances", group(rec), op="cut")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#99deef")
result = model.result()

