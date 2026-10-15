from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    import cadquery as cq
    from lib.cadquery_adapter import Model
    plate_x0 = -35.0
    plate_x1 = 24.0
    plate_half_y = 37.0
    plate_z0 = 158.45
    plate_z1 = 164.45
    plate_corner_r = 6.0
    front_strip_x0 = 5.0
    front_half_y = 34.5
    pad_x0 = -35.7
    pad_x1 = -27.7
    pad_y0 = 12.0
    pad_y1 = 29.0
    pad_z0 = 155.45
    pad_z1 = 159.0
    pad_corner_r = 2.0
    land_x_a = -24.65
    land_x_b = -0.2
    land_y_a = 13.375
    land_y_b = 33.875
    land_boss_r = 2.75
    land_boss_z0 = 155.45
    land_boss_z1 = 158.25
    land_collar_r = 2.85
    land_collar_z1 = 162.45
    land_bore_r = 1.1
    land_well_r = 2.7
    land_seat_z = 162.45
    window_x0 = -17.0
    window_x1 = 12.0
    window_half_y = 13.0
    window_r = 5.0
    slot_x0 = -10.9
    slot_x1 = -2.65
    slot_y0 = 13.125
    slot_y1 = 34.125
    slot_r = 2.5
    big_hole_x = 8.1
    big_hole_y = 23.625
    big_hole_r = 5.0
    yoke_x_a = -16.0
    yoke_x_b = 18.0
    yoke_y = 28.0
    yoke_pilot_r = 2.35
    yoke_floor_z = 159.95
    pilot_pad_r = 3.35
    pilot_pad_z0 = 158.25
    batt_x = -31.7
    batt_y_a = 18.0
    batt_y_b = 25.0
    batt_hole_r = 1.6
    batt_cb_r = 2.6
    shell_pilot_x_a = -20.0
    shell_pilot_x_b = 10.0
    shell_pilot_z = 162.0
    shell_pilot_r = 0.8
    shell_pilot_y0 = 31.0
    lug_x = 10.0
    lug_z = 162.0
    lug_r = 3.0
    lug_y1 = 37.0
    lug_end_fillet = 0.25
    well_tab_r = 4.4
    well_tab_y0 = 36.0
    chamfer_y0 = -20.6
    chamfer_y1 = 17.6
    chamfer_top_x = 6.137
    chamfer_bottom_x = 14.3908
    recess_z = 164.25
    recess_x0 = 20.75
    recess_half_y = 27.0
    recess_disc_x = 18.0
    recess_disc_r = 7.0
    neg_notch_x0 = 23.6
    neg_notch_y0 = -25.7
    neg_notch_y1 = -20.6
    neg_notch_z = 160.369
    pocket_a_x = 24.0898
    pocket_a_y = 21.1209
    pocket_a_r = 3.511
    pocket_b_x = 24.435
    pocket_b_y = 23.7614
    pocket_b_r = 2.1914
    pocket_floor_x = 20.4429
    pocket_floor_z = 160.995
    pocket_floor_r = 3.8283
    SIDES = (1, -1)
    LANDS = [(x, s * y) for x in (land_x_a, land_x_b) for y in (land_y_a, land_y_b) for s in SIDES]
    
    def box(x0, x1, y0, y1, z0, z1):
        return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))
    
    def cylz(x, y, r, z0, z1):
        return cq.Solid.makeCylinder(r, z1 - z0, cq.Vector(x, y, z0), cq.Vector(0, 0, 1))
    
    def cyly(x, z, r, y0, y1):
        return cq.Solid.makeCylinder(r, y1 - y0, cq.Vector(x, y0, z), cq.Vector(0, 1, 0))
    
    def rrect(x0, x1, y0, y1, r, z0, z1):
        return cq.Workplane('XY', origin=(0, 0, z0)).center((x0 + x1) / 2, (y0 + y1) / 2).rect(x1 - x0, y1 - y0).extrude(z1 - z0).edges('|Z').fillet(r).val()
    
    def group(solids):
        out = solids[0]
        for s in solids[1:]:
            out = out.fuse(s)
        return out.clean()
    model = Model()
    model.add('plate', rrect(plate_x0, plate_x1, -plate_half_y, plate_half_y, plate_corner_r, plate_z0, plate_z1))
    model.add('front_strips', group([box(front_strip_x0, plate_x1 + 1, s * front_half_y if s > 0 else -plate_half_y - 1, plate_half_y + 1 if s > 0 else -front_half_y, plate_z0 - 1, plate_z1 + 1) for s in SIDES]), op='cut')
    model.add('battery_pads', group([rrect(pad_x0, pad_x1, *sorted((s * pad_y0, s * pad_y1)), pad_corner_r, pad_z0, pad_z1) for s in SIDES]), op='join')
    openings = [rrect(window_x0, window_x1, -window_half_y, window_half_y, window_r, 150, 170)]
    for s in SIDES:
        openings.append(rrect(slot_x0, slot_x1, *sorted((s * slot_y0, s * slot_y1)), slot_r, 150, 170))
        openings.append(cylz(big_hole_x, s * big_hole_y, big_hole_r, 150, 170))
    model.add('openings', group(openings), op='cut')
    model.add('land_bosses', group([cylz(x, y, land_boss_r, land_boss_z0, land_boss_z1) for x, y in LANDS] + [cylz(x, y, land_collar_r, land_boss_z1, land_collar_z1) for x, y in LANDS]), op='join')
    lugs = []
    for s in SIDES:
        if s > 0:
            wp = cq.Workplane(cq.Plane(origin=(lug_x, lug_y1, lug_z), xDir=(1, 0, 0), normal=(0, -1, 0)))
            lug = wp.circle(lug_r).extrude(lug_y1 - front_half_y + 0.5).faces('>Y').edges().fillet(lug_end_fillet).val()
        else:
            wp = cq.Workplane(cq.Plane(origin=(lug_x, -lug_y1, lug_z), xDir=(1, 0, 0), normal=(0, 1, 0)))
            lug = wp.circle(lug_r).extrude(lug_y1 - front_half_y + 0.5).faces('<Y').edges().fillet(lug_end_fillet).val()
        lugs.append(lug.intersect(box(lug_x - 5, lug_x + 5, -40, 40, 150, plate_z1)))
    model.add('front_lugs', group(lugs), op='join')
    model.add('pilot_pads', group([cylz(x, s * yoke_y, pilot_pad_r, pilot_pad_z0, plate_z0 + 0.5) for x in (yoke_x_a, yoke_x_b) for s in SIDES]), op='join')
    model.add('well_tabs', group([cylz(land_x_b, s * land_y_b, well_tab_r, plate_z0, plate_z1).intersect(box(land_x_b - 5, land_x_b + 5, *sorted((s * well_tab_y0, s * (land_y_b + well_tab_r + 1))), plate_z0 - 1, plate_z1 + 1)) for s in SIDES]), op='join')
    model.add('land_holes', group([cylz(x, y, land_bore_r, 150, land_seat_z + 0.5) for x, y in LANDS] + [cylz(x, y, land_well_r, land_seat_z, 170) for x, y in LANDS]), op='cut')
    model.add('yoke_pilots', group([cylz(x, s * yoke_y, yoke_pilot_r, yoke_floor_z, 170) for x in (yoke_x_a, yoke_x_b) for s in SIDES]), op='cut')
    model.add('battery_holes', group([cylz(batt_x, s * y, batt_hole_r, 150, 170) for y in (batt_y_a, batt_y_b) for s in SIDES] + [cylz(batt_x, s * y, batt_cb_r, plate_z0, 170) for y in (batt_y_a, batt_y_b) for s in SIDES]), op='cut')
    model.add('shell_pilots', group([cyly(x, shell_pilot_z, shell_pilot_r, shell_pilot_y0, lug_y1 + 1) for x in (shell_pilot_x_a, shell_pilot_x_b)] + [cyly(x, shell_pilot_z, shell_pilot_r, -lug_y1 - 1, -shell_pilot_y0) for x in (shell_pilot_x_a, shell_pilot_x_b)]), op='cut')
    k = (chamfer_bottom_x - chamfer_top_x) / (plate_z1 - plate_z0)
    xz = lambda z: chamfer_top_x + (plate_z1 - z) * k
    zlo, zhi = (plate_z0 - 2, plate_z1 + 2)
    prof = cq.Wire.makePolygon([cq.Vector(xz(zhi), chamfer_y0, zhi), cq.Vector(xz(zlo), chamfer_y0, zlo), cq.Vector(40, chamfer_y0, zlo), cq.Vector(40, chamfer_y0, zhi)], close=True)
    model.add('front_notch', cq.Solid.extrudeLinear(cq.Face.makeFromWires(prof), cq.Vector(0, chamfer_y1 - chamfer_y0, 0)), op='cut')
    rec = [box(recess_x0, 30, -recess_half_y, recess_half_y, recess_z, 170)]
    rec += [cylz(recess_disc_x, s * recess_half_y, recess_disc_r, recess_z, 170).intersect(box(recess_x0, 30, -40, 40, recess_z, 170)) for s in SIDES]
    rec.append(box(neg_notch_x0, 30, neg_notch_y0, neg_notch_y1, neg_notch_z, 170))
    pocket = cylz(pocket_a_x, pocket_a_y, pocket_a_r, pocket_floor_z, 170).cut(cylz(pocket_b_x, pocket_b_y, pocket_b_r, pocket_floor_z - 1, 171)).cut(cyly(pocket_floor_x, pocket_floor_z, pocket_floor_r, 10, 30))
    rec.append(pocket)
    model.add('yoke_clearances', group(rec), op='cut')
    model.material('pla', color='#99deef')
    result = model.result()
    from lib.stick_pod import integrate_trunk
    return integrate_trunk(to_build123d(model, 'trunk_base_native'))


@step(out="../../STEP/parts/trunk_base_native.step")
def trunk_base_native():
    return make()


if __name__ == "__main__":
    trunk_base_native()
