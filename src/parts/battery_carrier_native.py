from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    import math
    import cadquery as cq
    from lib.steve_compat import Model
    floor_x = -39.8
    panel_t = 3.5
    panel_hy = 22.34
    panel_top_hy = 27.0
    panel_z0 = 74.0
    panel_z1 = 142.0
    panel_z2 = 147.0
    panel_z3 = 162.0
    panel_corner_r = 2.0
    panel_front_r = 0.45
    rail_y0 = 19.84
    rail_y1 = 22.34
    rail_below = 5.0
    rail_z0 = 76.0
    rail_z1 = 146.5
    rail_edge_r = 0.7
    hook_z_lo = 83.0
    hook_z_hi = 120.0
    hook_y0 = 17.84
    hook_x0 = -4.5
    hook_x1 = -2.5
    hook_h = 5.0
    hook_chamfer = 0.6
    ear_y0 = 25.3
    ear_y1 = 27.0
    ear_x0 = -8.4
    ear_x1 = 0.2
    ear_z0 = 142.0
    ear_z1 = 155.0
    ear_r = 1.3
    deck_x0 = -37.25
    deck_x1 = -27.75
    deck_hy = 31.25
    deck_z0 = 152.35
    deck_z1 = 155.45
    deck_corner_r = 3.0
    deck_ext_x1 = -27.5
    deck_ext_hy = 19.25
    gusset_y0 = 17.0
    gusset_y1 = 21.0
    seat_depth = 2.5
    seat_hy = 25.3
    seat_z0 = 141.5
    pilot_x = -31.7
    pilot_y_in = 18.0
    pilot_y_out = 25.0
    insert_pilot_d = 4.7
    insert_pilot_depth = 5.0
    boss_depth = 6.0
    tip_d = 3.2
    boss_r = 4.5
    boss_clip_x = -27.55
    pin_x = -40.1
    pin_y = 17.25
    pin_r = 1.8
    pin_z0 = 136.7
    pin_len = 12.0
    nut_ax = -43.3
    nut_ay = 31.0
    nboss_x0 = -48.0
    nboss_x1 = -38.6
    nboss_y0 = 26.5
    nboss_y1 = 34.5
    nboss_z0 = 147.5
    nboss_z1 = 155.0
    nboss_r = 1.5
    nut_hole_d = 3.2
    nut_af = 5.5
    nut_h = 2.4
    nut_clear = 0.3
    nut_roof_z = 153.0
    trim_x0 = -39.9
    trim_x1 = -37.3
    trim_y0 = 25.3
    trim_y1 = 27.2
    trim_z0 = 155.0
    F = floor_x
    X, Y, Z = (cq.Vector(1, 0, 0), cq.Vector(0, 1, 0), cq.Vector(0, 0, 1))
    
    def box(x0, x1, y0, y1, z0, z1):
        return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))
    
    def cylz(x, y, r, z0, z1):
        return cq.Solid.makeCylinder(r, z1 - z0, cq.Vector(x, y, z0), Z)
    
    def prism(pts3, vec):
        return cq.Solid.extrudeLinear(cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(*p) for p in pts3], close=True)), cq.Vector(*vec))
    
    def prism_xz(pts, y0, y1):
        return prism([(x, y0, z) for x, z in pts], (0, y1 - y0, 0))
    
    def edges(s, pred):
        return [e for e in s.Edges() if pred(e)]
    
    def L(e):
        return e.Length()
    
    def C(e):
        return e.Center()
    model = Model()
    outline = [(-panel_hy, panel_z0), (panel_hy, panel_z0), (panel_hy, panel_z1), (panel_top_hy, panel_z2), (panel_top_hy, panel_z3), (-panel_top_hy, panel_z3), (-panel_top_hy, panel_z2), (-panel_hy, panel_z1)]
    panel = prism([(F, y, z) for y, z in outline], (panel_t, 0, 0))
    panel = panel.fillet(panel_corner_r, edges(panel, lambda e: abs(L(e) - panel_t) < 1e-05))
    panel = panel.fillet(panel_front_r, edges(panel, lambda e: abs(C(e).x - F) < 1e-05))
    model.add('panel', panel, op='new', body='carrier')
    for sign in (-1, 1):
        ya, yb = (rail_y0, rail_y1) if sign == 1 else (-rail_y1, -rail_y0)
        wall = box(F - rail_below, F + 0.2, ya, yb, rail_z0, rail_z1)
        wall = wall.fillet(rail_edge_r, edges(wall, lambda e: abs(C(e).x - (F - 5)) < 1e-05 and L(e) > 65))
        model.add(f'rail_{sign + 1}', wall, op='join', body='carrier')
        for k, z in enumerate((hook_z_lo, hook_z_hi)):
            ya, yb = (hook_y0, rail_y1) if sign == 1 else (-rail_y1, -hook_y0)
            lip = box(F + hook_x0, F + hook_x1, ya, yb, z, z + hook_h)
            lip = lip.chamfer(hook_chamfer, None, edges(lip, lambda e: abs(C(e).z - z) < 1e-05 and abs(L(e) - (yb - ya)) < 1e-05))
            model.add(f'hook_{sign + 1}_{k}', lip, op='join', body='carrier')
            ya = sign * rail_y0
            yi = sign * (rail_y0 - 1.0)
            model.add(f'hook_gusset_{sign + 1}_{k}', prism([(F - 2.5, ya, z + 0.6), (F - 2.5, yi, z + 0.6), (F - 1.5, ya, z + 0.6)], (0, 0, 4.4)), op='join', body='carrier')
            ya, yb = (rail_y0, rail_y1) if sign == 1 else (-rail_y1, -rail_y0)
            dx = hook_x0 + 5.0
            model.add(f'rail_root_{sign + 1}_{k}', prism_xz([(F - 5 + dx, z + 4.5), (F - 2 + dx, z + 7.5), (F + 0.15, z + 7.5), (F + 0.15, z + 4.5)], ya, yb), op='join', body='carrier')
    for i, (ya, yb) in enumerate([(-ear_y1, -ear_y0), (ear_y0, ear_y1)]):
        ear = box(F + ear_x0, F + ear_x1, ya, yb, ear_z0, ear_z1)
        ear = ear.fillet(ear_r, edges(ear, lambda e: abs(L(e) - (yb - ya)) < 1e-05))
        model.add(f'ear_{i}', ear, op='join', body='carrier')
    deck = box(deck_x0, deck_x1, -deck_hy, deck_hy, deck_z0, deck_z1)
    deck = deck.fillet(deck_corner_r, edges(deck, lambda e: abs(L(e) - (deck_z1 - deck_z0)) < 1e-05))
    model.add('deck', deck, op='join', body='carrier')
    model.add('deck_ext', box(deck_x0, deck_ext_x1, -deck_ext_hy, deck_ext_hy, deck_z0, deck_z1), op='join', body='carrier')
    for i, (ya, yb) in enumerate([(-gusset_y1, -gusset_y0), (gusset_y0, gusset_y1)]):
        model.add(f'deck_gusset_{i}', prism_xz([(F + 1.5, 141.5), (-35.8, 143), (-30, deck_z0), (-30, 153), (F + 1.5, 153)], ya, yb), op='join', body='carrier')
    model.add('seat_back_a', box(-36.5, -36.1, -seat_hy, seat_hy, 141.5, 162.0), op='join', body='carrier')
    model.add('seat_back_b', box(-37.5, -33.8, -seat_hy, seat_hy, 141.5, 154.8), op='join', body='carrier')
    model.add('seat_back_c', prism_xz([(-37.5, 141.5), (-33.8, 141.5), (-33.8, 155.5), (-36.1, 157.8), (-36.1, 162), (-37.5, 162)], -11.5, 11.5), op='join', body='carrier')
    model.add('seat_back_d', prism_xz([(-36.4, 138.9), (-33.8, 141.5), (-33.8, 142), (-36.4, 142)], -11.5, 11.5), op='join', body='carrier')
    pilots = [(pilot_x, -pilot_y_out), (pilot_x, -pilot_y_in), (pilot_x, pilot_y_in), (pilot_x, pilot_y_out)]
    for i, (x, y) in enumerate(pilots):
        boss = cylz(x, y, boss_r, deck_z1 - boss_depth, deck_z1)
        boss = boss.intersect(box(x - boss_r - 1, boss_clip_x, y - boss_r - 1, y + boss_r + 1, deck_z1 - boss_depth - 1, deck_z1 + 1))
        model.add(f'insert_boss_{i}', boss, op='join', body='carrier')
    for s in (-1, 1):
        nb = box(nboss_x0, nboss_x1, min(s * nboss_y0, s * nboss_y1), max(s * nboss_y0, s * nboss_y1), nboss_z0, nboss_z1)
        nb = nb.fillet(nboss_r, edges(nb, lambda e: abs(L(e) - (nboss_z1 - nboss_z0)) < 1e-05 and abs(C(e).y - s * nboss_y1) < 1e-05))
        model.add(f'nut_boss_{s + 1}', nb, op='join', body='carrier')
    for s in (-1, 1):
        model.add(f'column_trim_{s + 1}', box(trim_x0, trim_x1, min(s * trim_y0, s * trim_y1), max(s * trim_y0, s * trim_y1), trim_z0, 163.0), op='cut', body='carrier')
    model.add('seat_pocket', box(F, F + seat_depth, -seat_hy, seat_hy, seat_z0, 163.0), op='cut', body='carrier')
    for sign in (-1, 1):
        yz = [(sign * 22.1, 139.5), (sign * 25.7, 143.1), (sign * 25.7, 145.5), (sign * 22.1, 145.5)]
        model.add(f'bridge_{sign + 1}', prism([(F - 5, y, z) for y, z in yz], (5.2, 0, 0)), op='join', body='carrier')
    for i, y in enumerate((-pin_y, pin_y)):
        model.add(f'pin_relief_{i}', cylz(pin_x, y, pin_r, pin_z0, pin_z0 + pin_len), op='cut', body='carrier')
    for i, (x, y) in enumerate(pilots):
        model.add(f'insert_pilot_{i}', cylz(x, y, insert_pilot_d / 2, deck_z1 - insert_pilot_depth, deck_z1 + 0.5), op='cut', body='carrier')
        model.add(f'tip_bore_{i}', cylz(x, y, tip_d / 2, deck_z1 - boss_depth - 0.5, deck_z1 - insert_pilot_depth + 0.01), op='cut', body='carrier')
    w = nut_af + nut_clear
    hz = nut_h + nut_clear
    rc = w / math.sqrt(3.0)
    z0 = nut_roof_z - hz
    for s in (-1, 1):
        y = s * nut_ay
        model.add(f'cover_hole_{s + 1}', cylz(nut_ax, y, nut_hole_d / 2, nboss_z0 - 0.5, nboss_z1 + 0.5), op='cut', body='carrier')
        hexpts = [(nut_ax + rc * math.sin(math.radians(a)), y + rc * math.cos(math.radians(a)), z0) for a in range(0, 360, 60)]
        model.add(f'nut_hex_{s + 1}', prism(hexpts, (0, 0, hz)), op='cut', body='carrier')
        ye = s * (nboss_y1 + 1.0)
        model.add(f'nut_lead_{s + 1}', box(nut_ax - w / 2, nut_ax + w / 2, min(y, ye), max(y, ye), z0, nut_roof_z), op='cut', body='carrier')
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'battery_carrier_native')


@step(out="../../STEP/parts/battery_carrier_native.step")
def battery_carrier_native():
    return make()


if __name__ == "__main__":
    battery_carrier_native()
