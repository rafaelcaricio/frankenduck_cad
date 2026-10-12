# battery_carrier_native: Frankenduck NP-F550 battery carrier under the trunk base (printed part).
# Native port of cad_blender/code_cad/battery_carrier/build.py stage B --rev r02 (canonical since
# 2026-09-28), build123d -> CadQuery. Robot world frame, mm, Z up. Same features, same order.
import math
import cadquery as cq
from source_api import Model

floor_x = -39.8  # mm battery seating floor (panel front face)
panel_t = 3.5  # mm panel thickness (X -39.8 .. -36.3)
panel_hy = 22.34  # mm panel lower half width
panel_top_hy = 27.0  # mm panel upper half width
panel_z0 = 74.0  # mm panel bottom
panel_z1 = 142.0  # mm panel shoulder start
panel_z2 = 147.0  # mm panel shoulder end
panel_z3 = 162.0  # mm panel top
panel_corner_r = 2.0  # mm fillet on the panel outline edges (along X)
panel_front_r = 0.45  # mm fillet on the seating floor face edges
rail_y0 = 19.84  # mm rail inner face |Y|
rail_y1 = 22.34  # mm rail outer face |Y|
rail_below = 5.0  # mm rails reach X floor-5 .. floor+0.2
rail_z0 = 76.0  # mm rail bottom
rail_z1 = 146.5  # mm rail top
rail_edge_r = 0.7  # mm rail outer long edge fillet
hook_z_lo = 83.0  # mm lower hook bottom
hook_z_hi = 120.0  # mm upper hook bottom
hook_y0 = 17.84  # mm hook inner |Y|
hook_x0 = -4.5  # mm hook X start relative to floor (r02: 2.0 mm thick)
hook_x1 = -2.5  # mm hook X end relative to floor
hook_h = 5.0  # mm hook height
hook_chamfer = 0.6  # mm hook bottom edge chamfer
ear_y0 = 25.3  # mm side ear inner |Y|
ear_y1 = 27.0  # mm side ear outer |Y|
ear_x0 = -8.4  # mm ear X start relative to floor
ear_x1 = 0.2  # mm ear X end relative to floor
ear_z0 = 142.0  # mm ear bottom
ear_z1 = 155.0  # mm ear top (stage B lowered)
ear_r = 1.3  # mm ear edge fillet
deck_x0 = -37.25  # mm deck X start
deck_x1 = -27.75  # mm deck X end (rounded part)
deck_hy = 31.25  # mm deck half width
deck_z0 = 152.35  # mm deck underside
deck_z1 = 155.45  # mm deck top (trunk_base.001 underside)
deck_corner_r = 3.0  # mm deck plan corner fillet
deck_ext_x1 = -27.5  # mm rear deck extension X end
deck_ext_hy = 19.25  # mm rear deck extension half width
gusset_y0 = 17.0  # mm deck gusset inner |Y|
gusset_y1 = 21.0  # mm deck gusset outer |Y|
seat_depth = 2.5  # mm connector seat pocket depth from the floor
seat_hy = 25.3  # mm connector seat half width
seat_z0 = 141.5  # mm connector seat floor
pilot_x = -31.7  # mm body fixing axes X
pilot_y_in = 18.0  # mm inner fixing axes |Y|
pilot_y_out = 25.0  # mm outer fixing axes |Y|
insert_pilot_d = 4.7  # mm M3x4x5 heat-set insert pilot diameter
insert_pilot_depth = 5.0  # mm pilot depth from the deck top
boss_depth = 6.0  # mm insert boss depth below the deck top
tip_d = 3.2  # mm screw tip bore diameter
boss_r = 4.5  # mm insert boss radius (2.5 + 2.0 wall)
boss_clip_x = -27.55  # mm insert boss flat (0.6 mm from the STS3215 case)
pin_x = -40.1  # mm donor pin relief axis X
pin_y = 17.25  # mm donor pin relief axis |Y|
pin_r = 1.8  # mm donor pin relief radius
pin_z0 = 136.7  # mm donor pin relief bottom
pin_len = 12.0  # mm donor pin relief length
nut_ax = -43.3  # mm cover screw axis X
nut_ay = 31.0  # mm cover screw axis |Y|
nboss_x0 = -48.0  # mm nut boss X start
nboss_x1 = -38.6  # mm nut boss X end
nboss_y0 = 26.5  # mm nut boss inner |Y|
nboss_y1 = 34.5  # mm nut boss outer |Y|
nboss_z0 = 147.5  # mm nut boss bottom
nboss_z1 = 155.0  # mm nut boss top
nboss_r = 1.5  # mm nut boss outer vertical edge fillet
nut_hole_d = 3.2  # mm M3 clearance hole
nut_af = 5.5  # mm M3 nut across flats
nut_h = 2.4  # mm M3 nut height
nut_clear = 0.3  # mm nut slot clearance
nut_roof_z = 153.0  # mm nut slot roof
trim_x0 = -39.9  # mm corner column trim X start
trim_x1 = -37.3  # mm corner column trim X end
trim_y0 = 25.3  # mm corner column trim inner |Y|
trim_y1 = 27.2  # mm corner column trim outer |Y|
trim_z0 = 155.0  # mm corner column trim bottom

F = floor_x
X, Y, Z = cq.Vector(1, 0, 0), cq.Vector(0, 1, 0), cq.Vector(0, 0, 1)


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

# back panel with filleted outline
outline = [(-panel_hy, panel_z0), (panel_hy, panel_z0), (panel_hy, panel_z1), (panel_top_hy, panel_z2), (panel_top_hy, panel_z3),
           (-panel_top_hy, panel_z3), (-panel_top_hy, panel_z2), (-panel_hy, panel_z1)]
panel = prism([(F, y, z) for y, z in outline], (panel_t, 0, 0))
panel = panel.fillet(panel_corner_r, edges(panel, lambda e: abs(L(e) - panel_t) < 1e-5))
panel = panel.fillet(panel_front_r, edges(panel, lambda e: abs(C(e).x - F) < 1e-5))
model.add("panel", panel, op="new", body="carrier")

# side rails, hooks, hook root gussets, 45 deg rail roots
for sign in (-1, 1):
    ya, yb = (rail_y0, rail_y1) if sign == 1 else (-rail_y1, -rail_y0)
    wall = box(F - rail_below, F + 0.2, ya, yb, rail_z0, rail_z1)
    wall = wall.fillet(rail_edge_r, edges(wall, lambda e: abs(C(e).x - (F - 5)) < 1e-5 and L(e) > 65))
    model.add(f"rail_{sign + 1}", wall, op="join", body="carrier")
    for k, z in enumerate((hook_z_lo, hook_z_hi)):
        ya, yb = (hook_y0, rail_y1) if sign == 1 else (-rail_y1, -hook_y0)
        lip = box(F + hook_x0, F + hook_x1, ya, yb, z, z + hook_h)
        lip = lip.chamfer(hook_chamfer, None, edges(lip, lambda e: abs(C(e).z - z) < 1e-5 and abs(L(e) - (yb - ya)) < 1e-5))
        model.add(f"hook_{sign + 1}_{k}", lip, op="join", body="carrier")
        ya = sign * rail_y0; yi = sign * (rail_y0 - 1.0)
        model.add(f"hook_gusset_{sign + 1}_{k}", prism([(F - 2.5, ya, z + .6), (F - 2.5, yi, z + .6), (F - 1.5, ya, z + .6)], (0, 0, 4.4)),
                  op="join", body="carrier")
        ya, yb = (rail_y0, rail_y1) if sign == 1 else (-rail_y1, -rail_y0)
        dx = hook_x0 + 5.0
        model.add(f"rail_root_{sign + 1}_{k}", prism_xz([(F - 5 + dx, z + 4.5), (F - 2 + dx, z + 7.5), (F + .15, z + 7.5), (F + .15, z + 4.5)], ya, yb),
                  op="join", body="carrier")

# rounded side ears locating the donor block
for i, (ya, yb) in enumerate([(-ear_y1, -ear_y0), (ear_y0, ear_y1)]):
    ear = box(F + ear_x0, F + ear_x1, ya, yb, ear_z0, ear_z1)
    ear = ear.fillet(ear_r, edges(ear, lambda e: abs(L(e) - (yb - ya)) < 1e-5))
    model.add(f"ear_{i}", ear, op="join", body="carrier")

# mounting deck under trunk_base.001
deck = box(deck_x0, deck_x1, -deck_hy, deck_hy, deck_z0, deck_z1)
deck = deck.fillet(deck_corner_r, edges(deck, lambda e: abs(L(e) - (deck_z1 - deck_z0)) < 1e-5))
model.add("deck", deck, op="join", body="carrier")
model.add("deck_ext", box(deck_x0, deck_ext_x1, -deck_ext_hy, deck_ext_hy, deck_z0, deck_z1), op="join", body="carrier")
for i, (ya, yb) in enumerate([(-gusset_y1, -gusset_y0), (gusset_y0, gusset_y1)]):
    model.add(f"deck_gusset_{i}", prism_xz([(F + 1.5, 141.5), (-35.8, 143), (-30, deck_z0), (-30, 153), (F + 1.5, 153)], ya, yb),
              op="join", body="carrier")

# reinforcement behind the recessed connector seat
model.add("seat_back_a", box(-36.5, -36.1, -seat_hy, seat_hy, 141.5, 162.), op="join", body="carrier")
model.add("seat_back_b", box(-37.5, -33.8, -seat_hy, seat_hy, 141.5, 154.8), op="join", body="carrier")
model.add("seat_back_c", prism_xz([(-37.5, 141.5), (-33.8, 141.5), (-33.8, 155.5), (-36.1, 157.8), (-36.1, 162), (-37.5, 162)], -11.5, 11.5),
          op="join", body="carrier")
model.add("seat_back_d", prism_xz([(-36.4, 138.9), (-33.8, 141.5), (-33.8, 142), (-36.4, 142)], -11.5, 11.5), op="join", body="carrier")

pilots = [(pilot_x, -pilot_y_out), (pilot_x, -pilot_y_in), (pilot_x, pilot_y_in), (pilot_x, pilot_y_out)]
# M3 insert bosses under the deck, flat-cut toward the servo case
for i, (x, y) in enumerate(pilots):
    boss = cylz(x, y, boss_r, deck_z1 - boss_depth, deck_z1)
    boss = boss.intersect(box(x - boss_r - 1, boss_clip_x, y - boss_r - 1, y + boss_r + 1, deck_z1 - boss_depth - 1, deck_z1 + 1))
    model.add(f"insert_boss_{i}", boss, op="join", body="carrier")

# cover-plate nut bosses beside the ears
for s in (-1, 1):
    nb = box(nboss_x0, nboss_x1, min(s * nboss_y0, s * nboss_y1), max(s * nboss_y0, s * nboss_y1), nboss_z0, nboss_z1)
    nb = nb.fillet(nboss_r, edges(nb, lambda e: abs(L(e) - (nboss_z1 - nboss_z0)) < 1e-5 and abs(C(e).y - s * nboss_y1) < 1e-5))
    model.add(f"nut_boss_{s + 1}", nb, op="join", body="carrier")

# corner columns beside the seat end at Z 155.0
for s in (-1, 1):
    model.add(f"column_trim_{s + 1}", box(trim_x0, trim_x1, min(s * trim_y0, s * trim_y1), max(s * trim_y0, s * trim_y1), trim_z0, 163.0),
              op="cut", body="carrier")

# open-top connector seat pocket
model.add("seat_pocket", box(F, F + seat_depth, -seat_hy, seat_hy, seat_z0, 163.0), op="cut", body="carrier")

# side bridges rail top -> ear
for sign in (-1, 1):
    yz = [(sign * 22.1, 139.5), (sign * 25.7, 143.1), (sign * 25.7, 145.5), (sign * 22.1, 145.5)]
    model.add(f"bridge_{sign + 1}", prism([(F - 5, y, z) for y, z in yz], (5.2, 0, 0)), op="join", body="carrier")

# donor pin reliefs
for i, y in enumerate((-pin_y, pin_y)):
    model.add(f"pin_relief_{i}", cylz(pin_x, y, pin_r, pin_z0, pin_z0 + pin_len), op="cut", body="carrier")

# M3x4x5 insert pilots and screw tip bores
for i, (x, y) in enumerate(pilots):
    model.add(f"insert_pilot_{i}", cylz(x, y, insert_pilot_d / 2, deck_z1 - insert_pilot_depth, deck_z1 + 0.5), op="cut", body="carrier")
    model.add(f"tip_bore_{i}", cylz(x, y, tip_d / 2, deck_z1 - boss_depth - 0.5, deck_z1 - insert_pilot_depth + 0.01), op="cut", body="carrier")

# cover-plate screw holes and side-loading captured-nut slots
w = nut_af + nut_clear
hz = nut_h + nut_clear
rc = w / math.sqrt(3.0)
z0 = nut_roof_z - hz
for s in (-1, 1):
    y = s * nut_ay
    model.add(f"cover_hole_{s + 1}", cylz(nut_ax, y, nut_hole_d / 2, nboss_z0 - 0.5, nboss_z1 + 0.5), op="cut", body="carrier")
    hexpts = [(nut_ax + rc * math.sin(math.radians(a)), y + rc * math.cos(math.radians(a)), z0) for a in range(0, 360, 60)]
    model.add(f"nut_hex_{s + 1}", prism(hexpts, (0, 0, hz)), op="cut", body="carrier")
    ye = s * (nboss_y1 + 1.0)
    model.add(f"nut_lead_{s + 1}", box(nut_ax - w / 2, nut_ax + w / 2, min(y, ye), max(y, ye), z0, nut_roof_z), op="cut", body="carrier")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#8a8a90")
result = model.result()

