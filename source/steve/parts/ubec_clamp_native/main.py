"""ubec_clamp_native: Frankenduck head, UBEC clamp (R02), robot world frame, Z up, mm.

Native port of ubec_clamp() in cad_blender/code_cad/head_internals/code/build_parts_r05.py.
Screwed clamp that holds the HENGE UBEC down on the two bottom-shell bosses: a rounded bar, a top
stiffening rib, two round pads over the shell bosses, each with an M2 clearance hole and a pan-head well.
No M3 insert pockets in this part; the M2 holes are clearances (the self-forming M2 holes are in the shell).
Printed upright: underside (z 282.9) on the bed.
"""
from steve import Model

# ---------------------------------------------------------------- bar
clamp_z0 = 282.9  # mm Clamp underside (shell boss tops)
clamp_z1 = 285.9  # mm Clamp top face
bar_x0 = 65.0  # mm Bar rear end x
bar_x1 = 99.0  # mm Bar front end x
bar_y0 = -28.5  # mm Bar outer edge y
bar_y1 = -23.8  # mm Bar inner edge y
bar_r = 2.0  # mm Bar plan corner radius
# ---------------------------------------------------------------- rib
rib_x0 = 71.5  # mm Rib start x
rib_x1 = 75.5  # mm Rib end x
rib_y0 = -27.2  # mm Rib outer edge y
rib_y1 = -25.2  # mm Rib inner edge y
rib_z1 = 287.5  # mm Rib top
# ---------------------------------------------------------------- screw pads (over the shell bosses)
boss_y = -25.0  # mm Shell boss centre y
boss1_x = 68.5  # mm Rear shell boss centre x
boss2_x = 95.5  # mm Front shell boss centre x
pad_r = 4.0  # mm Screw pad radius
m2_clear_d = 2.4  # mm M2 clearance hole diameter
well_r = 2.2  # mm Pan-head well radius
grip = 2.0  # mm Material under the screw head (well floor above the underside)

model = Model()

model.plane("plane_base", on="XY", offset=clamp_z0, name="Clamp underside")
with model.sketch("sketch_bar", on="plane_base", name="Bar outline") as s:
    s.line("bar_b", (bar_x0 + bar_r, bar_y0), (bar_x1 - bar_r, bar_y0))
    s.arc("bar_br", (bar_x1 - bar_r, bar_y0 + bar_r), bar_r, 270, 360)
    s.line("bar_rt", (bar_x1, bar_y0 + bar_r), (bar_x1, bar_y1 - bar_r))
    s.arc("bar_tr", (bar_x1 - bar_r, bar_y1 - bar_r), bar_r, 0, 90)
    s.line("bar_t", (bar_x1 - bar_r, bar_y1), (bar_x0 + bar_r, bar_y1))
    s.arc("bar_tl", (bar_x0 + bar_r, bar_y1 - bar_r), bar_r, 90, 180)
    s.line("bar_lt", (bar_x0, bar_y1 - bar_r), (bar_x0, bar_y0 + bar_r))
    s.arc("bar_bl", (bar_x0 + bar_r, bar_y0 + bar_r), bar_r, 180, 270)
model.extrude("bar", "sketch_bar", distance=clamp_z1 - clamp_z0, name="Clamp bar")
model.box("rib", rib_x1 - rib_x0, rib_y1 - rib_y0, rib_z1 - clamp_z1,
          at=((rib_x0 + rib_x1) / 2, (rib_y0 + rib_y1) / 2, (clamp_z1 + rib_z1) / 2), op="join", name="Stiffening rib")
model.cylinder("pad_rear", pad_r, clamp_z1 - clamp_z0, at=(boss1_x, boss_y, clamp_z0), op="join", name="Rear screw pad")
model.cylinder("pad_front", pad_r, clamp_z1 - clamp_z0, at=(boss2_x, boss_y, clamp_z0), op="join", name="Front screw pad")
model.cylinder("clear_rear", m2_clear_d / 2, clamp_z1 - clamp_z0 + 3.0, at=(boss1_x, boss_y, clamp_z0 - 1.0), op="cut",
               name="Rear M2 clearance")
model.cylinder("clear_front", m2_clear_d / 2, clamp_z1 - clamp_z0 + 3.0, at=(boss2_x, boss_y, clamp_z0 - 1.0), op="cut",
               name="Front M2 clearance")
model.cylinder("well_rear", well_r, clamp_z1 - clamp_z0 - grip + 2.0, at=(boss1_x, boss_y, clamp_z0 + grip), op="cut",
               name="Rear pan-head well")
model.cylinder("well_front", well_r, clamp_z1 - clamp_z0 - grip + 2.0, at=(boss2_x, boss_y, clamp_z0 + grip), op="cut",
               name="Front pan-head well")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#8a8a90")
result = model.result()

