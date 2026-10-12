import math
from source_api import Model

axis_y = -0.12336  # mm Ring axis Y position (robot world frame)
axis_z = 301.50477  # mm Ring axis Z position (robot world frame)
ring_x0 = 109.89  # mm Ring back face X (tab side)
ring_len = 10.8  # mm Ring length along +X
outer_r = 20.25  # mm Ring outer radius
bore_r = 9.72  # mm Bore radius
outer_chamfer = 0.675  # mm 45 deg chamfer on the outer front edge
bore_chamfer = 2.7  # mm 45 deg chamfer on the bore front edge
tab_ri = 11.475  # mm Tab inner radius
tab_ro = 12.825  # mm Tab outer radius
tab_h = 2.025  # mm Tab height, protruding along -X from the back face
tab_w = 20.0  # deg Tab angular width
tab_a0 = 90.0  # deg First tab centre angle, from +Y toward +Z
tab_count = 3  # count Number of tabs, equally spaced

model = Model()

model.plane("plane_axis", on="XY", offset=axis_z, name="Axis plane")
model.axis("axis_ring", through=(0.0, axis_y, axis_z), direction=(1, 0, 0), name="Ring axis")

with model.sketch("sketch_ring", on="plane_axis", name="Ring profile") as s:
    s.line("l1", (ring_x0, axis_y + bore_r), (ring_x0 + ring_len - bore_chamfer, axis_y + bore_r))
    s.line("l2", (ring_x0 + ring_len - bore_chamfer, axis_y + bore_r), (ring_x0 + ring_len, axis_y + bore_r + bore_chamfer))
    s.line("l3", (ring_x0 + ring_len, axis_y + bore_r + bore_chamfer), (ring_x0 + ring_len, axis_y + outer_r - outer_chamfer))
    s.line("l4", (ring_x0 + ring_len, axis_y + outer_r - outer_chamfer), (ring_x0 + ring_len - outer_chamfer, axis_y + outer_r))
    s.line("l5", (ring_x0 + ring_len - outer_chamfer, axis_y + outer_r), (ring_x0, axis_y + outer_r))
    s.line("l6", (ring_x0, axis_y + outer_r), (ring_x0, axis_y + bore_r))

model.revolve("revolve_ring", "sketch_ring", axis="axis_ring", name="Ring")

model.plane("plane_tab", on="YZ", offset=ring_x0 - tab_h, name="Tab plane")

with model.sketch("sketch_tab", on="plane_tab", name="Tab profile") as s:
    s.arc("a_out", (axis_y, axis_z), tab_ro, tab_a0 - tab_w / 2, tab_a0 + tab_w / 2)
    s.arc("a_in", (axis_y, axis_z), tab_ri, tab_a0 - tab_w / 2, tab_a0 + tab_w / 2)
    s.line("side_1", (axis_y + tab_ri * math.cos(math.radians(tab_a0 - tab_w / 2)), axis_z + tab_ri * math.sin(math.radians(tab_a0 - tab_w / 2))), (axis_y + tab_ro * math.cos(math.radians(tab_a0 - tab_w / 2)), axis_z + tab_ro * math.sin(math.radians(tab_a0 - tab_w / 2))))
    s.line("side_2", (axis_y + tab_ri * math.cos(math.radians(tab_a0 + tab_w / 2)), axis_z + tab_ri * math.sin(math.radians(tab_a0 + tab_w / 2))), (axis_y + tab_ro * math.cos(math.radians(tab_a0 + tab_w / 2)), axis_z + tab_ro * math.sin(math.radians(tab_a0 + tab_w / 2))))

model.extrude("extrude_tab", "sketch_tab", distance=tab_h, op="join", name="Tab")
model.circular_pattern("pattern_tabs", axis="axis_ring", count=tab_count, features=["extrude_tab"], name="Tab pattern")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#f79507")
result = model.result()

