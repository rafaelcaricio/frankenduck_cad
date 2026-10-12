# power_switch_kcd1_native: KCD1 round rocker switch (purchased), envelope model.
# Robot world frame, mm, Z up; switch axis along Y, bezel face outward at Y -44.6.
from source_api import Model

axis_x = -36.0  # mm switch axis X
axis_z = 139.0  # mm switch axis Z
bezel_y = -44.6  # mm bezel outer face Y
bezel_d = 16.5  # mm bezel diameter
bezel_t = 1.6  # mm bezel thickness
body_d = 15.0  # mm body (panel cut-out) diameter
body_y1 = -30.6  # mm body rear face Y
term_w = 10.0  # mm terminal block width (X)
term_h = 3.6  # mm terminal block height (Z)
term_y1 = -23.4  # mm terminal block rear face Y

model = Model()
model.cylinder("bezel", radius=bezel_d / 2, height=bezel_t, at=(axis_x, bezel_y, axis_z), direction=(0, 1, 0), op="new", body="switch")
model.cylinder("body", radius=body_d / 2, height=body_y1 - bezel_y - bezel_t, at=(axis_x, bezel_y + bezel_t, axis_z), direction=(0, 1, 0), op="join", body="switch")
model.box("terminals", length=term_w, width=term_y1 - body_y1, height=term_h, at=(axis_x, (body_y1 + term_y1) / 2, axis_z), op="join", body="switch")
result = model.result()

