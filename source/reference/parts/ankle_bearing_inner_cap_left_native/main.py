# ankle_bearing_inner_cap_<side>_native: printed cap on the inner race of the ankle F688 flanged bearing (Frankenduck).
# Native rebuild of mesh ankle_bearing_inner_cap_<side>, robot world frame, mm, Z up. Left = +Y, right = -Y (exact mirror about Y = 0).
# Ø9.5 disk 1 mm thick on the bearing axis (along Y) with two Ø1.8 holes.
from source_api import Model

SIDE = 1  # side switch: 1 = left (+Y), -1 = right (-Y); the only line that differs between the two documents

axis_x = -42.899  # mm Bearing axis X
axis_z = 3.975  # mm Bearing axis Z
y_inner = 39.605  # mm |Y| of the face against the bearing inner race
thickness = 1.0  # mm Cap thickness along Y
cap_r = 4.75  # mm Cap radius (Ø9.5, inside the F688 inner race shoulder)
hole_offset = 1.8  # mm Hole offset from the axis along X
hole_r = 0.9  # mm Hole radius (Ø1.8)

model = Model()
model.cylinder("cap", radius=cap_r, height=thickness, at=(axis_x, SIDE * y_inner, axis_z), direction=(0, SIDE, 0))
for n, dx in (("a", hole_offset), ("b", -hole_offset)):
    model.cylinder(f"hole_{n}", radius=hole_r, height=thickness + 2,
                   at=(axis_x + dx, SIDE * (y_inner - 1), axis_z), direction=(0, SIDE, 0), op="cut")

# Aluminium (user, 2026-10-10; alloy not stated, 6061-T6 assumed). The Frankenduck design's
# simulation export overrides its mass with design.mass (mass_record.py), so this sets appearance
# and the part's own reported mass only.
model.material("aluminum-6061-t6", color="#f96c0a")
result = model.result()

