# imu_yoke_bridge_native: removable printed bracket that hangs the trunk MPU-6050 board under the neck_internal_yoke (Frankenduck).
# Native rebuild of imu_mpu6050_removable_yoke_bridge (Blender recipe cad_blender/candidates/trunk_mpu6050/build_r02.py),
# robot world frame, mm, Z up. A bar clamped on the yoke seat plane Z 168.45 by two countersunk M3 screws, a link and a
# vertical plate carrying four M2 board bosses with hex nut pockets on the board side (+X).
import math
import cadquery as cq
from source_api import Model

bar_x0 = -20.0  # mm Clamp bar rear X
bar_x1 = -12.0  # mm Clamp bar front X
bar_half_y = 31.5  # mm Clamp bar half length
z_seat = 168.45  # mm Underside on the yoke seat plane
z_top = 170.45  # mm Top of bar and link
link_x0 = -17.0  # mm Link rear X
link_x1 = -6.5  # mm Link front X (plate outer face)
link_half_y = 10.5  # mm Link and plate half width
plate_x0 = -9.0  # mm Vertical plate rear face X
plate_z0 = 128.0  # mm Vertical plate bottom
plate_z1 = 169.5  # mm Vertical plate top (inside the link)
m3_x = -16.0  # mm M3 clamp screw X
m3_y = 28.0  # mm M3 clamp screw |Y|
m3_r = 1.7  # mm M3 clearance radius
csk_r = 3.2  # mm Countersink top radius at the bar top
csk_depth = 1.5  # mm Countersink depth below the bar top
boss_x1 = -4.9  # mm Board boss face X (board side)
boss_len = 1.7  # mm Boss length from the plate face
boss_r = 3.5  # mm Board boss radius
board_z = 143.0  # mm Board hole pattern centre Z
board_dz = 10.16  # mm Board hole half pitch along Z
board_dy = 6.35  # mm Board hole half pitch along Y
m2_r = 1.15  # mm M2 clearance radius
nut_circ_r = 2.425  # mm Hex nut pocket circumradius
nut_depth = 1.6  # mm Hex nut pocket depth from the boss face
nut_angle = 30.0  # deg Hex pocket rotation about X (30: corners toward +-Y, flats toward +-Z)

BOARD = [(sy * board_dy, board_z + sz * board_dz) for sy in (1, -1) for sz in (1, -1)]

model = Model()
model.box("clamp_bar", bar_x1 - bar_x0, 2 * bar_half_y, z_top - z_seat, at=(bar_x0, -bar_half_y, z_seat), centered=False)
model.box("link", link_x1 - link_x0, 2 * link_half_y, z_top - z_seat, at=(link_x0, -link_half_y, z_seat), centered=False, op="join")
model.box("vertical_plate", link_x1 - plate_x0, 2 * link_half_y, plate_z1 - plate_z0, at=(plate_x0, -link_half_y, plate_z0),
          centered=False, op="join")
for i, (y, z) in enumerate(BOARD):
    model.cylinder(f"boss_{i}", radius=boss_r, height=boss_len, at=(boss_x1 - boss_len, y, z), direction=(1, 0, 0), op="join")
for s, n in ((1, "p"), (-1, "n")):
    model.cylinder(f"m3_hole_{n}", radius=m3_r, height=5.0, at=(m3_x, s * m3_y, z_seat - 1.45), op="cut")
    model.add(f"countersink_{n}", cq.Solid.makeCone(m3_r, csk_r, csk_depth, cq.Vector(m3_x, s * m3_y, z_top - csk_depth),
                                                     cq.Vector(0, 0, 1)), op="cut")
for i, (y, z) in enumerate(BOARD):
    model.cylinder(f"m2_hole_{i}", radius=m2_r, height=7.0, at=(boss_x1 - 5.05, y, z), direction=(1, 0, 0), op="cut")
    pts = [(y + nut_circ_r * math.sin(math.radians(nut_angle + 60 * k)), z + nut_circ_r * math.cos(math.radians(nut_angle + 60 * k)))
           for k in range(6)]
    hexp = cq.Wire.makePolygon([cq.Vector(boss_x1 - nut_depth, py, pz) for py, pz in pts], close=True)
    model.add(f"nut_pocket_{i}", cq.Solid.extrudeLinear(cq.Face.makeFromWires(hexp), cq.Vector(nut_depth + 0.5, 0, 0)), op="cut")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#8a8a90")
result = model.result()

