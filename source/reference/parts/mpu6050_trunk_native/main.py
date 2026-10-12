# mpu6050_trunk_native: Adafruit MPU-6050 breakout on the trunk (purchased), board outline model.
# Robot world frame, mm, Z up. Board plane is YZ, thickness along X. The source mesh carries the
# board only (no chip or header bodies), so none are modelled.
import cadquery as cq
from source_api import Model

board_x0 = -4.9  # mm board back face X
board_t = 1.6  # mm board thickness
board_w = 17.78  # mm board width along Y (0.7 in)
board_h = 25.4  # mm board height along Z (1.0 in)
board_zc = 143.0  # mm board centre Z
corner_r = 2.54  # mm board corner radius
hole_d = 2.5  # mm mounting hole diameter
hole_dy = 6.35  # mm hole |Y| from the centre line
hole_dz = 10.16  # mm hole |Z| from the board centre

model = Model()
board = (cq.Workplane("YZ", origin=(board_x0, 0, board_zc)).rect(board_w, board_h).extrude(board_t)
         .edges("|X").fillet(corner_r))
model.add("board", board, op="new", body="board")
for i, (sy, sz) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)]):
    model.cylinder(f"hole_{i}", radius=hole_d / 2, height=board_t + 1, at=(board_x0 - 0.5, sy * hole_dy, board_zc + sz * hole_dz),
                   direction=(1, 0, 0), op="cut", body="board")
result = model.result()

