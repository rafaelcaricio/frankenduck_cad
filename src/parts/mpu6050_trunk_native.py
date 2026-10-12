from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    import cadquery as cq
    from lib.steve_compat import Model
    board_x0 = -4.9
    board_t = 1.6
    board_w = 17.78
    board_h = 25.4
    board_zc = 143.0
    corner_r = 2.54
    hole_d = 2.5
    hole_dy = 6.35
    hole_dz = 10.16
    model = Model()
    board = cq.Workplane('YZ', origin=(board_x0, 0, board_zc)).rect(board_w, board_h).extrude(board_t).edges('|X').fillet(corner_r)
    model.add('board', board, op='new', body='board')
    for i, (sy, sz) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)]):
        model.cylinder(f'hole_{i}', radius=hole_d / 2, height=board_t + 1, at=(board_x0 - 0.5, sy * hole_dy, board_zc + sz * hole_dz), direction=(1, 0, 0), op='cut', body='board')
    result = model.result()
    return to_build123d(model, 'mpu6050_trunk_native')


@step(out="../../STEP/parts/mpu6050_trunk_native.step")
def mpu6050_trunk_native():
    return make()


if __name__ == "__main__":
    mpu6050_trunk_native()
