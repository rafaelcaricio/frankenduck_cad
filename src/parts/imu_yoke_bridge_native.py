from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    import math
    import cadquery as cq
    from lib.steve_compat import Model
    bar_x0 = -20.0
    bar_x1 = -12.0
    bar_half_y = 31.5
    z_seat = 168.45
    z_top = 170.45
    link_x0 = -17.0
    link_x1 = -6.5
    link_half_y = 10.5
    plate_x0 = -9.0
    plate_z0 = 128.0
    plate_z1 = 169.5
    m3_x = -16.0
    m3_y = 28.0
    m3_r = 1.7
    csk_r = 3.2
    csk_depth = 1.5
    boss_x1 = -4.9
    boss_len = 1.7
    boss_r = 3.5
    board_z = 143.0
    board_dz = 10.16
    board_dy = 6.35
    m2_r = 1.15
    nut_circ_r = 2.425
    nut_depth = 1.6
    nut_angle = 30.0
    BOARD = [(sy * board_dy, board_z + sz * board_dz) for sy in (1, -1) for sz in (1, -1)]
    model = Model()
    model.box('clamp_bar', bar_x1 - bar_x0, 2 * bar_half_y, z_top - z_seat, at=(bar_x0, -bar_half_y, z_seat), centered=False)
    model.box('link', link_x1 - link_x0, 2 * link_half_y, z_top - z_seat, at=(link_x0, -link_half_y, z_seat), centered=False, op='join')
    model.box('vertical_plate', link_x1 - plate_x0, 2 * link_half_y, plate_z1 - plate_z0, at=(plate_x0, -link_half_y, plate_z0), centered=False, op='join')
    for i, (y, z) in enumerate(BOARD):
        model.cylinder(f'boss_{i}', radius=boss_r, height=boss_len, at=(boss_x1 - boss_len, y, z), direction=(1, 0, 0), op='join')
    for s, n in ((1, 'p'), (-1, 'n')):
        model.cylinder(f'm3_hole_{n}', radius=m3_r, height=5.0, at=(m3_x, s * m3_y, z_seat - 1.45), op='cut')
        model.add(f'countersink_{n}', cq.Solid.makeCone(m3_r, csk_r, csk_depth, cq.Vector(m3_x, s * m3_y, z_top - csk_depth), cq.Vector(0, 0, 1)), op='cut')
    for i, (y, z) in enumerate(BOARD):
        model.cylinder(f'm2_hole_{i}', radius=m2_r, height=7.0, at=(boss_x1 - 5.05, y, z), direction=(1, 0, 0), op='cut')
        pts = [(y + nut_circ_r * math.sin(math.radians(nut_angle + 60 * k)), z + nut_circ_r * math.cos(math.radians(nut_angle + 60 * k))) for k in range(6)]
        hexp = cq.Wire.makePolygon([cq.Vector(boss_x1 - nut_depth, py, pz) for py, pz in pts], close=True)
        model.add(f'nut_pocket_{i}', cq.Solid.extrudeLinear(cq.Face.makeFromWires(hexp), cq.Vector(nut_depth + 0.5, 0, 0)), op='cut')
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'imu_yoke_bridge_native')


@step(out="../../STEP/parts/imu_yoke_bridge_native.step")
def imu_yoke_bridge_native():
    return make()


if __name__ == "__main__":
    imu_yoke_bridge_native()
