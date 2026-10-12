from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    from lib.cadquery_adapter import Model
    SIDE = -1
    axis_x = -42.899
    axis_z = 3.975
    y_inner = 39.605
    thickness = 1.0
    cap_r = 4.75
    hole_offset = 1.8
    hole_r = 0.9
    model = Model()
    model.cylinder('cap', radius=cap_r, height=thickness, at=(axis_x, SIDE * y_inner, axis_z), direction=(0, SIDE, 0))
    for n, dx in (('a', hole_offset), ('b', -hole_offset)):
        model.cylinder(f'hole_{n}', radius=hole_r, height=thickness + 2, at=(axis_x + dx, SIDE * (y_inner - 1), axis_z), direction=(0, SIDE, 0), op='cut')
    model.material('aluminum-6061-t6', color='#f96c0a')
    result = model.result()
    return to_build123d(model, 'ankle_bearing_inner_cap_left_native')


@step(out="../../STEP/parts/ankle_bearing_inner_cap_left_native_right.step")
def ankle_bearing_inner_cap_left_native_right():
    return make()


if __name__ == "__main__":
    ankle_bearing_inner_cap_left_native_right()
