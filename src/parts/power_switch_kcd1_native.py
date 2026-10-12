from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    from lib.cadquery_adapter import Model
    axis_x = -36.0
    axis_z = 139.0
    bezel_y = -44.6
    bezel_d = 16.5
    bezel_t = 1.6
    body_d = 15.0
    body_y1 = -30.6
    term_w = 10.0
    term_h = 3.6
    term_y1 = -23.4
    model = Model()
    model.cylinder('bezel', radius=bezel_d / 2, height=bezel_t, at=(axis_x, bezel_y, axis_z), direction=(0, 1, 0), op='new', body='switch')
    model.cylinder('body', radius=body_d / 2, height=body_y1 - bezel_y - bezel_t, at=(axis_x, bezel_y + bezel_t, axis_z), direction=(0, 1, 0), op='join', body='switch')
    model.box('terminals', length=term_w, width=term_y1 - body_y1, height=term_h, at=(axis_x, (body_y1 + term_y1) / 2, axis_z), op='join', body='switch')
    result = model.result()
    return to_build123d(model, 'power_switch_kcd1_native')


@step(out="../../STEP/parts/power_switch_kcd1_native.step")
def power_switch_kcd1_native():
    return make()


if __name__ == "__main__":
    power_switch_kcd1_native()
