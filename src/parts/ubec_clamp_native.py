from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    """Head-mounted UBEC retaining clamp. World millimetres, Z up."""
    from lib.cadquery_adapter import Model
    clamp_z0 = 282.9
    clamp_z1 = 285.9
    bar_x0 = 65.0
    bar_x1 = 99.0
    bar_y0 = -28.5
    bar_y1 = -23.8
    bar_r = 2.0
    rib_x0 = 71.5
    rib_x1 = 75.5
    rib_y0 = -27.2
    rib_y1 = -25.2
    rib_z1 = 287.5
    boss_y = -25.0
    boss1_x = 68.5
    boss2_x = 95.5
    pad_r = 4.0
    m2_clear_d = 2.4
    well_r = 2.2
    grip = 2.0
    model = Model()
    model.plane('plane_base', on='XY', offset=clamp_z0, name='Clamp underside')
    with model.sketch('sketch_bar', on='plane_base', name='Bar outline') as s:
        s.line('bar_b', (bar_x0 + bar_r, bar_y0), (bar_x1 - bar_r, bar_y0))
        s.arc('bar_br', (bar_x1 - bar_r, bar_y0 + bar_r), bar_r, 270, 360)
        s.line('bar_rt', (bar_x1, bar_y0 + bar_r), (bar_x1, bar_y1 - bar_r))
        s.arc('bar_tr', (bar_x1 - bar_r, bar_y1 - bar_r), bar_r, 0, 90)
        s.line('bar_t', (bar_x1 - bar_r, bar_y1), (bar_x0 + bar_r, bar_y1))
        s.arc('bar_tl', (bar_x0 + bar_r, bar_y1 - bar_r), bar_r, 90, 180)
        s.line('bar_lt', (bar_x0, bar_y1 - bar_r), (bar_x0, bar_y0 + bar_r))
        s.arc('bar_bl', (bar_x0 + bar_r, bar_y0 + bar_r), bar_r, 180, 270)
    model.extrude('bar', 'sketch_bar', distance=clamp_z1 - clamp_z0, name='Clamp bar')
    model.box('rib', rib_x1 - rib_x0, rib_y1 - rib_y0, rib_z1 - clamp_z1, at=((rib_x0 + rib_x1) / 2, (rib_y0 + rib_y1) / 2, (clamp_z1 + rib_z1) / 2), op='join', name='Stiffening rib')
    model.cylinder('pad_rear', pad_r, clamp_z1 - clamp_z0, at=(boss1_x, boss_y, clamp_z0), op='join', name='Rear screw pad')
    model.cylinder('pad_front', pad_r, clamp_z1 - clamp_z0, at=(boss2_x, boss_y, clamp_z0), op='join', name='Front screw pad')
    model.cylinder('clear_rear', m2_clear_d / 2, clamp_z1 - clamp_z0 + 3.0, at=(boss1_x, boss_y, clamp_z0 - 1.0), op='cut', name='Rear M2 clearance')
    model.cylinder('clear_front', m2_clear_d / 2, clamp_z1 - clamp_z0 + 3.0, at=(boss2_x, boss_y, clamp_z0 - 1.0), op='cut', name='Front M2 clearance')
    model.cylinder('well_rear', well_r, clamp_z1 - clamp_z0 - grip + 2.0, at=(boss1_x, boss_y, clamp_z0 + grip), op='cut', name='Rear pan-head well')
    model.cylinder('well_front', well_r, clamp_z1 - clamp_z0 - grip + 2.0, at=(boss2_x, boss_y, clamp_z0 + grip), op='cut', name='Front pan-head well')
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'ubec_clamp_native')


@step(out="../../STEP/parts/ubec_clamp_native.step")
def ubec_clamp_native():
    return make()


if __name__ == "__main__":
    ubec_clamp_native()
