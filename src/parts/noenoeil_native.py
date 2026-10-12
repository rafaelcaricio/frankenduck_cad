from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    import math
    from lib.steve_compat import Model
    axis_y = -0.12336
    axis_z = 301.50477
    ring_x0 = 109.89
    ring_len = 10.8
    outer_r = 20.25
    bore_r = 9.72
    outer_chamfer = 0.675
    bore_chamfer = 2.7
    tab_ri = 11.475
    tab_ro = 12.825
    tab_h = 2.025
    tab_w = 20.0
    tab_a0 = 90.0
    tab_count = 3
    model = Model()
    model.plane('plane_axis', on='XY', offset=axis_z, name='Axis plane')
    model.axis('axis_ring', through=(0.0, axis_y, axis_z), direction=(1, 0, 0), name='Ring axis')
    with model.sketch('sketch_ring', on='plane_axis', name='Ring profile') as s:
        s.line('l1', (ring_x0, axis_y + bore_r), (ring_x0 + ring_len - bore_chamfer, axis_y + bore_r))
        s.line('l2', (ring_x0 + ring_len - bore_chamfer, axis_y + bore_r), (ring_x0 + ring_len, axis_y + bore_r + bore_chamfer))
        s.line('l3', (ring_x0 + ring_len, axis_y + bore_r + bore_chamfer), (ring_x0 + ring_len, axis_y + outer_r - outer_chamfer))
        s.line('l4', (ring_x0 + ring_len, axis_y + outer_r - outer_chamfer), (ring_x0 + ring_len - outer_chamfer, axis_y + outer_r))
        s.line('l5', (ring_x0 + ring_len - outer_chamfer, axis_y + outer_r), (ring_x0, axis_y + outer_r))
        s.line('l6', (ring_x0, axis_y + outer_r), (ring_x0, axis_y + bore_r))
    model.revolve('revolve_ring', 'sketch_ring', axis='axis_ring', name='Ring')
    model.plane('plane_tab', on='YZ', offset=ring_x0 - tab_h, name='Tab plane')
    with model.sketch('sketch_tab', on='plane_tab', name='Tab profile') as s:
        s.arc('a_out', (axis_y, axis_z), tab_ro, tab_a0 - tab_w / 2, tab_a0 + tab_w / 2)
        s.arc('a_in', (axis_y, axis_z), tab_ri, tab_a0 - tab_w / 2, tab_a0 + tab_w / 2)
        s.line('side_1', (axis_y + tab_ri * math.cos(math.radians(tab_a0 - tab_w / 2)), axis_z + tab_ri * math.sin(math.radians(tab_a0 - tab_w / 2))), (axis_y + tab_ro * math.cos(math.radians(tab_a0 - tab_w / 2)), axis_z + tab_ro * math.sin(math.radians(tab_a0 - tab_w / 2))))
        s.line('side_2', (axis_y + tab_ri * math.cos(math.radians(tab_a0 + tab_w / 2)), axis_z + tab_ri * math.sin(math.radians(tab_a0 + tab_w / 2))), (axis_y + tab_ro * math.cos(math.radians(tab_a0 + tab_w / 2)), axis_z + tab_ro * math.sin(math.radians(tab_a0 + tab_w / 2))))
    model.extrude('extrude_tab', 'sketch_tab', distance=tab_h, op='join', name='Tab')
    model.circular_pattern('pattern_tabs', axis='axis_ring', count=tab_count, features=['extrude_tab'], name='Tab pattern')
    model.material('pla', color='#f79507')
    result = model.result()
    return to_build123d(model, 'noenoeil_native')


@step(out="../../STEP/parts/noenoeil_native.step")
def noenoeil_native():
    return make()


if __name__ == "__main__":
    noenoeil_native()
