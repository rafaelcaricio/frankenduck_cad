from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    """Experimental face for MightyCamera, copied from face_part_native revision 6. Canonical untouched."""
    import math
    from lib.cadquery_adapter import Model
    sym_y = -0.12336
    ring_z = 301.50477
    x_back = 108.14
    plate_t = 1.75
    half_w_back = 59.0426
    half_w_front = 59.1636
    z_arch = 274.4997
    z_bottom = 273.425
    lens_z = 301.50477
    lens_r = 8.5
    small_z = 296.05
    small_r = 2.85
    slot_y = 30.252
    slot_z = 298.805
    slot_half = 3.2065
    slot_r = 4.05
    tab_ri = 11.4
    tab_ro = 12.885
    tab_w = 20.68
    so_dy = 39.042
    so_dz = 15.4375
    so_x0 = 93.015
    so_r = 4.05
    so_ch = 0.67
    so_cb_r = 2.0
    so_cb_x = 98.22
    so_pilot_r = 1.39
    so_pilot_x = 99.77
    so_tip_x = 100.6
    boss_y = 10.5
    boss_z_hi = 317.0
    boss_z_lo = 304.5
    boss_x0 = 102.9
    boss_r = 2.8
    boss_collar_r = 3.3
    boss_collar_x = 107.8
    boss_fr = 0.5
    boss_hole_r = 0.8
    boss_hole_x = 108.3
    tof_board_length = 24.81
    tof_board_width = 10.16
    tof_mount_pitch = 18.72
    tof_sensor_length = 7.02
    ear_l_y = slot_y - tof_mount_pitch / 2
    ear_r_y = slot_y + tof_mount_pitch / 2
    ear_z = 298.805
    tof_post_length = 2.5
    ear_x0 = x_back - tof_post_length
    ear_r = 3.0
    ear_fr = 0.6
    ear_hole_r = 1.2
    tof_pilot_depth = 3.0
    ear_hole_x = ear_x0 + tof_pilot_depth
    rim_x0 = 106.785
    rim_r = 5.4
    rim_hole_r = 4.117
    notch_z = 301.505
    notch_r = 4.85
    clip_x0 = 105.5
    clip_lip_x = 106.2
    clip_pad_x = 107.6
    clip_z0 = 290.75
    clip_z1 = 301.4
    model = Model()
    model.plane('plane_back', on='YZ', offset=x_back, name='Plate back plane')
    with model.sketch('sketch_plate_back', on='plane_back', name='Plate outline, back face') as s:
        s.line('edge_r', (sym_y + half_w_back, z_bottom), (sym_y + half_w_back, z_arch))
        s.spline('arch', [(sym_y + half_w_back, z_arch), (58.7563, 279.258), (57.4053, 289.6711), (53.8059, 303.5096), (47.7028, 317.8332), (37.0203, 328.222), (21.9259, 332.49), (7.5648, 333.5904), (-7.8115, 333.5904), (-22.1727, 332.49), (-37.267, 328.222), (-47.9495, 317.8332), (-54.0526, 303.5096), (-57.652, 289.6711), (-59.003, 279.258), (sym_y - half_w_back, z_arch)])
        s.line('edge_l', (sym_y - half_w_back, z_arch), (sym_y - half_w_back, z_bottom))
        s.line('bottom', (sym_y - half_w_back, z_bottom), (sym_y + half_w_back, z_bottom))
    model.plane('plane_front', on='YZ', offset=x_back + plate_t, name='Plate front plane')
    with model.sketch('sketch_plate_front', on='plane_front', name='Plate outline, front face') as s:
        s.line('edge_r', (sym_y + half_w_front, z_bottom), (sym_y + half_w_front, z_arch))
        s.spline('arch', [(sym_y + half_w_front, z_arch), (58.8631, 279.3734), (57.5347, 289.5824), (53.9258, 303.5737), (47.7588, 317.9338), (37.1072, 328.3004), (21.9672, 332.6177), (7.5449, 333.7116), (-7.7917, 333.7116), (-22.214, 332.6177), (-37.3539, 328.3004), (-48.0055, 317.9338), (-54.1726, 303.5737), (-57.7814, 289.5824), (-59.1098, 279.3734), (sym_y - half_w_front, z_arch)])
        s.line('edge_l', (sym_y - half_w_front, z_arch), (sym_y - half_w_front, z_bottom))
        s.line('bottom', (sym_y - half_w_front, z_bottom), (sym_y + half_w_front, z_bottom))
    model.loft('loft_plate', ['sketch_plate_back', 'sketch_plate_front'], ruled=True, name='Plate (drafted edge)')
    with model.sketch('sketch_openings', on='plane_back', name='Plate openings') as s:
        s.circle('lens', (sym_y, lens_z), lens_r)
        s.arc('st_r', (slot_y + slot_half, slot_z), slot_r, -90.0, 90.0)
        s.line('st_t', (slot_y + slot_half, slot_z + slot_r), (slot_y - slot_half, slot_z + slot_r))
        s.arc('st_l', (slot_y - slot_half, slot_z), slot_r, 90.0, 270.0)
        s.line('st_b', (slot_y - slot_half, slot_z - slot_r), (slot_y + slot_half, slot_z - slot_r))
        for k, a0 in enumerate((90.0, 210.0, 330.0)):
            a1 = math.radians(a0 - tab_w / 2)
            a2 = math.radians(a0 + tab_w / 2)
            s.arc(f'tab_o{k}', (sym_y, ring_z), tab_ro, a0 - tab_w / 2, a0 + tab_w / 2)
            s.arc(f'tab_i{k}', (sym_y, ring_z), tab_ri, a0 - tab_w / 2, a0 + tab_w / 2)
            s.line(f'tab_a{k}', (sym_y + tab_ri * math.cos(a1), ring_z + tab_ri * math.sin(a1)), (sym_y + tab_ro * math.cos(a1), ring_z + tab_ro * math.sin(a1)))
            s.line(f'tab_b{k}', (sym_y + tab_ri * math.cos(a2), ring_z + tab_ri * math.sin(a2)), (sym_y + tab_ro * math.cos(a2), ring_z + tab_ro * math.sin(a2)))
    model.extrude('cut_openings', 'sketch_openings', distance=plate_t, op='cut', name='Plate openings')
    model.plane('plane_ear', on='XY', offset=ear_z, name='Ear axis plane')
    model.axis('axis_ear_l', through=(0.0, ear_l_y, ear_z), direction=(1, 0, 0), name='Left ear axis')
    model.axis('axis_ear_r', through=(0.0, ear_r_y, ear_z), direction=(1, 0, 0), name='Right ear axis')
    for side, ey in (('l', ear_l_y), ('r', ear_r_y)):
        with model.sketch(f'sketch_ear_{side}', on='plane_ear', name=f'Ear profile {side}') as s:
            s.line('e1', (ear_x0, ey), (ear_x0, ey + ear_r))
            s.line('e2', (ear_x0, ey + ear_r), (x_back - ear_fr, ey + ear_r))
            s.arc('e3', (x_back - ear_fr, ey + ear_r + ear_fr), ear_fr, -90.0, 0.0)
            s.line('e4', (x_back, ey + ear_r + ear_fr), (x_back, ey))
            s.line('e5', (x_back, ey), (ear_x0, ey))
        model.revolve(f'revolve_ear_{side}', f'sketch_ear_{side}', axis=f'axis_ear_{side}', op='join', name=f'Ear {side}')
    model.plane('plane_ear_end', on='YZ', offset=ear_x0, name='Ear and bracket end plane')
    camera_pcb_back_x = 98.2
    camera_pcb_t = 1.5162
    mount_seat_x = camera_pcb_back_x + camera_pcb_t
    mount_r = 2.0
    mount_root_r = 3.0
    mount_pilot_r = 0.8
    mount_pilot_depth = 5.0
    model.plane('plane_mighty_seat', on='YZ', offset=mount_seat_x)
    with model.sketch('sketch_mighty_seats', on='plane_mighty_seat') as s:
        for k, (yy, zz) in enumerate(((sym_y - 14.5, ring_z + 12.5), (sym_y - 14.5, ring_z - 13.5), (sym_y + 14.5, ring_z + 12.5), (sym_y + 14.5, ring_z - 13.5))):
            s.circle(f'seat{k}', (yy, zz), mount_r)
    model.extrude('mighty_mount_posts', 'sketch_mighty_seats', distance=x_back - mount_seat_x, op='join')
    model.plane('plane_mighty_roots', on='YZ', offset=x_back - 1.5)
    with model.sketch('sketch_mighty_roots', on='plane_mighty_roots') as s:
        for k, (yy, zz) in enumerate(((sym_y - 14.5, ring_z + 12.5), (sym_y - 14.5, ring_z - 13.5), (sym_y + 14.5, ring_z + 12.5), (sym_y + 14.5, ring_z - 13.5))):
            s.circle(f'root{k}', (yy, zz), mount_root_r)
    model.extrude('mighty_mount_roots', 'sketch_mighty_roots', distance=1.5, op='join')
    with model.sketch('sketch_mighty_pilots', on='plane_mighty_seat') as s:
        for k, (yy, zz) in enumerate(((sym_y - 14.5, ring_z + 12.5), (sym_y - 14.5, ring_z - 13.5), (sym_y + 14.5, ring_z + 12.5), (sym_y + 14.5, ring_z - 13.5))):
            s.circle(f'pilot{k}', (yy, zz), mount_pilot_r)
    model.extrude('mighty_mount_pilots', 'sketch_mighty_pilots', distance=mount_pilot_depth, op='cut')
    for zname, sz in (('hi', ring_z + so_dz), ('lo', ring_z - so_dz)):
        model.plane(f'plane_so_{zname}', on='XY', offset=sz, name=f'Standoff plane {zname}')
        for side, sy in (('l', sym_y - so_dy), ('r', sym_y + so_dy)):
            model.axis(f'axis_so_{zname}_{side}', through=(0.0, sy, sz), direction=(1, 0, 0), name=f'Standoff axis {zname} {side}')
            with model.sketch(f'sketch_so_{zname}_{side}', on=f'plane_so_{zname}', name=f'Standoff profile {zname} {side}') as s:
                s.line('s1', (so_x0, sy + so_cb_r), (so_x0, sy + so_r - so_ch))
                s.line('s2', (so_x0, sy + so_r - so_ch), (so_x0 + so_ch, sy + so_r))
                s.line('s3', (so_x0 + so_ch, sy + so_r), (x_back, sy + so_r))
                s.line('s4', (x_back, sy + so_r), (x_back, sy))
                s.line('s5', (x_back, sy), (so_tip_x, sy))
                s.line('s6', (so_tip_x, sy), (so_pilot_x, sy + so_pilot_r))
                s.line('s7', (so_pilot_x, sy + so_pilot_r), (so_cb_x, sy + so_pilot_r))
                s.line('s8', (so_cb_x, sy + so_pilot_r), (so_cb_x, sy + so_cb_r))
                s.line('s9', (so_cb_x, sy + so_cb_r), (so_x0, sy + so_cb_r))
            model.revolve(f'revolve_so_{zname}_{side}', f'sketch_so_{zname}_{side}', axis=f'axis_so_{zname}_{side}', op='join', name=f'Standoff {zname} {side}')
    with model.sketch('sketch_ear_holes', on='plane_ear_end', name='Ear pilot holes') as s:
        s.circle('eh1', (ear_l_y, ear_z), ear_hole_r)
        s.circle('eh2', (ear_r_y, ear_z), ear_hole_r)
    model.extrude('cut_ear_holes', 'sketch_ear_holes', distance=ear_hole_x - ear_x0, op='cut', name='Ear pilot holes')
    model.material('pla', color='#9b9891')
    result = model.result()
    return to_build123d(model, 'face_part_native')


@step(out="../../STEP/parts/face_part_native.step")
def face_part_native():
    return make()


if __name__ == "__main__":
    face_part_native()
