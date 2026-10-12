from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    """face_radxa_bracket_native: Frankenduck head, face Radxa bracket (R02), robot world frame, Z up, mm.
    
    Native port of face_bracket() in head_internals/code/build_parts_r05.py (build123d).
    Local Text to CAD revision: 3.2 mm rails, short face screw pads and a
    12 mm sideways PCB shift for full jumper housings and upward wires.
    Three existing face bosses are used; the upper-left face boss is unused.
    M3 countersunk screws seat flush beneath the PCB. The four Cubie M2
    clearance holes retain their pan-head wells and 1.6 mm screw grip.
    No heat-set insert pockets in this part (the M3 inserts sit in the face bosses, not here).
    """
    from lib.steve_compat import Model
    x0 = 76.0
    # A 3.2 mm rail keeps the 1.6 mm screw grip and moves the PCB 2 mm
    # toward the face, giving the rear-facing GPIO plugs a larger exit bay.
    x1 = 79.2
    rail_r = 2.0
    low_rail_y = 46.0
    low_rail_z0 = 286.5
    low_rail_z1 = 294.5
    top_rail_y = 44.5
    top_rail_z0 = 309.5
    top_rail_z1 = 318.0
    tie_y0 = 38.5
    tie_y1 = 43.0
    tie_z0 = 286.5
    tie_z1 = 312.0
    # Remove the three long rearward tubes marked in the user's screenshot.
    boss_x1 = x1
    boss_left_y = -39.17
    boss_right_y = 38.92
    boss_low_z = 286.07
    boss_up_z = 316.94
    post_r_low = 4.2
    post_r_up = 3.8
    m3_clr_r = 1.7
    radxa_y0 = -17.12
    radxa_y1 = 40.88
    radxa_z0 = 291.0
    radxa_z1 = 314.0
    m2_clr_r = 1.2
    m2_well_r = 2.2
    m2_grip = 1.6
    model = Model()
    model.plane('plane_back', on='YZ', offset=x0, name='Bracket back plane')
    with model.sketch('sketch_low_rail', on='plane_back', name='Lower rail profile') as s:
        s.line('lr_b', (-low_rail_y + rail_r, low_rail_z0), (low_rail_y - rail_r, low_rail_z0))
        s.arc('lr_br', (low_rail_y - rail_r, low_rail_z0 + rail_r), rail_r, 270, 360)
        s.line('lr_r', (low_rail_y, low_rail_z0 + rail_r), (low_rail_y, low_rail_z1 - rail_r))
        s.arc('lr_tr', (low_rail_y - rail_r, low_rail_z1 - rail_r), rail_r, 0, 90)
        s.line('lr_t', (low_rail_y - rail_r, low_rail_z1), (-low_rail_y + rail_r, low_rail_z1))
        s.arc('lr_tl', (-low_rail_y + rail_r, low_rail_z1 - rail_r), rail_r, 90, 180)
        s.line('lr_l', (-low_rail_y, low_rail_z1 - rail_r), (-low_rail_y, low_rail_z0 + rail_r))
        s.arc('lr_bl', (-low_rail_y + rail_r, low_rail_z0 + rail_r), rail_r, 180, 270)
    model.extrude('low_rail', 'sketch_low_rail', distance=x1 - x0, body='bracket', name='Lower rail')
    with model.sketch('sketch_top_rail', on='plane_back', name='Top rail profile') as s:
        s.line('tr_b', (-top_rail_y + rail_r, top_rail_z0), (top_rail_y - rail_r, top_rail_z0))
        s.arc('tr_br', (top_rail_y - rail_r, top_rail_z0 + rail_r), rail_r, 270, 360)
        s.line('tr_r', (top_rail_y, top_rail_z0 + rail_r), (top_rail_y, top_rail_z1 - rail_r))
        s.arc('tr_tr', (top_rail_y - rail_r, top_rail_z1 - rail_r), rail_r, 0, 90)
        s.line('tr_t', (top_rail_y - rail_r, top_rail_z1), (-top_rail_y + rail_r, top_rail_z1))
        s.arc('tr_tl', (-top_rail_y + rail_r, top_rail_z1 - rail_r), rail_r, 90, 180)
        s.line('tr_l', (-top_rail_y, top_rail_z1 - rail_r), (-top_rail_y, top_rail_z0 + rail_r))
        s.arc('tr_bl', (-top_rail_y + rail_r, top_rail_z0 + rail_r), rail_r, 180, 270)
    model.extrude('top_rail', 'sketch_top_rail', distance=x1 - x0, op='join', body='bracket', name='Top rail')
    model.box('tie_left', x1 - x0, tie_y1 - tie_y0, tie_z1 - tie_z0, at=((x0 + x1) / 2, -(tie_y0 + tie_y1) / 2, (tie_z0 + tie_z1) / 2), op='join', body='bracket', name='Side tie -Y')
    model.box('tie_right', x1 - x0, tie_y1 - tie_y0, tie_z1 - tie_z0, at=((x0 + x1) / 2, (tie_y0 + tie_y1) / 2, (tie_z0 + tie_z1) / 2), op='join', body='bracket', name='Side tie +Y')
    with model.sketch('sketch_posts', on='plane_back', name='Face boss posts') as s:
        s.circle('post_left_up', (boss_left_y, boss_up_z), post_r_up)
        # After the assembly flip, this is the right pair. Use the spare
        # lower-left face boss so the shifted PCB's upper M2 well stays clear.
        s.circle('post_left_low', (boss_left_y, boss_low_z), post_r_low)
        s.circle('post_right_low', (boss_right_y, boss_low_z), post_r_low)
    model.extrude('posts', 'sketch_posts', distance=boss_x1 - x0, op='join', body='bracket', name='Short face screw pads')
    # The full 53 mm connector bay is behind the PCB seating plane. Keep
    # solid lands around the M2 wells and the new lower-left M3 seat here.
    model.box('gpio_opening', x1 - x0 + 2.0, 48.0, 10.0,
              at=((x0 + x1) / 2, 10.13, 290.0), op='cut', body='bracket',
              name='GPIO relief between protected screw lands')
    with model.sketch('sketch_m3', on='plane_back', name='M3 clearance through posts') as s:
        s.circle('m3_left_up', (boss_left_y, boss_up_z), m3_clr_r)
        s.circle('m3_left_low', (boss_left_y, boss_low_z), m3_clr_r)
        s.circle('m3_right_low', (boss_right_y, boss_low_z), m3_clr_r)
    model.extrude('m3_clearance', 'sketch_m3', distance=boss_x1 - x0, op='cut', body='bracket', name='M3 clearance')
    # 90-degree flat-head seats keep M3 heads below the PCB mounting plane.
    # Radius 3.0 mm allows 0.2 mm around a nominal 5.6 mm flat screw head.
    for i, (yy, zz) in enumerate([(boss_left_y, boss_up_z),
                                 (boss_left_y, boss_low_z),
                                 (boss_right_y, boss_low_z)]):
        model.cone(f'm3_flat_head_{i}', m3_clr_r, 3.0, 1.3,
                   at=(x1 - 1.3, yy, zz), direction=(1, 0, 0),
                   op='cut', body='bracket', name='Flush M3 flat-head seat')
    with model.sketch('sketch_m2', on='plane_back', name='M2 clearance (Cubie)') as s:
        s.circle('m2_0', (radxa_y0, radxa_z0), m2_clr_r)
        s.circle('m2_1', (radxa_y1, radxa_z0), m2_clr_r)
        s.circle('m2_2', (radxa_y0, radxa_z1), m2_clr_r)
        s.circle('m2_3', (radxa_y1, radxa_z1), m2_clr_r)
    model.extrude('m2_clearance', 'sketch_m2', distance=x1 - x0, op='cut', body='bracket', name='M2 clearance')
    with model.sketch('sketch_m2_wells', on='plane_back', name='M2 pan-head wells') as s:
        s.circle('well_0', (radxa_y0, radxa_z0), m2_well_r)
        s.circle('well_1', (radxa_y1, radxa_z0), m2_well_r)
        s.circle('well_2', (radxa_y0, radxa_z1), m2_well_r)
        s.circle('well_3', (radxa_y1, radxa_z1), m2_well_r)
    model.extrude('m2_wells', 'sketch_m2_wells', distance=x1 - m2_grip - x0, op='cut', body='bracket', name='M2 pan-head wells')
    # After the front/back flip, this lower M2 foot sits over the UBEC clamp.
    # Keep the clamp's fixed mounts and give its upper surface 0.5 mm clearance.
    model.box('flipped_ubec_clearance', 22.0, 9.0, 12.0,
              at=(84.51, 24.75, 282.0), op='cut', body='bracket',
              name='UBEC clearance beneath flipped lower foot')
    model.box('lower_left_ubec_relief', x1 - x0 + 2.0, 10.0, 6.0,
              at=((x0 + x1) / 2, boss_right_y, 280.0), op='cut',
              body='bracket', name='UBEC clearance beneath lower-left M3 pad')
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'face_radxa_bracket_native')


@step(out="../../STEP/parts/face_radxa_bracket_native.step")
def face_radxa_bracket_native():
    return make()


if __name__ == "__main__":
    face_radxa_bracket_native()
