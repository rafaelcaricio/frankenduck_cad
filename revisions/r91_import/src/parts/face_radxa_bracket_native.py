from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    """face_radxa_bracket_native: Frankenduck head, face Radxa bracket (R02), robot world frame, Z up, mm.
    
    Native port of face_bracket() in head_internals/code/build_parts_r05.py (build123d).
    Holds the Radxa Cubie A7Z + 6530B on three of the four face bosses (the lower -Y boss is unused).
    Construction: two rounded rails in the YZ plane (x 76..81.2), two side ties, three bosses along +X
    to the face boss end face (x 93.02), rear-facing GPIO opening in the lower rail, M3 clearance through the
    bosses, four M2 clearance holes with pan-head wells for the Cubie screws.
    No heat-set insert pockets in this part (the M3 inserts sit in the face bosses, not here).
    """
    from lib.cadquery_adapter import Model
    x0 = 76.0
    x1 = 81.2
    rail_r = 2.0
    low_rail_y = 43.0
    low_rail_z0 = 286.5
    low_rail_z1 = 294.5
    top_rail_y = 41.0
    top_rail_z0 = 309.5
    top_rail_z1 = 318.0
    tie_y0 = 38.5
    tie_y1 = 43.0
    tie_z0 = 286.5
    tie_z1 = 312.0
    boss_x1 = 93.02
    boss_left_y = -39.17
    boss_right_y = 38.92
    boss_low_z = 286.07
    boss_up_z = 316.94
    post_r_low = 4.2
    post_r_up = 3.8
    m3_clr_r = 1.7
    radxa_y0 = -29.12
    radxa_y1 = 28.88
    radxa_z0 = 290.5
    radxa_z1 = 313.5
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
        s.circle('post_right_low', (boss_right_y, boss_low_z), post_r_low)
        s.circle('post_right_up', (boss_right_y, boss_up_z), post_r_up)
    model.extrude('posts', 'sketch_posts', distance=boss_x1 - x0, op='join', body='bracket', name='Face boss posts')
    model.box('gpio_opening', x1 - x0 + 2.0, 52.0, 10.0, at=((x0 + x1) / 2, -0.12, 290.0), op='cut', body='bracket', name='Rear GPIO header clearance')
    with model.sketch('sketch_m3', on='plane_back', name='M3 clearance through posts') as s:
        s.circle('m3_left_up', (boss_left_y, boss_up_z), m3_clr_r)
        s.circle('m3_right_low', (boss_right_y, boss_low_z), m3_clr_r)
        s.circle('m3_right_up', (boss_right_y, boss_up_z), m3_clr_r)
    model.extrude('m3_clearance', 'sketch_m3', distance=boss_x1 - x0, op='cut', body='bracket', name='M3 clearance')
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
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'face_radxa_bracket_native')


@step(out="../../STEP/parts/face_radxa_bracket_native.step")
def face_radxa_bracket_native():
    return make()


if __name__ == "__main__":
    face_radxa_bracket_native()
