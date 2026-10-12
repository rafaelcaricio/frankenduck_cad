from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    """Head roll bulkhead with F688 bearing seat, speaker mounts and self-forming M3 holes. World millimetres."""
    import math
    import cadquery as cq
    from lib.cadquery_adapter import Model
    x_back = 53.0
    x_front = 60.0
    roll_axis_z = 280.58
    bore_r = 8.275
    bore_x1 = 58.0
    flange_r = 9.4
    flange_x1 = 59.1
    race_clear_r = 5.8
    race_clear_x0 = 59.0
    race_clear_x1 = 62.0
    ring_r = 11.0
    ring_x0 = 59.5
    ring_x1 = 60.8
    foot_r = 4.2
    foot_z0 = 274.0
    foot_z1 = 283.0
    foot1_x = 56.84
    foot1_y = -40.5
    foot2_x = 63.29
    foot2_y = 11.12
    leg_y0 = 10.0
    leg_y1 = 14.6
    pad_x1 = 61.5
    pad_y0 = -0.5
    pad_y1 = 8.5
    pad_z0 = 294.0
    top_z = 300.6
    corner_y0 = 8.4
    corner_y1 = 10.5
    corner_z0 = 271.0
    deck_x = 57.0
    deck_y = 4.0
    deck_depth = 5.5
    deck_taper = 2.0
    bkhd_y = -45.3
    bkhd_z_lo = 287.0
    bkhd_z_hi = 295.4
    m3_clr = 3.4
    cbore_r = 3.2
    cbore_depth = 1.5
    sft_d = 3.0
    sft_margin = 0.15
    sft_ridge_h = 0.4
    sft_taper = 3.0
    sft_ridges = 3
    stemma_y0 = -40.0
    stemma_z_top = 318.8
    # Move the speaker 5 mm towards the bulkhead to free the GPIO wire bay.
    stemma_seat_x = 60.0
    stemma_base_x = 59.5
    stemma_post_r = 2.8
    stemma_pilot_d = 1.6
    PROFILE = [(-50.5, 276.0), (-45.5, 274.6), (-45.5, 274.0), (-33.0, 274.0), (-30.0, 271.0), (-18.0, 271.0), (-12.0, 265.0), (4.0, 265.0), (8.4, 269.4), (8.4, 274.0), (14.6, 274.0), (14.6, 300.6), (8.0, 300.6), (-26.0, 300.6), (-40.0, 304.25), (-48.0, 304.25), (-50.5, 300.0)]
    PROFILE_R = [1, 0, 0, 0, 2, 2, 2, 2, 1, 0, 0, 0, 0, 6, 2, 1, 1]
    
    def _rounded_profile(pts, radii):
        """Corner-rounded closed polygon as ('line', p, q) and ('arc', centre, r, a0, a1) items (angles in degrees,
        counter-clockwise a0 -> a1). Uses tangent lengths at each rounded corner."""
        n = len(pts)
        corners = []
        for i in range(n):
            p0, p1, p2 = (pts[i - 1], pts[i], pts[(i + 1) % n])
            r = radii[i]
            if r <= 0:
                corners.append((p1, p1, None))
                continue
            ax, ay = (p0[0] - p1[0], p0[1] - p1[1])
            bx, by = (p2[0] - p1[0], p2[1] - p1[1])
            la, lb = (math.hypot(ax, ay), math.hypot(bx, by))
            ax, ay, bx, by = (ax / la, ay / la, bx / lb, by / lb)
            ang = math.acos(max(-1.0, min(1.0, ax * bx + ay * by)))
            t = min(r / math.tan(ang / 2), 0.45 * la, 0.45 * lb)
            rad = t * math.tan(ang / 2)
            q0 = (p1[0] + ax * t, p1[1] + ay * t)
            q1 = (p1[0] + bx * t, p1[1] + by * t)
            sx, sy = (ax + bx, ay + by)
            sl = math.hypot(sx, sy)
            k = rad / math.sin(ang / 2) / sl
            c = (p1[0] + sx * k, p1[1] + sy * k)
            a0 = math.degrees(math.atan2(q0[1] - c[1], q0[0] - c[0]))
            a1 = math.degrees(math.atan2(q1[1] - c[1], q1[0] - c[0]))
            da = (a1 - a0 + 180.0) % 360.0 - 180.0
            lo, hi = (a0, a0 + da) if da > 0 else (a0 + da, a0)
            corners.append((q0, q1, (c, rad, lo, hi)))
        items = []
        for i in range(n):
            q0, q1, arc = corners[i]
            if arc is not None:
                items.append(('arc',) + arc)
            nxt = corners[(i + 1) % n][0]
            items.append(('line', q1, nxt))
        return items
    
    def _draw_profile(s, tag):
        for k, it in enumerate(_rounded_profile(PROFILE, PROFILE_R)):
            if it[0] == 'line':
                s.line(f'{tag}_l{k}', it[1], it[2])
            else:
                s.arc(f'{tag}_a{k}', it[1], it[2], it[3], it[4])
    
    def self_forming_hole(x, y, z_face, z_end, d=sft_d, margin=sft_margin, ridge_h=sft_ridge_h, taper=sft_taper, ridges=sft_ridges, overshoot=2.0):
        """Cutting tool for a self-forming thread hole on a vertical axis at (x, y) (from top_head_shell_native).
    
        The screw enters at z_face and the hole runs to z_end (either direction). Bore radius r = (d + margin) / 2;
        each ridge is what a large cylinder (radius R = r + 5) leaves when it overlaps the bore by ridge_h, so the
        ridge tips sit at r - ridge_h. Over the first `taper` mm from z_face the overlap grows from 0 to ridge_h.
        The bore also runs `overshoot` mm outside the entry face so the cut opens cleanly.
        """
        s = 1.0 if z_end > z_face else -1.0
        r = (d + margin) / 2.0
        big_r = r + 5.0
        off = big_r + r - ridge_h
        z_out = z_face - s * overshoot
        z_lo, z_hi = (min(z_out, z_end), max(z_out, z_end))
        tool = cq.Solid.makeCylinder(r, z_hi - z_lo, cq.Vector(x, y, z_lo))
        dz = cq.Vector(0, 0, s)
        for i in range(int(ridges)):
            a = 2.0 * math.pi * i / ridges
            cx, cy = (x + off * math.cos(a), y + off * math.sin(a))
            entry = cq.Solid.makeCylinder(big_r - ridge_h, overshoot, cq.Vector(cx, cy, z_out), dz)
            ramp = cq.Solid.makeCone(big_r - ridge_h, big_r, taper, cq.Vector(cx, cy, z_face), dz)
            full = cq.Solid.makeCylinder(big_r, abs(z_end - z_face) - taper, cq.Vector(cx, cy, z_face + s * taper), dz)
            tool = tool.cut(entry.fuse(ramp).fuse(full).clean())
        return tool.clean()
    model = Model()
    model.plane('plane_back', on='YZ', offset=x_back, name='Bulkhead back face')
    with model.sketch('sketch_profile', on='plane_back', name='Bulkhead side profile') as s:
        _draw_profile(s, 'p')
    model.extrude('plate', 'sketch_profile', distance=x_front - x_back, body='bulkhead', name='Bulkhead plate')
    for i, (yy, zz) in enumerate([(stemma_y0 + y, stemma_z_top - 27.94) for y in (2.54, 34.29)]):
        model.cylinder(f'stemma_post_{i}', stemma_post_r, stemma_seat_x - stemma_base_x, at=(stemma_base_x, yy, zz), direction=(1, 0, 0), op='join', body='bulkhead', name=f'STEMMA M2 seat {i + 1}')
        model.cylinder(f'stemma_pilot_{i}', stemma_pilot_d / 2, 7.5, at=(stemma_seat_x + 1.0, yy, zz), direction=(-1, 0, 0), op='cut', body='bulkhead', name=f'STEMMA M2 pilot {i + 1}')
    model.cylinder('foot1', foot_r, foot_z1 - foot_z0, at=(foot1_x, foot1_y, foot_z0), op='new', body='foot1', name='-Y shell foot')
    model.box('foot1_clip', 40.0, 20.0, 20.0, at=(x_back + 20.0, foot1_y, (foot_z0 + foot_z1) / 2), op='intersect', body='foot1', name='Clip foot at the back face')
    model.combine('join_foot1', 'bulkhead', ['foot1'], op='join', name='Join -Y foot')
    model.box('leg', foot2_x - x_back, leg_y1 - leg_y0, foot_z1 - foot_z0, at=((x_back + foot2_x) / 2, (leg_y0 + leg_y1) / 2, (foot_z0 + foot_z1) / 2), op='join', body='bulkhead', name='Leg to the +Y foot')
    model.box('pad', pad_x1 - (x_front - 0.01), pad_y1 - pad_y0, top_z - pad_z0, at=((x_front - 0.01 + pad_x1) / 2, (pad_y0 + pad_y1) / 2, (pad_z0 + top_z) / 2), op='join', body='bulkhead', name='Deck screw pad')
    model.box('corner', flange_x1 - x_back, corner_y1 - corner_y0, foot_z0 - corner_z0, at=((x_back + flange_x1) / 2, (corner_y0 + corner_y1) / 2, (corner_z0 + foot_z0) / 2), op='join', body='bulkhead', name='Corner block')
    model.plane('plane_ring', on='YZ', offset=x_front - 0.01, name='Ring pad back face')
    with model.sketch('sketch_ring', on='plane_ring', name='Ring pad profile') as s:
        _draw_profile(s, 'r')
    model.extrude('ring', 'sketch_ring', distance=ring_x1 - (x_front - 0.01), op='new', body='ring', name='Ring pad slab')
    model.cylinder('ring_disc', ring_r, ring_x1 - ring_x0, at=(ring_x0, 0.0, roll_axis_z), direction=(1, 0, 0), op='intersect', body='ring', name='Ring pad disc')
    model.combine('join_ring', 'bulkhead', ['ring'], op='join', name='Join ring pad')
    model.cylinder('bore', bore_r, bore_x1 - (x_back - 3.0), at=(x_back - 3.0, 0.0, roll_axis_z), direction=(1, 0, 0), op='cut', body='bulkhead', name='F688 seat')
    model.cylinder('flange', flange_r, flange_x1 - bore_x1, at=(bore_x1, 0.0, roll_axis_z), direction=(1, 0, 0), op='cut', body='bulkhead', name='F688 flange counterbore')
    model.cylinder('race_clear', race_clear_r, race_clear_x1 - race_clear_x0, at=(race_clear_x0, 0.0, roll_axis_z), direction=(1, 0, 0), op='cut', body='bulkhead', name='Inner race clearance')
    model.cylinder('bkhd_lo', m3_clr / 2, x_front - x_back + 2.0, at=(x_back - 1.0, bkhd_y, bkhd_z_lo), direction=(1, 0, 0), op='cut', body='bulkhead', name='Lower M3 clearance')
    model.cylinder('bkhd_lo_cb', cbore_r, cbore_depth + 20.0, at=(x_front - cbore_depth, bkhd_y, bkhd_z_lo), direction=(1, 0, 0), op='cut', body='bulkhead', name='Lower button-head counterbore')
    model.cylinder('bkhd_hi', m3_clr / 2, x_front - x_back + 2.0, at=(x_back - 1.0, bkhd_y, bkhd_z_hi), direction=(1, 0, 0), op='cut', body='bulkhead', name='Upper M3 clearance')
    model.cylinder('bkhd_hi_cb', cbore_r, cbore_depth + 20.0, at=(x_front - cbore_depth, bkhd_y, bkhd_z_hi), direction=(1, 0, 0), op='cut', body='bulkhead', name='Upper button-head counterbore')
    model.cylinder('foot2', foot_r, foot_z1 - foot_z0, at=(foot2_x, foot2_y, foot_z0), op='new', body='foot2', name='+Y shell foot')
    model.box('foot2_clip', 40.0, 20.0, 20.0, at=(x_back + 20.0, foot2_y, (foot_z0 + foot_z1) / 2), op='intersect', body='foot2', name='Clip foot at the back face')
    model.combine('join_foot2', 'bulkhead', ['foot2'], op='join', name='Join +Y foot')
    model.add('sft_foot1', self_forming_hole(foot1_x, foot1_y, foot_z0, foot_z1), op='cut', body='bulkhead', name='-Y foot self-forming M3 hole')
    model.add('sft_foot2', self_forming_hole(foot2_x, foot2_y, foot_z0, foot_z1), op='cut', body='bulkhead', name='+Y foot self-forming M3 hole')
    model.add('sft_deck', self_forming_hole(deck_x, deck_y, top_z, top_z - deck_depth, taper=deck_taper), op='cut', body='bulkhead', name='Deck screw self-forming M3 hole')
    model.material('pla', color='#8a8a90')
    # The shifted speaker's underside magnet enters the plate by 1 mm.
    # Clear it by 0.25 mm at the back and retain 5.25 mm of plate thickness.
    model.box('stemma_magnet_recess', 2.0, 14.4, 14.4,
              at=(59.25, -18.41, 300.56), op='cut', body='bulkhead',
              name='Shifted speaker magnet clearance')
    # Remove only the front ring lip above the PCB seating plane. The
    # bearing seat/flange depths and the two speaker screw seats stay intact.
    model.box('stemma_pcb_corner_relief', 2.0, 6.8, 6.4,
              at=(61.0, -5.5, 291.3), op='cut', body='bulkhead',
              name='Shifted speaker PCB corner clearance')
    model.box('shell_foot_corner_relief', 1.2, 1.2, 1.6,
              at=(60.6, 8.15, 273.3), op='cut', body='bulkhead',
              name='Bottom shell clearance at foot corner')
    result = model.result()
    return to_build123d(model, 'roll_front_bulkhead_native')


@step(out="../../STEP/parts/roll_front_bulkhead_native.step")
def roll_front_bulkhead_native():
    return make()


if __name__ == "__main__":
    roll_front_bulkhead_native()
