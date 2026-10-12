from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    """Head frame carrying the jaw and head electronics. World millimetres, Z up."""
    import math
    import cadquery as cq
    from lib.cadquery_adapter import Model
    z0_roll = 280.58
    roll_back_x = -20.265
    bed_x = -25.27
    frame_top = 300.6
    bar_top = 267.9
    jaw_cx = 35.24
    jaw_cz = 284.89
    jaw_back_y = 17.8
    jaw_recess_y = 19.0
    shell_boss_z = 274.0
    rear_foot_x = -21.6
    rear_foot_z = 261.8
    rear_foot_r = 4.1
    foot_r = 4.2
    foot_top = 283.0
    boss_r = 3.5
    boss_pocket = 3.0
    cover_xl = 9.4
    cover_y0 = -30.0
    cover_d = 2.2
    plug_xl = 9.5
    plug_y1 = -12.2
    plug_y0_ry = -17.7
    plug_y1_ry = -11.2
    f688_bore_r = 8.275
    passage_r = 9.3
    retainer_r = 10.8
    retainer_deg = -30.0
    m2_clr = 2.4
    m2_pilot = 1.6
    washer_r = 2.6
    sft_d = 3.0
    sft_margin = 0.15
    sft_ridge_h = 0.4
    sft_taper = 3.0
    sft_taper_short = 2.0
    sft_ridges = 3
    flange_hole_depth = 5.5
    wallb_hole_depth = 4.5
    foot_hole_depth = 9.0
    rear_hole_depth = 4.5
    
    def rounded(pts, r):
        """Round every corner of a closed polygon (radius per corner, 0 = sharp), 10-degree segments (source round_poly)."""
        n = len(pts)
        rr = r if isinstance(r, (list, tuple)) else [r] * n
        out = []
        for i in range(n):
            p0, p1, p2 = (pts[i - 1], pts[i], pts[(i + 1) % n])
            if rr[i] <= 0:
                out.append(tuple(p1))
                continue
            ax, ay = (p0[0] - p1[0], p0[1] - p1[1])
            bx, by = (p2[0] - p1[0], p2[1] - p1[1])
            la, lb = (math.hypot(ax, ay), math.hypot(bx, by))
            ax, ay, bx, by = (ax / la, ay / la, bx / lb, by / lb)
            ang = math.acos(max(-1.0, min(1.0, ax * bx + ay * by)))
            if ang < 0.001 or abs(ang - math.pi) < 0.001:
                out.append(tuple(p1))
                continue
            t = min(rr[i] / math.tan(ang / 2), 0.45 * la, 0.45 * lb)
            rad = t * math.tan(ang / 2)
            q0 = (p1[0] + ax * t, p1[1] + ay * t)
            q1 = (p1[0] + bx * t, p1[1] + by * t)
            sx, sy = (ax + bx, ay + by)
            ls = math.hypot(sx, sy)
            k = rad / math.sin(ang / 2) / ls
            c = (p1[0] + sx * k, p1[1] + sy * k)
            a0 = math.atan2(q0[1] - c[1], q0[0] - c[0])
            a1 = math.atan2(q1[1] - c[1], q1[0] - c[0])
            da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
            m = max(2, int(abs(da) / math.radians(10)) + 1)
            for j in range(m + 1):
                aa = a0 + da * j / m
                out.append((c[0] + rad * math.cos(aa), c[1] + rad * math.sin(aa)))
        clean = [out[0]]
        for p in out[1:]:
            if math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > 0.0001:
                clean.append(p)
        if math.hypot(clean[0][0] - clean[-1][0], clean[0][1] - clean[-1][1]) < 0.0001:
            clean.pop()
        return clean
    
    def outline(s, pts):
        """Closed polygon as sketch line segments (one line per edge)."""
        for i in range(len(pts)):
            s.line(f'e{i}', tuple(pts[i]), tuple(pts[(i + 1) % len(pts)]))
    jaw_o = (35.235, 53.0, 284.635)
    jaw_rot = ((-0.1045, -0.9945, 0.0), (0.0025, -0.0003, -1.0), (0.9945, -0.1045, 0.0025))
    
    def jaw_world(xl, yl, zl):
        return tuple((jaw_o[i] + jaw_rot[i][0] * xl + jaw_rot[i][1] * yl + jaw_rot[i][2] * zl for i in range(3)))
    
    def jaw_xz(pts_l, zl=35.2):
        return [(jaw_world(a, b, zl)[0], jaw_world(a, b, zl)[2]) for a, b in pts_l]
    
    def roll_yz(xl, yl):
        """roll servo local (x_l, y_l) -> world (y, z)"""
        return (yl, z0_roll + xl)
    
    def self_forming_hole(face, axis, depth, taper, d=sft_d, margin=sft_margin, ridge_h=sft_ridge_h, ridges=sft_ridges, overshoot=2.0):
        """Cutting tool for a self-forming thread hole (Sanladerer) entering at point `face` along unit vector `axis`.
    
        Bore radius r = (d + margin) / 2; each ridge is what a large cylinder (radius R = r + 5) leaves where it overlaps
        the bore by ridge_h, so the ridge tips sit at r - ridge_h. Over the first `taper` mm from the face the overlap
        grows from 0 to ridge_h. The bore also runs `overshoot` mm outside the entry face so the cut opens cleanly.
        Same construction as top_head_shell_native, generalised from +-Z to any axis.
        """
        u = cq.Vector(*axis).normalized()
        e1 = cq.Vector(0, 0, 1).cross(u) if abs(u.z) < 0.9 else cq.Vector(1, 0, 0)
        e1 = e1.normalized()
        e2 = u.cross(e1).normalized()
        p = cq.Vector(*face)
        r = (d + margin) / 2.0
        big_r = r + 5.0
        off = big_r + r - ridge_h
        p_out = p - u * overshoot
        tool = cq.Solid.makeCylinder(r, depth + overshoot, p_out, u)
        for i in range(int(ridges)):
            a = 2.0 * math.pi * i / ridges
            c = e1 * (off * math.cos(a)) + e2 * (off * math.sin(a))
            entry = cq.Solid.makeCylinder(big_r - ridge_h, overshoot, p_out + c, u)
            ramp = cq.Solid.makeCone(big_r - ridge_h, big_r, taper, p + c, u)
            full = cq.Solid.makeCylinder(big_r, depth - taper, p + c + u * taper, u)
            tool = tool.cut(entry.fuse(ramp).fuse(full).clean())
        return tool.clean()
    model = Model()
    model.plane('pl_bed', on='YZ', offset=bed_x, name='Print bed face x -25.27')
    with model.sketch('sk_bulkhead', on='pl_bed', name='Rear bulkhead profile') as s:
        outline(s, rounded([(-34.0, 264.0), (14.5, 264.0), (14.5, frame_top), (-22.5, frame_top), (-26.0, 298.5), (-30.0, 295.0), (-34.0, 289.0), (-36.4, 285.0), (-36.4, 267.0)], [2, 2, 0, 0, 2, 2, 2, 2, 2]))
    model.extrude('bulkhead', 'sk_bulkhead', distance=roll_back_x - bed_x, op='new', body='frame', name='Rear bulkhead')
    with model.sketch('sk_flange', on='pl_bed', name='Bulkhead top flange profile') as s:
        outline(s, rounded([(-22.5, 293.5), (14.5, 293.5), (14.5, frame_top), (-22.5, frame_top)], [1.5, 0, 0, 0]))
    model.extrude('flange', 'sk_flange', distance=9.0, op='join', body='frame', name='Bulkhead top flange')
    model.plane('pl_sw', on='XZ', offset=35.5, name='Saddle -Y wall outer face y -35.5')
    with model.sketch('sk_sw', on='pl_sw', name='Saddle -Y wall profile') as s:
        outline(s, rounded([(-20.27, 264.0), (-10.0, 264.0), (0.0, 274.0), (7.0, 274.0), (7.0, 296.0), (-8.0, 296.0), (-16.0, 291.0), (-20.27, 286.0)], [0, 3, 0, 0, 0, 3, 3, 0]))
    model.extrude('saddle_my', 'sk_sw', distance=3.1, op='join', body='frame', name='Saddle -Y wall')
    model.plane('pl_nw', on='XZ', offset=-14.5, name='Saddle +Y wall face y 14.5')
    with model.sketch('sk_nw', on='pl_nw', name='Saddle +Y wall profile') as s:
        outline(s, rounded([(bed_x, 264.0), (1.5, 264.0), (10.2, 272.5), (10.2, frame_top), (bed_x, frame_top)], [0, 3, 0, 0, 0]))
    model.extrude('saddle_py', 'sk_nw', distance=4.0, op='join', body='frame', name='Saddle +Y wall')
    model.box('bottom_bar', 17.27, 47.0, bar_top - 264.0, at=(-11.635, -12.5, (264.0 + bar_top) / 2), op='join', body='frame', name='Bottom bar under the roll servo')
    model.plane('pl_wy', on='XY', offset=274.6, name='Wall B base z 274.6')
    with model.sketch('sk_wy', on='pl_wy', name='-Y jog and wall B plan') as s:
        outline(s, rounded([(-0.5, -35.5), (6.0, -35.5), (13.5, -43.0), (53.0, -43.0), (53.0, -48.0), (14.1, -48.0), (4.7, -38.6), (-0.5, -38.6)], [0, 2, 2, 0, 0, 2, 2, 0]))
    model.extrude('wall_b', 'sk_wy', distance=24.9, op='new', body='wallb', name='-Y jog and wall B')
    model.plane('pl_wout', on='YZ', offset=8.0, name='Wall B slope start x 8')
    with model.sketch('sk_wout', on='pl_wout', name='Wall B outer slope') as s:
        outline(s, [(-70.0, 284.0), (-47.5, 284.0), (-43.6, 299.5), (-43.6, 340.0), (-70.0, 340.0)])
    model.extrude('wall_b_out', 'sk_wout', distance=17.9, op='cut', body='wallb', name='Wall B outer face follows the top shell')
    model.plane('pl_win', on='YZ', offset=12.0, name='Wall B inner slope start x 12')
    with model.sketch('sk_win', on='pl_win', name='Wall B inner slope') as s:
        outline(s, [(-47.5, 274.6), (-43.0, 274.6), (-43.0, 284.0), (-39.6, 299.5), (-43.6, 299.5), (-47.5, 284.0)])
    model.extrude('wall_b_in', 'sk_win', distance=13.9, op='join', body='wallb', name='Wall B inner face moved in')
    model.combine('join_wallb', 'frame', ['wallb'], op='join', name='Join wall B')
    model.cylinder('seat_boss', 12.0, 4.0, at=(jaw_cx, -53.0, jaw_cz), direction=(0, 1, 0), op='new', body='seat', name='Jaw F688 seat boss r12')
    model.cylinder('seat_ring', 14.0, 3.0, at=(jaw_cx, -49.0, jaw_cz), direction=(0, 1, 0), op='join', body='seat', name='Seat ring r14 tying the boss to wall B')
    model.cylinder('retainer_lobe', 2.6, 5.1, at=(jaw_cx + retainer_r * math.cos(math.radians(retainer_deg)), -53.0, jaw_cz + retainer_r * math.sin(math.radians(retainer_deg))), direction=(0, 1, 0), op='join', body='seat', name='Lobe round the retainer pilot')
    model.box('seat_flat', 80.0, 20.0, 400.0 - 274.6, at=(40.0, -50.0, (274.6 + 400.0) / 2), op='intersect', body='seat', name='Seat boss flat at z 274.6')
    model.combine('join_seat', 'frame', ['seat'], op='join', name='Join jaw seat boss')
    model.box('end_block', 4.5, 9.0, 24.6, at=(50.75, -44.5, (274.9 + 299.5) / 2), op='join', body='frame', name='Wall B front end block')
    model.cylinder('foot_my', foot_r, foot_top - shell_boss_z, at=(4.18, -40.5, shell_boss_z), op='join', body='frame', name='Rear -Y shell foot')
    model.plane('pl_shell', on='XY', offset=shell_boss_z, name='Shell boss tops z 274')
    with model.sketch('sk_foot_my_web', on='pl_shell', name='Rear -Y foot web plan') as s:
        outline(s, [(-5.0, -38.0), (8.4, -38.0), (8.4, -40.5), (4.18, -44.7), (1.1, -44.7)])
    model.extrude('foot_my_web', 'sk_foot_my_web', distance=foot_top - shell_boss_z, op='join', body='frame', name='Rear -Y foot web')
    model.plane('pl_jn', on='XY', offset=272.5, name='+Y jog base z 272.5')
    with model.sketch('sk_jn', on='pl_jn', name='+Y jog plan') as s:
        outline(s, [(8.0, 10.5), (12.5, 15.0), (14.4, 15.0), (14.4, jaw_recess_y), (11.0, jaw_recess_y), (8.0, 16.0)])
    model.extrude('jog_py', 'sk_jn', distance=frame_top - 272.5, op='join', body='frame', name='+Y jog')
    model.box('cp_r', 9.5, jaw_recess_y - 15.0, 28.1, at=(19.15, 17.0, 286.55), op='join', body='frame', name='Cradle zone R')
    model.box('cp_s1', 16.6, jaw_recess_y - 15.5, 2.0, at=(32.2, 17.25, 275.0), op='join', body='frame', name='Cradle zone S1')
    model.box('cp_s2', 16.6, jaw_recess_y - 14.5, 9.5, at=(32.2, 16.75, 280.75), op='join', body='frame', name='Cradle zone S2')
    model.box('cp_s3', 5.1, jaw_recess_y - 15.0, 7.0, at=(26.45, 17.0, 289.0), op='join', body='frame', name='Cradle zone S3')
    model.box('cp_s4', 11.5, jaw_recess_y - 16.0, 7.0, at=(34.75, 17.5, 289.0), op='join', body='frame', name='Cradle zone S4')
    model.box('cp_s5', 16.6, jaw_recess_y - 14.5, 8.1, at=(32.2, 16.75, 296.55), op='join', body='frame', name='Cradle zone S5')
    model.box('cp_m1', 5.9, jaw_back_y - 15.5, 2.0, at=(43.45, 16.65, 275.0), op='join', body='frame', name='Cradle zone M1')
    model.box('cp_m2', 5.9, jaw_back_y - 14.5, 24.6, at=(43.45, 16.15, 288.3), op='join', body='frame', name='Cradle zone M2')
    model.box('cp_c1', 1.6, jaw_back_y - 14.5, 26.6, at=(47.2, 16.15, 287.3), op='join', body='frame', name='Cradle zone C1')
    model.plane('pl_cp_c', on='XZ', offset=-jaw_back_y, name='Jaw servo lands y 17.8')
    with model.sketch('sk_cp_c', on='pl_cp_c', name='Cradle zone C/F profile') as s:
        outline(s, rounded([(48.0, 267.0), (59.6, 267.0), (59.6, 274.2), (65.8, 274.2), (65.8, 275.1), (74.0, 275.1), (74.0, 306.0), (50.5, 306.0), (50.5, frame_top), (48.0, frame_top)], [0, 0, 0, 0, 0, 0, 2, 2, 1.5, 0]))
    model.extrude('cp_c2', 'sk_cp_c', distance=jaw_back_y - 14.8, op='join', body='frame', name='Cradle zone C/F with chord and raised top bar')
    model.box('cradle_flange', 33.6, 5.6, 3.0, at=(31.2, 12.8, 299.1), op='join', body='frame', name='Cradle top flange')
    model.cylinder('foot_py', foot_r, foot_top - shell_boss_z, at=(8.91, 29.65, shell_boss_z), op='join', body='frame', name='Rear +Y shell foot')
    with model.sketch('sk_foot_py_arm', on='pl_shell', name='Rear +Y foot arm plan') as s:
        outline(s, [(-10.9, 12.0), (13.1, 12.0), (13.1, 29.65), (8.95, 33.85)])
    model.extrude('foot_py_arm', 'sk_foot_py_arm', distance=foot_top - shell_boss_z, op='join', body='frame', name='Rear +Y foot arm (45-degree print)')
    model.cylinder('rear_foot', rear_foot_r, 264.5 - rear_foot_z, at=(rear_foot_x, 0.0, rear_foot_z), op='new', body='rfoot', name='Rear-centre shell foot')
    model.box('rear_foot_clip', 25.27, 20.0, 20.0, at=(bed_x / 2, 0.0, 260.0), op='intersect', body='rfoot', name='Clip rear foot at the print bed face')
    model.combine('join_rfoot', 'frame', ['rfoot'], op='join', name='Join rear-centre foot')
    import numpy as np
    from scipy.spatial import ConvexHull
    corner_rear = [(-30.0, 264.0), (-36.4, 264.0), (-36.4, 285.0), (-30.0, 295.0)]
    corner_front = [(-35.5, 264.0), (-38.6, 264.0), (-38.6, 293.5), (-35.5, 293.5)]
    corner_points = np.array([(bed_x, y, z) for y, z in corner_rear] + [(-12.0, y, z) for y, z in corner_front] + [(-20.27, -38.6, 264.0), (-20.27, -38.6, 289.0)])
    corner_hull = ConvexHull(corner_points)
    corner_faces = []
    for corner_ids, corner_eq in zip(corner_hull.simplices, corner_hull.equations):
        corner_triangle = corner_points[corner_ids]
        if np.dot(np.cross(corner_triangle[1] - corner_triangle[0], corner_triangle[2] - corner_triangle[0]), corner_eq[:3]) < 0:
            corner_triangle = corner_triangle[::-1]
        corner_wire = cq.Wire.makePolygon([cq.Vector(*v) for v in corner_triangle], close=True)
        corner_faces.append(cq.Face.makeFromWires(corner_wire))
    corner_solid = cq.Solid.makeSolid(cq.Shell.makeShell(corner_faces)).clean()
    corner_servo_void = cq.Solid.makeBox(40.0, 100.0, 80.0, cq.Vector(roll_back_x, -35.5, 250.0))
    model.add('saddle_bulkhead_blend', corner_solid.cut(corner_servo_void).clean(), op='join', body='frame', name='Single filled bulkhead-saddle corner')
    model.cylinder('roll_boss_pocket', boss_r, boss_pocket + 1.0, at=(roll_back_x - boss_pocket, 0.0, z0_roll), direction=(1, 0, 0), op='cut', body='frame', name='Roll servo passive-boss pocket')
    model.plane('pl_cover', on='YZ', offset=roll_back_x - cover_d, name='Roll cover pocket floor')
    with model.sketch('sk_cover', on='pl_cover', name='Roll raised-cover pocket') as s:
        outline(s, rounded([roll_yz(-cover_xl, cover_y0), roll_yz(-cover_xl, plug_y1), roll_yz(cover_xl, plug_y1), roll_yz(cover_xl, cover_y0)], 1.0))
    model.extrude('roll_cover_pocket', 'sk_cover', distance=cover_d + 1.0, op='cut', body='frame', name='Roll servo raised-cover pocket')
    model.plane('pl_x30', on='YZ', offset=-30.0, name='Plane x -30')
    with model.sketch('sk_roll_plug', on='pl_x30', name='Roll plug window') as s:
        outline(s, rounded([roll_yz(-plug_xl, plug_y0_ry), roll_yz(-plug_xl, plug_y1_ry), roll_yz(plug_xl, plug_y1_ry), roll_yz(plug_xl, plug_y0_ry)], 1.0))
    model.extrude('roll_plug_window', 'sk_roll_plug', distance=14.0, op='cut', body='frame', name='Roll servo plug window')
    model.cylinder('roll_m2_a', m2_clr / 2, 15.0, at=(-30.0, -32.74, 270.32), direction=(1, 0, 0), op='cut', body='frame', name='Roll back M2 clearance')
    model.cylinder('roll_m2_b', m2_clr / 2, 15.0, at=(-30.0, -8.3, 270.31), direction=(1, 0, 0), op='cut', body='frame', name='Roll back M2 clearance')
    model.cylinder('roll_m2_c', m2_clr / 2, 15.0, at=(-30.0, -8.3, 290.85), direction=(1, 0, 0), op='cut', body='frame', name='Roll back M2 clearance')
    model.add('deck_hole_a', self_forming_hole((-20.77, -17.5, frame_top), (0, 0, -1), flange_hole_depth, sft_taper_short), op='cut', body='frame', name='Deck self-forming M3 hole')
    model.add('deck_hole_b', self_forming_hole((-20.77, 2.5, frame_top), (0, 0, -1), flange_hole_depth, sft_taper_short), op='cut', body='frame', name='Deck self-forming M3 hole')
    model.cylinder('f688_seat', f688_bore_r, 11.1, at=(jaw_cx, -60.0, jaw_cz), direction=(0, 1, 0), op='cut', body='frame', name='Jaw F688 seat bore')
    model.cylinder('f688_passage', passage_r, 6.1, at=(jaw_cx, -49.0, jaw_cz), direction=(0, 1, 0), op='cut', body='frame', name='F688 insertion passage')
    model.box('f688_passage_open', 2 * passage_r, 6.1, jaw_cz - 270.0, at=(jaw_cx, -45.95, (270.0 + jaw_cz) / 2), op='cut', body='frame', name='Passage open below')
    model.box('f688_band_corners', 28.2, 1.0, 6.5, at=(jaw_cx, -48.5, 273.25), op='cut', body='frame', name='Ring band bottom corners')
    model.cylinder('retainer_pilot', m2_pilot / 2, 4.4, at=(jaw_cx + retainer_r * math.cos(math.radians(retainer_deg)), -51.4, jaw_cz + retainer_r * math.sin(math.radians(retainer_deg))), direction=(0, 1, 0), op='cut', body='frame', name='F688 retainer M2 pilot')
    model.cylinder('retainer_pocket', washer_r, 5.1, at=(jaw_cx + retainer_r * math.cos(math.radians(retainer_deg)), -48.0, jaw_cz + retainer_r * math.sin(math.radians(retainer_deg))), direction=(0, 1, 0), op='cut', body='frame', name='Retainer washer and head pocket')
    model.add('wallb_hole_a', self_forming_hole((53.0, -45.3, 287.0), (-1, 0, 0), wallb_hole_depth, sft_taper_short), op='cut', body='frame', name='Front bulkhead self-forming M3 hole')
    model.add('wallb_hole_b', self_forming_hole((53.0, -45.3, 295.4), (-1, 0, 0), wallb_hole_depth, sft_taper_short), op='cut', body='frame', name='Front bulkhead self-forming M3 hole')
    model.add('foot_my_hole', self_forming_hole((4.18, -40.5, shell_boss_z), (0, 0, 1), foot_hole_depth, sft_taper), op='cut', body='frame', name='-Y shell foot self-forming M3 hole')
    model.add('foot_py_hole', self_forming_hole((8.91, 29.65, shell_boss_z), (0, 0, 1), foot_hole_depth, sft_taper), op='cut', body='frame', name='+Y shell foot self-forming M3 hole')
    model.cylinder('jaw_boss_hole', boss_r, 15.0, at=(jaw_world(0, 0, 36.5)[0], 10.0, jaw_world(0, 0, 36.5)[2]), direction=(0, 1, 0), op='cut', body='frame', name='Jaw servo passive-boss hole')
    model.plane('pl_y25', on='XZ', offset=-25.0, name='Plane y 25')
    with model.sketch('sk_jaw_win', on='pl_y25', name='Jaw plug + raised-cover window') as s:
        outline(s, rounded(jaw_xz([(-plug_xl, plug_y1), (plug_xl, plug_y1), (plug_xl, cover_y0), (-cover_xl, cover_y0)]), [0.5, 0.5, 1.5, 1.5]))
    model.extrude('jaw_window', 'sk_jaw_win', distance=15.0, op='cut', body='frame', name='Jaw servo plug + cover window')
    model.cylinder('jaw_m2_a', m2_clr / 2, 10.0, at=(jaw_world(10.26, -8.3, 35.2)[0], 10.0, jaw_world(10.26, -8.3, 35.2)[2]), direction=(0, 1, 0), op='cut', body='frame', name='Jaw back M2 clearance')
    model.cylinder('jaw_m2_b', m2_clr / 2, 10.0, at=(jaw_world(10.26, -32.74, 35.2)[0], 10.0, jaw_world(10.26, -32.74, 35.2)[2]), direction=(0, 1, 0), op='cut', body='frame', name='Jaw back M2 clearance')
    model.cylinder('jaw_m2_c', m2_clr / 2, 10.0, at=(jaw_world(-10.26, -32.74, 35.2)[0], 10.0, jaw_world(-10.26, -32.74, 35.2)[2]), direction=(0, 1, 0), op='cut', body='frame', name='Jaw back M2 clearance')
    model.box('bulkhead_foot_relief', 5.5, 2.7, 13.5, at=(63.25, 14.35, 276.75), op='cut', body='frame', name='Relief round the front bulkhead +Y foot')
    model.add('rear_foot_hole', self_forming_hole((rear_foot_x, 0.0, rear_foot_z), (0, 0, 1), rear_hole_depth, sft_taper_short), op='cut', body='frame', name='Rear-centre foot self-forming M3 hole')
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'head_frame_native')


@step(out="../../STEP/parts/head_frame_native.step")
def head_frame_native():
    return make()


if __name__ == "__main__":
    head_frame_native()
