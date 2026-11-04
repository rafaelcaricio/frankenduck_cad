from __future__ import annotations
from cadgen import step


def make_original():
    from lib.cadquery_adapter import to_build123d
    import math
    import cadquery as cq
    SIDE = 1
    cx = -42.899
    knee_z = 70.675
    ankle_z = 3.975
    half_w = 13.5
    bot_z = -0.525
    bot_corner_r = 4.0
    bulge_r = 8.633
    bulge_cz = 7.457
    y_core = 48.88
    y_top = 51.98
    y_top_low = 51.88
    y_neck = 51.28
    neck_z0 = 41.1
    neck_z1 = 57.1
    ramp_z1 = 60.5
    fl_y = 47.875
    fl_y_up = 48.18
    fl_split_z = 44.5
    sp_y = 44.82
    sp_y_mid = 46.35
    sp_hw = 8.625
    sp_hw_up = 8.22
    sp_head_r = 11.83
    sp_neck_r = 3.0
    sp_draft = 33.4
    sp_top_r = 1.24
    sp_base_r = 1.28
    sp_draft_up = 19.6
    sp_top_r_up = 1.37
    sp_base_r_up = 3.32
    end_z_mid = 24.58
    end_slope = 0.784
    boss_r = 3.9
    boss_y = 40.66
    boss_ch = 0.2
    collar_r = 4.75
    collar_y = 46.61
    collar_blend_r = 2.1
    boss_hole_r = 0.6
    boss_hole_dx = 1.8
    boss_hole_y = 44.71
    ring_ro = 4.8
    ring_y = 53.18
    bore_r = 3.2
    bore_y = 49.23
    knee_bore_y = 48.78
    horn_bolt = 7.0
    horn_hole_r = 1.7
    horn_cb_r = 3.4
    horn_cb_y = 47.98
    srv_dx = 10.25
    srv_hole_r = 1.1
    srv_z_low = 12.275
    srv_z_mid = 36.725
    srv_cb_r = 2.55
    srv_z_up = 46.725
    srv_up_y = 50.33
    win_hw = 10.5
    win_z0 = 14.725
    win_z1 = 22.975
    win_r = 2.5
    slot_hw = 3.375
    slot_z0 = 41.65
    slot_z1 = 55.15
    slot_r = 2.04
    pk_hw = 9.5
    pk_z0 = 21.0
    pk_y = 49.78
    pk_ear_r = 3.25
    strip_z0 = 41.0
    strip_z1 = 44.03
    strip_y = 49.47
    strip_corner_r = 1.9
    strip_corner_dx = 10.8
    strip_corner_z = 44.35
    lip_x0 = -47.62
    lip_x1 = -38.17
    lip_r = 2.71
    lip_y = 49.95
    
    def side(s):
        return s.mirror('XZ') if SIDE < 0 else s
    
    def prism_xz(profile, y0, y1):
        return profile(cq.Workplane('XZ', origin=(0, y1, 0))).extrude(y1 - y0)
    
    def rr_prism(x0, x1, z0, z1, r, y0, y1):
        b = cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
        return b.edges('|Y').fillet(r)
    
    def outline(w):
        xl, xr = (cx - half_w, cx + half_w)
        bx = math.sqrt(bulge_r ** 2 - (bulge_cz - bot_z) ** 2)
        w = w.moveTo(xl, knee_z).lineTo(xl, bot_z + bot_corner_r)
        w = w.radiusArc((xl + bot_corner_r, bot_z), -bot_corner_r)
        w = w.lineTo(cx - bx, bot_z).threePointArc((cx, bulge_cz - bulge_r), (cx + bx, bot_z))
        w = w.lineTo(xr - bot_corner_r, bot_z).radiusArc((xr, bot_z + bot_corner_r), -bot_corner_r)
        w = w.lineTo(xr, knee_z).threePointArc((cx, knee_z + half_w), (xl, knee_z))
        return w.close()
    
    def spine_wire(y, z_low, hw0, draft):
        d = (y - sp_y_mid) * math.tan(math.radians(draft))
        hw, R, rn = (hw0 + d, sp_head_r + d, sp_neck_r - d)
        zf = knee_z - math.sqrt((R + rn) ** 2 - (hw + rn) ** 2)
        pts = {}
        for s in (1, -1):
            F = (cx + s * (hw + rn), zf)
            L = math.hypot(F[0] - cx, F[1] - knee_z)
            P = (cx + (F[0] - cx) * R / L, knee_z + (F[1] - knee_z) * R / L)
            a0 = math.atan2(0, -s)
            a1 = math.atan2(knee_z - F[1], cx - F[0])
            am = (a0 + a1) / 2
            pts[s] = (F, P, (F[0] + rn * math.cos(am), F[1] + rn * math.sin(am)))
        wp = cq.Workplane('XZ', origin=(0, y, 0))
        w = wp.moveTo(cx - hw, z_low).lineTo(cx + hw, z_low).lineTo(cx + hw, zf)
        w = w.threePointArc(pts[1][2], pts[1][1]).threePointArc((cx, knee_z + R), pts[-1][1])
        w = w.threePointArc(pts[-1][2], (cx - hw, zf)).close()
        return w.wires().val()
    sil = prism_xz(outline, 30.0, 60.0)
    body = prism_xz(outline, y_core, y_top)
    plus_cut = cq.Workplane('YZ', origin=(cx - 20, 0, 0)).polyline([(y_top_low, -5), (y_top_low, neck_z0), (y_neck, neck_z0), (y_neck, neck_z1), (y_top, ramp_z1), (60, ramp_z1), (60, -5)]).close().extrude(40)
    body = body.cut(plus_cut)
    zb = lambda y: end_z_mid - end_slope * (y - sp_y_mid)
    end_cut = cq.Workplane('YZ', origin=(cx - 20, 0, 0)).polyline([(30, zb(30)), (y_core + 1, zb(y_core + 1)), (y_core + 1, -10), (30, -10)]).close().extrude(40)
    
    def relief_part(base_y, hw0, draft, r_top, r_base):
        flange = prism_xz(outline, base_y, y_core + 0.5)
        spine = cq.Solid.makeLoft([spine_wire(sp_y, 10.0, hw0, draft), spine_wire(y_core + 0.5, 10.0, hw0, draft)], True)
        rel = flange.union(cq.Workplane().add(spine).intersect(sil)).cut(end_cut)
        rel = rel.faces('<Y').edges().fillet(r_top)
    
        def inside(p):
            d = min(p.x - (cx - half_w), cx + half_w - p.x)
            if p.z > knee_z:
                d = min(d, half_w - math.hypot(p.x - cx, p.z - knee_z))
            return d
    
        def base_edge(e):
            pts = [e.positionAt(t) for t in (0.1, 0.5, 0.9)]
            return abs(pts[1].y - base_y) < 0.01 and min((inside(q) for q in pts)) > 0.2 and (pts[1].z > 24.0)
        return rel.edges().filter(base_edge).fillet(r_base)
    below = cq.Workplane('XY').box(100, 100, 100).translate((cx, 50, fl_split_z - 50))
    above = cq.Workplane('XY').box(100, 100, 100).translate((cx, 50, fl_split_z + 50))
    body = body.union(relief_part(fl_y, sp_hw, sp_draft, sp_top_r, sp_base_r).intersect(below))
    body = body.union(relief_part(fl_y_up, sp_hw_up, sp_draft_up, sp_top_r_up, sp_base_r_up).intersect(above))
    bp = [(0, boss_y), (boss_r - boss_ch, boss_y), (boss_r, boss_y + boss_ch), (boss_r, collar_y), (collar_r, collar_y), (collar_r, collar_y + (y_core - collar_y) - collar_blend_r)]
    boss = cq.Workplane('XY').polyline(bp).radiusArc((collar_r + collar_blend_r, y_core), collar_blend_r).lineTo(collar_r + collar_blend_r, y_core + 0.5).lineTo(0, y_core + 0.5).close().revolve(360, (0, 0, 0), (0, 1, 0))
    boss = boss.translate((cx, 0, ankle_z)).intersect(sil)
    body = body.union(boss)
    body = body.union(cq.Workplane('XZ', origin=(0, ring_y, 0)).center(cx, ankle_z).circle(ring_ro).extrude(ring_y - y_top_low + 0.2))
    body = body.cut(rr_prism(cx - win_hw, cx + win_hw, win_z0, win_z1, win_r, 30, 60))
    body = body.cut(rr_prism(cx - slot_hw, cx + slot_hw, slot_z0, slot_z1, slot_r, 30, 60))
    horn = [(cx - horn_bolt, knee_z), (cx + horn_bolt, knee_z), (cx, knee_z - horn_bolt), (cx, knee_z + horn_bolt)]
    body = body.cut(prism_xz(lambda w: w.pushPoints(horn).circle(horn_hole_r), 30, 60))
    body = body.cut(prism_xz(lambda w: w.pushPoints(horn).circle(horn_cb_r), 30, horn_cb_y))
    srv = [(cx + s * srv_dx, z) for s in (1, -1) for z in (srv_z_low, srv_z_mid)]
    body = body.cut(prism_xz(lambda w: w.pushPoints(srv).circle(srv_hole_r), 30, 60))
    body = body.cut(prism_xz(lambda w: w.pushPoints([(cx + s * srv_dx, srv_z_mid) for s in (1, -1)]).circle(srv_cb_r), 30, fl_y))
    body = body.cut(prism_xz(lambda w: w.center(cx, knee_z).circle(bore_r), knee_bore_y, 60))
    body = body.cut(prism_xz(lambda w: w.center(cx, ankle_z).circle(bore_r), bore_y, 60))
    body = body.cut(prism_xz(lambda w: w.pushPoints([(cx - boss_hole_dx, ankle_z), (cx + boss_hole_dx, ankle_z)]).circle(boss_hole_r), 30, boss_hole_y))
    body = body.cut(prism_xz(lambda w: w.pushPoints([(cx + s * srv_dx, srv_z_up) for s in (1, -1)]).circle(srv_hole_r), srv_up_y, 60))
    ears = prism_xz(lambda w: w.pushPoints([(cx + s * srv_dx, srv_z_mid) for s in (1, -1)]).circle(pk_ear_r), 30, 60)
    corners = prism_xz(lambda w: w.pushPoints([(cx + s * strip_corner_dx, strip_corner_z) for s in (1, -1)]).circle(strip_corner_r), 30, 60)
    pocket = prism_xz(lambda w: w.center(cx, (pk_z0 + strip_z1) / 2).rect(2 * pk_hw, strip_z1 - pk_z0), pk_y, 60)
    body = body.cut(pocket.cut(ears).cut(corners))
    strip = prism_xz(lambda w: w.center(cx, (strip_z0 + strip_z1) / 2).rect(2 * pk_hw, strip_z1 - strip_z0), strip_y, 60)
    body = body.cut(strip.cut(corners))
    lip = rr_prism(lip_x0, lip_x1, strip_z0, slot_z1, lip_r, lip_y, 60)
    body = body.cut(lip)
    from lib.cadquery_adapter import Model
    model = Model()
    model.add('printed_body', side(body))
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'leg_left_native')


def make():
    from lib.bearing_revision.factories import revised
    from lib.viewer_palette import paint_original
    return paint_original(revised('leg_left_native'), 'leg_left')


@step(out="../../STEP/parts/leg_left_native.step")
def leg_left_native():
    return make()


if __name__ == "__main__":
    leg_left_native()
