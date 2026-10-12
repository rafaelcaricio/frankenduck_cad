from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    """Head bottom shell. World millimetres."""
    import cadquery as cq
    import numpy as np
    from scipy.interpolate import PchipInterpolator
    from lib.cadquery_adapter import Model
    plan_half_widths = [(-41.04, 0.0), (-40.38, 11.26), (-38.988, 19.097), (-36.236, 27.424), (-31.622, 35.366), (-25.634, 41.468), (-17.46, 46.473), (-7.235, 50.099), (8.098, 53.198), (23.086, 55.055), (56.273, 57.88), (112.89, 61.903), (114.01, 61.795), (114.85, 61.447), (115.58, 60.869), (116.06, 60.224), (116.57, 58.503)]
    flank_x0 = 56.273
    flank_x1 = 112.89
    x_rear = -40.9
    x_front = 116.57
    relief_depth = 4.0
    relief_in_x0 = 15.0
    relief_in_len = 11.0
    relief_out_x1 = 70.0
    relief_out_len = 8.0
    floor_z0 = 258.56
    floor_slope = 0.00748
    rear_curl_x = -30.0
    rear_curl_k = 0.035
    step_x0 = 25.0
    step_len = 10.0
    step_h = 2.5
    front_blend_x0 = 62.0
    front_blend_len = 8.0
    front_floor_z0 = 273.23
    front_floor_slope = -0.008726
    floor_t = 2.2951
    side_wall_t = 3.0
    inner_top = 280.0
    top_mid = 274.05
    top_end = 274.305
    corner_r = 5.5
    corner_r_min = 1.0
    corner_r_taper = 0.6
    corner_r_inner_shrink = 2.4
    inner_x0 = -38.5
    inner_x1 = 114.17
    station_step = 2.0
    chord_tol = 0.025
    seam_rebate_t = 2.4
    seam_rebate_z0 = 274.305
    rim1_in = 0.985
    rim1_out = 2.4
    rim1_z0 = 273.95
    rim1_z1 = 274.4
    rim2_in = 1.28
    rim2_out = 2.27
    rim2_z0 = 274.35
    rim2_z1 = 276.33
    rim_gap_x0 = 20.0
    rim_gap_x1 = 72.0
    rim_gap_y = 45.0
    rim_gap_z0 = 274.1
    rim_front_x0 = 112.5
    rim_front_z0 = 274.3
    front_floor_cut_x0 = 107.0
    front_floor_cut_z0 = 273.4247
    mouth_x0 = 69.0
    mouth_x1 = 107.0
    mouth_z0 = 273.28
    mouth_cut_z0 = 250.0
    skirt_t = 2.5
    skirt_z0 = 268.0
    skirt_x0 = 70.05
    below_x0 = 64.0
    below_z0 = 270.665
    front_deep_x0 = 113.95
    land_cx = 113.035
    land_w = 5.07
    land_h = 115.9
    land_r = 0.4
    land_z0 = 272.5
    land_z1 = 274.77478
    pocket_x0 = 107.0
    pocket_x1 = 110.5
    pocket_y = 59.4
    return_x0 = 65.0
    return_x1 = 72.0
    return_skirt_t = 2.2
    return_roof_t = 3.7
    return_roof_z0 = 272.7
    ap_cx = 34.0
    ap_w = 61.0
    ap_h = 79.0
    ap_r = 12.0
    yoke_cx = 34.0
    yoke_cy = -38.5
    yoke_w = 29.0
    yoke_h = 7.4
    yoke_r = 3.0
    yoke_z1 = 270.0
    harness_cx = -14.2
    harness_cy = 28.1
    harness_w = 8.5
    harness_h = 11.0
    harness_r = 2.5
    cut_z0 = 245.0
    cut_z1 = 290.0
    m3_r = 1.7
    cb_r = 3.2
    seat_x = -21.6
    seat_y = 0.0
    seat_pad_r = 4.1
    seat_pad_z0 = 260.0
    seat_pad_z1 = 261.8
    seat_cb_z1 = 260.05
    seat_clr_z1 = 280.0
    cl_cx = 8.9099907
    cl_cy = 36.5
    cl_w = 7.0
    cl_h = 18.0
    cl_r = 3.0
    cover_land_z0 = 260.0
    cover_land_z1 = 274.0
    cover_lands = [(8.9099907, 29.6500111), (63.285025, 11.1225061), (4.1849903, -40.500019), (56.8350166, -40.500019)]
    cover_land_r = 4.1
    cover_land_front_z0 = 261.54485
    cover_land_hole_z0 = 255.0
    cover_land_hole_z1 = 280.0
    mic_xs = [82.2999336, 93.6999248]
    mic_y = 22.0
    mic_r = 2.8
    mic_top = 278.30694
    mic_pilot_z0 = 271.8
    ubec_xs = [68.5, 95.5]
    ubec_y = -25.0
    ubec_r = 3.0
    ubec_top = 282.9
    ubec_pilot_z0 = 277.9
    sft_d = 2.0
    sft_margin = 0.15
    sft_ridge_h = 0.4
    sft_taper = 3.0
    sft_ridges = 3
    boss_lift = 2.0
    port_x = 87.99993
    port_y = 22.0
    port_r = 1.5
    port_z0 = 268.0
    port_z1 = 280.0
    cover_front = [(102.1, 52.65), (102.1, -52.65)]
    cover_front_r = 4.5
    cover_front_z0 = 274.0
    cover_front_z1 = 278.62
    cover_front_cb_z1 = 274.2
    cover_rear = [(-30.0, -26.0), (-1.5, 43.5)]
    post_r = 4.5
    post_z0 = 259.0
    post_z1 = 277.0
    post_webs = [(-35.0, -30.0, -28.0, -24.0, 274.0), (-6.0, 3.0, 43.5, 49.6, 276.0)]
    cover_rear_cb_z1 = 260.35
    envelope_z0 = 274.0
    envelope_top = 290.0
    post_clip_inset = 2.4
    hole_z0 = 250.0
    hole_z1 = 290.0
    big = 140.0
    tool_x1 = 120.0
    tool_y = 70.0
    window_z0 = 267.0
    tool_z1 = 280.0
    
    def smooth(u):
        u = min(max(u, 0.0), 1.0)
        return u * u * (3 - 2 * u)
    _pchip = PchipInterpolator([p[0] for p in plan_half_widths], [p[1] for p in plan_half_widths])
    _f0 = dict(plan_half_widths)[flank_x0]
    _f1 = dict(plan_half_widths)[flank_x1]
    
    def width(x):
        """Plan half width of the outer outline."""
        if flank_x0 <= x <= flank_x1:
            return _f0 + (x - flank_x0) * (_f1 - _f0) / (flank_x1 - flank_x0)
        return float(_pchip(x))
    
    def relief(x):
        return relief_depth * smooth((x - relief_in_x0) / relief_in_len) * smooth((relief_out_x1 - x) / relief_out_len)
    
    def base_z(x):
        """Outer underside height."""
        rear = floor_z0 + floor_slope * x + rear_curl_k * max(0.0, rear_curl_x - x) ** 2 + step_h * smooth((x - step_x0) / step_len)
        t = smooth((x - front_blend_x0) / front_blend_len)
        return rear * (1 - t) + (front_floor_z0 + front_floor_slope * x) * t
    
    def section(x, inner=False):
        """Closed rounded-U wire in the YZ plane at x."""
        w = width(x) - relief(x) - (side_wall_t if inner else 0.0)
        z = base_z(x) + (floor_t if inner else 0.0)
        top = inner_top if inner else max(z + floor_t, top_mid if 20 <= x <= 60 else top_end)
        r = min(corner_r, max(corner_r_min, (70 - x) * corner_r_taper)) if x > 60 else corner_r
        if inner:
            r = max(0.5, r - corner_r_inner_shrink)
        r = min(r, w * 0.45, max(0.15, (top - z) * 0.75))
        w = max(w, 0.15)
        p = [(x, -w, top), (x, -w, z + r), (x, -w + r, z), (x, w - r, z), (x, w, z + r), (x, w, top)]
        k = r / 2 ** 0.5
        return cq.Wire.assembleEdges([cq.Edge.makeLine(cq.Vector(*p[0]), cq.Vector(*p[1])), cq.Edge.makeThreePointArc(cq.Vector(*p[1]), cq.Vector(x, -w + r - k, z + r - k), cq.Vector(*p[2])), cq.Edge.makeLine(cq.Vector(*p[2]), cq.Vector(*p[3])), cq.Edge.makeThreePointArc(cq.Vector(*p[3]), cq.Vector(x, w - r + k, z + r - k), cq.Vector(*p[4])), cq.Edge.makeLine(cq.Vector(*p[4]), cq.Vector(*p[5])), cq.Edge.makeLine(cq.Vector(*p[5]), cq.Vector(*p[0]))])
    stations = [x_rear, -40, -38, -35, -30, -20, -5, 10, 20, 24, 28, 32, 40, 50, 55, 60, 62, 64, 66, 68, 70, 72, 75, 90, 105, flank_x1, 115, 116.4, x_front]
    stations = sorted(set(stations + list(np.arange(-40, 116.01, station_step))))
    for _ in range(12):
        add = []
        for a, c in zip(stations[:-1], stations[1:]):
            for t in (0.25, 0.5, 0.75):
                x = a + (c - a) * t
                va = (width(a) - relief(a), base_z(a))
                vc = (width(c) - relief(c), base_z(c))
                vx = (width(x) - relief(x), base_z(x))
                if max((abs(vx[i] - ((1 - t) * va[i] + t * vc[i])) for i in range(2))) > chord_tol:
                    add.append((a + c) / 2)
                    break
        if not add:
            break
        stations = sorted(set(stations + add))
    
    def loft(xs, inner):
        return cq.Solid.makeLoft([section(x, inner) for x in xs], True)
    
    def plan_wire(with_relief):
        """Plan outline polygon through the stations (with or without the jaw relief)."""
        hw = [width(x) - (relief(x) if with_relief else 0.0) for x in stations]
        pts = [cq.Vector(x, -h, 0) for x, h in zip(stations, hw)] + [cq.Vector(x, h, 0) for x, h in zip(stations[::-1], hw[::-1])]
        return cq.Wire.makePolygon(pts, close=True)
    
    def inset(wire, t):
        return wire if t == 0 else wire.offset2D(-t, 'arc')[0]
    
    def band(wire, t_out, t_in, z0, z1):
        """Prism z0..z1 of the plan band between insets t_out and t_in (t_in None: full inset face)."""
        face = cq.Face.makeFromWires(inset(wire, t_out), [] if t_in is None else [inset(wire, t_in)])
        return cq.Solid.extrudeLinear(face.translate((0, 0, z0)), cq.Vector(0, 0, z1 - z0))
    rim_plan = plan_wire(True)
    full_plan = plan_wire(False)
    outer_skin = loft(stations, False)
    inner_cavity = loft([inner_x0] + [x for x in stations if inner_x0 < x < inner_x1] + [inner_x1], True)
    model = Model()
    model.add('outer_skin', outer_skin, op='new', body='shell', name='Outer skin (section loft)')
    model.add('hollow', inner_cavity, op='cut', body='shell', name='Hollow (inner section loft)')
    model.add('seam_rebate', band(rim_plan, 0, seam_rebate_t, seam_rebate_z0, inner_top), op='cut', body='shell', name='Seam rebate')
    model.add('rim_step1', band(rim_plan, rim1_in, rim1_out, rim1_z0, rim1_z1), op='new', body='rim', name='Seam rim step 1')
    model.add('rim_step2', band(rim_plan, rim2_in, rim2_out, rim2_z0, rim2_z1), op='join', body='rim', name='Seam rim step 2')
    model.box('rim_gap_right', rim_gap_x1 - rim_gap_x0, tool_y - rim_gap_y, tool_z1 - rim_gap_z0, at=((rim_gap_x0 + rim_gap_x1) / 2, -(tool_y + rim_gap_y) / 2, (rim_gap_z0 + tool_z1) / 2), op='cut', body='rim')
    model.box('rim_gap_left', rim_gap_x1 - rim_gap_x0, tool_y - rim_gap_y, tool_z1 - rim_gap_z0, at=((rim_gap_x0 + rim_gap_x1) / 2, (rim_gap_y + tool_y) / 2, (rim_gap_z0 + tool_z1) / 2), op='cut', body='rim')
    model.box('rim_front_trim', tool_x1 - rim_front_x0, 2 * tool_y, tool_z1 - rim_front_z0, at=((rim_front_x0 + tool_x1) / 2, 0.0, (rim_front_z0 + tool_z1) / 2), op='cut', body='rim')
    model.combine('join_rim', 'shell', ['rim'], op='join', name='Join seam rim')
    model.box('front_floor_cut', tool_x1 - front_floor_cut_x0, 2 * tool_y, tool_z1 - front_floor_cut_z0, at=((front_floor_cut_x0 + tool_x1) / 2, 0.0, (front_floor_cut_z0 + tool_z1) / 2), op='cut', body='shell')
    with model.sketch('sketch_mouth_seat', on='XZ', name='Soft-mouth seat profile') as s:
        s.line('mouth_0', (mouth_x0, mouth_cut_z0), (mouth_x1, mouth_cut_z0))
        s.line('mouth_1', (mouth_x1, mouth_cut_z0), (mouth_x1, mouth_z0 + front_floor_slope * mouth_x1))
        s.line('mouth_2', (mouth_x1, mouth_z0 + front_floor_slope * mouth_x1), (mouth_x0, mouth_z0 + front_floor_slope * mouth_x0))
        s.line('mouth_3', (mouth_x0, mouth_z0 + front_floor_slope * mouth_x0), (mouth_x0, mouth_cut_z0))
    model.extrude('mouth_seat', 'sketch_mouth_seat', distance=big, both=True, op='cut', body='shell', name='Soft-mouth seat')
    with model.sketch('sketch_below', on='XZ', name='Skirt underside profile') as s:
        s.line('below_0', (below_x0, cut_z0), (tool_x1, cut_z0))
        s.line('below_1', (tool_x1, cut_z0), (tool_x1, below_z0 + front_floor_slope * tool_x1))
        s.line('below_2', (tool_x1, below_z0 + front_floor_slope * tool_x1), (below_x0, below_z0 + front_floor_slope * below_x0))
        s.line('below_3', (below_x0, below_z0 + front_floor_slope * below_x0), (below_x0, cut_z0))
    model.add('skirt', band(rim_plan, 0, skirt_t, skirt_z0, top_end), op='new', body='skirt', name='Front skirt band')
    model.box('skirt_window', tool_x1 - skirt_x0, 2 * tool_y, tool_z1 - window_z0, at=((skirt_x0 + tool_x1) / 2, 0.0, (window_z0 + tool_z1) / 2), op='intersect', body='skirt')
    model.extrude('skirt_underside', 'sketch_below', distance=big, both=True, op='cut', body='skirt', name='Skirt underside')
    model.add('front_deep', band(rim_plan, 0, None, skirt_z0, top_end), op='new', body='front_deep', name='Deep front lip')
    model.box('front_deep_window', tool_x1 - front_deep_x0, 2 * tool_y, tool_z1 - window_z0, at=((front_deep_x0 + tool_x1) / 2, 0.0, (window_z0 + tool_z1) / 2), op='intersect', body='front_deep')
    model.extrude('front_deep_underside', 'sketch_below', distance=big, both=True, op='cut', body='front_deep', name='Deep lip underside')
    model.combine('merge_front', 'skirt', ['front_deep'], op='join', name='Merge deep lip into the skirt body')
    model.plane('plane_land', on='XY', offset=land_z0, name='Optical land base')
    with model.sketch('sketch_land', on='plane_land', name='Optical face land') as s:
        s.line('land_b', (land_cx - land_w / 2 + land_r, 0.0 - land_h / 2), (land_cx + land_w / 2 - land_r, 0.0 - land_h / 2))
        s.arc('land_br', (land_cx + land_w / 2 - land_r, 0.0 - land_h / 2 + land_r), land_r, 270, 360)
        s.line('land_r', (land_cx + land_w / 2, 0.0 - land_h / 2 + land_r), (land_cx + land_w / 2, 0.0 + land_h / 2 - land_r))
        s.arc('land_tr', (land_cx + land_w / 2 - land_r, 0.0 + land_h / 2 - land_r), land_r, 0, 90)
        s.line('land_t', (land_cx + land_w / 2 - land_r, 0.0 + land_h / 2), (land_cx - land_w / 2 + land_r, 0.0 + land_h / 2))
        s.arc('land_tl', (land_cx - land_w / 2 + land_r, 0.0 + land_h / 2 - land_r), land_r, 90, 180)
        s.line('land_l', (land_cx - land_w / 2, 0.0 + land_h / 2 - land_r), (land_cx - land_w / 2, 0.0 - land_h / 2 + land_r))
        s.arc('land_bl', (land_cx - land_w / 2 + land_r, 0.0 - land_h / 2 + land_r), land_r, 180, 270)
    model.extrude('front_land', 'sketch_land', distance=land_z1 - land_z0, op='join', body='skirt', name='Optical face land')
    model.combine('join_front', 'shell', ['skirt'], op='join', name='Join skirt, deep lip and optical land')
    model.add('return_skirt', band(full_plan, 0, return_skirt_t, skirt_z0, top_end), op='new', body='ret', name='Return skirt band')
    model.box('return_window', return_x1 - return_x0, 2 * tool_y, tool_z1 - window_z0, at=((return_x0 + return_x1) / 2, 0.0, (window_z0 + tool_z1) / 2), op='intersect', body='ret')
    model.extrude('return_underside', 'sketch_below', distance=big, both=True, op='cut', body='ret', name='Return underside')
    model.add('return_roof', band(full_plan, 0, return_roof_t, return_roof_z0, top_end), op='new', body='roof', name='Return roof band')
    model.box('roof_window', return_x1 - return_x0, 2 * tool_y, tool_z1 - window_z0, at=((return_x0 + return_x1) / 2, 0.0, (window_z0 + tool_z1) / 2), op='intersect', body='roof')
    model.combine('merge_return', 'ret', ['roof'], op='join', name='Merge roof into the return body')
    model.combine('join_return', 'shell', ['ret'], op='join', name='Join jaw-side return')
    model.box('seat_pocket', pocket_x1 - pocket_x0, 2 * pocket_y, tool_z1 - front_floor_cut_z0, at=((pocket_x0 + pocket_x1) / 2, 0.0, (front_floor_cut_z0 + tool_z1) / 2), op='cut', body='shell')
    model.plane('plane_cut', on='XY', offset=cut_z0, name='Through-cut base')
    with model.sketch('sketch_aperture', on='plane_cut', name='Neck/servo aperture') as s:
        s.line('ap_b', (ap_cx - ap_w / 2 + ap_r, 0.0 - ap_h / 2), (ap_cx + ap_w / 2 - ap_r, 0.0 - ap_h / 2))
        s.arc('ap_br', (ap_cx + ap_w / 2 - ap_r, 0.0 - ap_h / 2 + ap_r), ap_r, 270, 360)
        s.line('ap_r', (ap_cx + ap_w / 2, 0.0 - ap_h / 2 + ap_r), (ap_cx + ap_w / 2, 0.0 + ap_h / 2 - ap_r))
        s.arc('ap_tr', (ap_cx + ap_w / 2 - ap_r, 0.0 + ap_h / 2 - ap_r), ap_r, 0, 90)
        s.line('ap_t', (ap_cx + ap_w / 2 - ap_r, 0.0 + ap_h / 2), (ap_cx - ap_w / 2 + ap_r, 0.0 + ap_h / 2))
        s.arc('ap_tl', (ap_cx - ap_w / 2 + ap_r, 0.0 + ap_h / 2 - ap_r), ap_r, 90, 180)
        s.line('ap_l', (ap_cx - ap_w / 2, 0.0 + ap_h / 2 - ap_r), (ap_cx - ap_w / 2, 0.0 - ap_h / 2 + ap_r))
        s.arc('ap_bl', (ap_cx - ap_w / 2 + ap_r, 0.0 - ap_h / 2 + ap_r), ap_r, 180, 270)
    model.extrude('aperture', 'sketch_aperture', distance=cut_z1 - cut_z0, op='cut', body='shell', name='Neck/servo aperture')
    with model.sketch('sketch_yoke', on='plane_cut', name='Yoke sweep extension') as s:
        s.line('yoke_b', (yoke_cx - yoke_w / 2 + yoke_r, yoke_cy - yoke_h / 2), (yoke_cx + yoke_w / 2 - yoke_r, yoke_cy - yoke_h / 2))
        s.arc('yoke_br', (yoke_cx + yoke_w / 2 - yoke_r, yoke_cy - yoke_h / 2 + yoke_r), yoke_r, 270, 360)
        s.line('yoke_r', (yoke_cx + yoke_w / 2, yoke_cy - yoke_h / 2 + yoke_r), (yoke_cx + yoke_w / 2, yoke_cy + yoke_h / 2 - yoke_r))
        s.arc('yoke_tr', (yoke_cx + yoke_w / 2 - yoke_r, yoke_cy + yoke_h / 2 - yoke_r), yoke_r, 0, 90)
        s.line('yoke_t', (yoke_cx + yoke_w / 2 - yoke_r, yoke_cy + yoke_h / 2), (yoke_cx - yoke_w / 2 + yoke_r, yoke_cy + yoke_h / 2))
        s.arc('yoke_tl', (yoke_cx - yoke_w / 2 + yoke_r, yoke_cy + yoke_h / 2 - yoke_r), yoke_r, 90, 180)
        s.line('yoke_l', (yoke_cx - yoke_w / 2, yoke_cy + yoke_h / 2 - yoke_r), (yoke_cx - yoke_w / 2, yoke_cy - yoke_h / 2 + yoke_r))
        s.arc('yoke_bl', (yoke_cx - yoke_w / 2 + yoke_r, yoke_cy - yoke_h / 2 + yoke_r), yoke_r, 180, 270)
    model.extrude('yoke', 'sketch_yoke', distance=yoke_z1 - cut_z0, op='cut', body='shell', name='Yoke sweep extension')
    with model.sketch('sketch_harness', on='plane_cut', name='Rear harness opening') as s:
        s.line('harness_b', (harness_cx - harness_w / 2 + harness_r, harness_cy - harness_h / 2), (harness_cx + harness_w / 2 - harness_r, harness_cy - harness_h / 2))
        s.arc('harness_br', (harness_cx + harness_w / 2 - harness_r, harness_cy - harness_h / 2 + harness_r), harness_r, 270, 360)
        s.line('harness_r', (harness_cx + harness_w / 2, harness_cy - harness_h / 2 + harness_r), (harness_cx + harness_w / 2, harness_cy + harness_h / 2 - harness_r))
        s.arc('harness_tr', (harness_cx + harness_w / 2 - harness_r, harness_cy + harness_h / 2 - harness_r), harness_r, 0, 90)
        s.line('harness_t', (harness_cx + harness_w / 2 - harness_r, harness_cy + harness_h / 2), (harness_cx - harness_w / 2 + harness_r, harness_cy + harness_h / 2))
        s.arc('harness_tl', (harness_cx - harness_w / 2 + harness_r, harness_cy + harness_h / 2 - harness_r), harness_r, 90, 180)
        s.line('harness_l', (harness_cx - harness_w / 2, harness_cy + harness_h / 2 - harness_r), (harness_cx - harness_w / 2, harness_cy - harness_h / 2 + harness_r))
        s.arc('harness_bl', (harness_cx - harness_w / 2 + harness_r, harness_cy - harness_h / 2 + harness_r), harness_r, 180, 270)
    model.extrude('harness', 'sketch_harness', distance=cut_z1 - cut_z0, op='cut', body='shell', name='Rear harness opening')
    model.cylinder('seat_pad', seat_pad_r, seat_pad_z1 - seat_pad_z0, at=(seat_x, seat_y, seat_pad_z0), op='new', body='bosses', name='Rear seat pad')
    model.plane('plane_cover_land', on='XY', offset=cover_land_z0, name='Cover land base')
    with model.sketch('sketch_cover_land', on='plane_cover_land', name='Cover land block') as s:
        s.line('cl_b', (cl_cx - cl_w / 2 + cl_r, cl_cy - cl_h / 2), (cl_cx + cl_w / 2 - cl_r, cl_cy - cl_h / 2))
        s.arc('cl_br', (cl_cx + cl_w / 2 - cl_r, cl_cy - cl_h / 2 + cl_r), cl_r, 270, 360)
        s.line('cl_r', (cl_cx + cl_w / 2, cl_cy - cl_h / 2 + cl_r), (cl_cx + cl_w / 2, cl_cy + cl_h / 2 - cl_r))
        s.arc('cl_tr', (cl_cx + cl_w / 2 - cl_r, cl_cy + cl_h / 2 - cl_r), cl_r, 0, 90)
        s.line('cl_t', (cl_cx + cl_w / 2 - cl_r, cl_cy + cl_h / 2), (cl_cx - cl_w / 2 + cl_r, cl_cy + cl_h / 2))
        s.arc('cl_tl', (cl_cx - cl_w / 2 + cl_r, cl_cy + cl_h / 2 - cl_r), cl_r, 90, 180)
        s.line('cl_l', (cl_cx - cl_w / 2, cl_cy + cl_h / 2 - cl_r), (cl_cx - cl_w / 2, cl_cy - cl_h / 2 + cl_r))
        s.arc('cl_bl', (cl_cx - cl_w / 2 + cl_r, cl_cy - cl_h / 2 + cl_r), cl_r, 180, 270)
    model.extrude('cover_land_block', 'sketch_cover_land', distance=cover_land_z1 - cover_land_z0, op='join', body='bosses', name='Cover land block')
    for i, (x, y) in enumerate(cover_lands):
        z0 = cover_land_front_z0 if x > 60 else cover_land_z0
        model.cylinder(f'cover_land_{i}', cover_land_r, cover_land_z1 - z0, at=(x, y, z0), op='join', body='bosses', name=f'Cover land {i}')
    for i, x in enumerate(mic_xs):
        z0 = base_z(x) + boss_lift
        model.cylinder(f'mic_boss_{i}', mic_r, mic_top - z0, at=(x, mic_y, z0), op='join', body='bosses', name=f'Mic boss {i}')
    for i, x in enumerate(ubec_xs):
        z0 = base_z(x) + boss_lift
        model.cylinder(f'ubec_base_{i}', ubec_r, ubec_top - z0, at=(x, ubec_y, z0), op='join', body='bosses', name=f'UBEC strap base {i}')
    for i, (x, y) in enumerate(cover_front):
        model.cylinder(f'cover_front_pad_{i}', cover_front_r, cover_front_z1 - cover_front_z0, at=(x, y, cover_front_z0), op='join', body='bosses', name=f'Front cover pad {i}')
    outer_envelope = outer_skin.fuse(band(full_plan, post_clip_inset, None, envelope_z0, envelope_top)).clean()
    for i, ((x, y), (bx0, bx1, by0, by1, bz1)) in enumerate(zip(cover_rear, post_webs)):
        model.cylinder(f'post_{i}', post_r, post_z1 - post_z0, at=(x, y, post_z0), op='new', body=f'post{i}', name=f'Rear cover post {i}')
        model.box(f'post_web_{i}', bx1 - bx0, by1 - by0, bz1 - post_z0, at=((bx0 + bx1) / 2, (by0 + by1) / 2, (post_z0 + bz1) / 2), op='join', body=f'post{i}')
        model.add(f'post_clip_{i}', outer_envelope, op='intersect', body=f'post{i}', name=f'Clip post {i} to outer envelope')
    model.combine('merge_posts', 'bosses', ['post0', 'post1'], op='join', name='Merge clipped rear posts into the bosses body')
    model.combine('join_bosses', 'shell', ['bosses'], op='join', name='Join pads, bosses and rear posts')
    model.cylinder('seat_clear', m3_r, seat_clr_z1 - hole_z0, at=(seat_x, seat_y, hole_z0), op='new', body='holes', name='Rear seat clearance')
    model.cylinder('seat_cb', cb_r, seat_cb_z1 - hole_z0, at=(seat_x, seat_y, hole_z0), op='join', body='holes', name='Rear seat counterbore')
    for i, (x, y) in enumerate(cover_lands):
        model.cylinder(f'cover_land_hole_{i}', m3_r, cover_land_hole_z1 - cover_land_hole_z0, at=(x, y, cover_land_hole_z0), op='join', body='holes', name=f'Cover land hole {i}')
        model.cylinder(f'cover_land_cb_{i}', cb_r, cover_land_front_z0 - hole_z0, at=(x, y, hole_z0), op='join', body='holes', name=f'Cover land counterbore {i}')
    
    def self_forming_hole(x, y, z_face, z_end, d=sft_d, margin=sft_margin, ridge_h=sft_ridge_h, taper=sft_taper, ridges=sft_ridges, overshoot=2.0):
        """Cutting tool for a self-forming thread hole on a vertical axis at (x, y).
    
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
        for k in range(int(ridges)):
            a = 2.0 * np.pi * k / ridges
            cx, cy = (x + off * np.cos(a), y + off * np.sin(a))
            entry = cq.Solid.makeCylinder(big_r - ridge_h, overshoot, cq.Vector(cx, cy, z_out), dz)
            ramp = cq.Solid.makeCone(big_r - ridge_h, big_r, taper, cq.Vector(cx, cy, z_face), dz)
            full = cq.Solid.makeCylinder(big_r, abs(z_end - z_face) - taper, cq.Vector(cx, cy, z_face + s * taper), dz)
            tool = tool.cut(entry.fuse(ramp).fuse(full).clean())
        return tool.clean()
    for i, x in enumerate(mic_xs):
        model.add(f'mic_pilot_{i}', self_forming_hole(x, mic_y, mic_top, mic_pilot_z0), op='join', body='holes', name=f'Mic self-forming M2 hole {i}')
    for i, x in enumerate(ubec_xs):
        model.add(f'ubec_pilot_{i}', self_forming_hole(x, ubec_y, ubec_top, ubec_pilot_z0), op='join', body='holes', name=f'UBEC self-forming M2 hole {i}')
    model.cylinder('acoustic_port', port_r, port_z1 - port_z0, at=(port_x, port_y, port_z0), op='join', body='holes', name='Acoustic port')
    for i, (x, y) in enumerate(cover_front):
        model.cylinder(f'cover_front_hole_{i}', m3_r, hole_z1 - hole_z0, at=(x, y, hole_z0), op='join', body='holes', name=f'Front cover hole {i}')
        model.cylinder(f'cover_front_cb_{i}', cb_r, cover_front_cb_z1 - hole_z0, at=(x, y, hole_z0), op='join', body='holes', name=f'Front cover counterbore {i}')
    for i, (x, y) in enumerate(cover_rear):
        model.cylinder(f'cover_rear_hole_{i}', m3_r, hole_z1 - hole_z0, at=(x, y, hole_z0), op='join', body='holes', name=f'Rear cover hole {i}')
        model.cylinder(f'cover_rear_cb_{i}', cb_r, cover_rear_cb_z1 - hole_z0, at=(x, y, hole_z0), op='join', body='holes', name=f'Rear cover counterbore {i}')
    model.combine('cut_holes', 'shell', ['holes'], op='cut', name='Cut all screw holes, counterbores, pilots and the acoustic port')
    model.material('pla', color='#f96c0a')
    result = model.result()
    return to_build123d(model, 'bottom_head_shell_native')


@step(out="../../STEP/parts/bottom_head_shell_native.step")
def bottom_head_shell_native():
    return make()


if __name__ == "__main__":
    bottom_head_shell_native()
