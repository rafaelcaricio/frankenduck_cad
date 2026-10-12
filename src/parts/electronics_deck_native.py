from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    """Head electronics deck for the bus-servo adapter. World millimetres, Z up."""
    import math
    from lib.cadquery_adapter import Model
    deck_z0 = 300.6
    deck_z1 = 303.6
    outline = [(-24.5, -21.0), (0.0, -21.0), (14.0, -14.0), (14.0, 0.5), (61.5, 0.5), (61.5, 7.5), (49.5, 7.5), (49.5, 14.5), (50.0, 14.5), (50.0, 39.5), (26.0, 39.5), (14.0, 33.0), (2.0, 33.0), (-24.5, 15.5)]
    outline_r = [3, 3, 2, 2, 3, 3, 2, 0, 0, 3, 3, 3, 3, 3]
    tab_x0 = 46.0
    tab_x1 = 61.5
    tab_y0 = 0.5
    tab_y1 = 7.5
    tab_r = 2.0
    tab_z0 = 303.1
    tab_z1 = 305.1
    m3_clr_r = 1.7
    m3_holes = [(-20.77, -17.5), (-20.77, 2.5), (57.0, 4.0)]
    hole_z0 = 299.6
    hole_z1 = 306.1
    relief_depth = 1.0
    reliefs = [(1.7, 17.2, 11.9, 29.1), (4.4, 15.9, -11.8, -1.6), (14.7, 26.2, -5.8, 4.4), (-7.1, 3.3, 12.7, 23.1)]
    adapter_posts = [(13.82, 27.36), (27.82, 3.11), (-18.22, 8.86), (-4.23, -15.4)]
    adapter_post_r = 3.0
    adapter_post_z1 = 306.05
    imu_posts = [(30.88, 15.84), (30.88, 36.16), (46.12, 15.84), (46.12, 36.16)]
    imu_post_r = 2.6
    imu_post_z0 = 303.1
    imu_post_z1 = 305.4
    m2_pilot_r = 0.8
    adapter_pilot_z0 = 302.6
    adapter_pilot_z1 = 307.0
    imu_pilot_z0 = 302.3
    imu_pilot_z1 = 306.0
    
    def rounded_segments(pts, radii):
        """Lines and tangent arcs of a closed polygon with per-corner radii (convex or concave)."""
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
            t = r / math.tan(ang / 2)
            q0 = (p1[0] + ax * t, p1[1] + ay * t)
            q1 = (p1[0] + bx * t, p1[1] + by * t)
            sx, sy = (ax + bx, ay + by)
            sl = math.hypot(sx, sy)
            d = r / math.sin(ang / 2)
            c = (p1[0] + sx / sl * d, p1[1] + sy / sl * d)
            a0 = math.degrees(math.atan2(q0[1] - c[1], q0[0] - c[0]))
            a1 = math.degrees(math.atan2(q1[1] - c[1], q1[0] - c[0]))
            da = (a1 - a0 + 180.0) % 360.0 - 180.0
            start, end = (a0, a0 + da) if da > 0 else (a1, a1 - da)
            corners.append((q0, q1, (c, r, start, end)))
        segs = []
        for i in range(n):
            q0, q1, arc = corners[i]
            if arc is not None:
                segs.append(('arc', arc))
            segs.append(('line', (q1, corners[(i + 1) % n][0])))
        return segs
    deck_segments = rounded_segments(outline, outline_r)
    model = Model()
    model.plane('plane_deck', on='XY', offset=deck_z0, name='Deck underside')
    with model.sketch('sketch_deck', on='plane_deck', name='Deck outline') as s:
        for k, (kind, g) in enumerate(deck_segments):
            if kind == 'line':
                s.line(f'deck_l{k}', g[0], g[1])
            else:
                s.arc(f'deck_a{k}', g[0], g[1], g[2], g[3])
    model.extrude('deck', 'sketch_deck', distance=deck_z1 - deck_z0, op='new', body='deck', name='Deck plate')
    model.plane('plane_tab', on='XY', offset=tab_z0, name='Tab bottom')
    with model.sketch('sketch_tab', on='plane_tab', name='Bulkhead tab outline') as s:
        s.line('tab_b', (tab_x0, tab_y0), (tab_x1 - tab_r, tab_y0))
        s.arc('tab_br', (tab_x1 - tab_r, tab_y0 + tab_r), tab_r, 270, 360)
        s.line('tab_f', (tab_x1, tab_y0 + tab_r), (tab_x1, tab_y1 - tab_r))
        s.arc('tab_tr', (tab_x1 - tab_r, tab_y1 - tab_r), tab_r, 0, 90)
        s.line('tab_t', (tab_x1 - tab_r, tab_y1), (tab_x0, tab_y1))
        s.line('tab_rear', (tab_x0, tab_y1), (tab_x0, tab_y0))
    model.extrude('tab', 'sketch_tab', distance=tab_z1 - tab_z0, op='join', body='deck', name='Bulkhead tab')
    for i, (x, y) in enumerate(m3_holes):
        model.cylinder(f'm3_clear_{i}', m3_clr_r, hole_z1 - hole_z0, at=(x, y, hole_z0), op='new' if i == 0 else 'join', body='cuts', name=f'M3 clearance {i}')
    for i, (x0, x1, y0, y1) in enumerate(reliefs):
        model.box(f'pin_relief_{i}', x1 - x0, y1 - y0, 2 * relief_depth, at=((x0 + x1) / 2, (y0 + y1) / 2, deck_z1), op='join', body='cuts', name=f'Adapter pin relief {i}')
    model.combine('cut_holes', 'deck', ['cuts'], op='cut', name='Cut clearance holes and pin reliefs')
    for i, (x, y) in enumerate(adapter_posts):
        model.cylinder(f'adapter_post_{i}', adapter_post_r, adapter_post_z1 - (deck_z1 - relief_depth), at=(x, y, deck_z1 - relief_depth), op='join', body='deck', name=f'Adapter standoff {i}')
    for i, (x, y) in enumerate(imu_posts):
        model.cylinder(f'imu_post_{i}', imu_post_r, imu_post_z1 - imu_post_z0, at=(x, y, imu_post_z0), op='join', body='deck', name=f'BNO055 standoff {i}')
    for i, (x, y) in enumerate(adapter_posts):
        model.cylinder(f'adapter_pilot_{i}', m2_pilot_r, adapter_pilot_z1 - adapter_pilot_z0, at=(x, y, adapter_pilot_z0), op='new' if i == 0 else 'join', body='pilots', name=f'Adapter M2 pilot {i}')
    for i, (x, y) in enumerate(imu_posts):
        model.cylinder(f'imu_pilot_{i}', m2_pilot_r, imu_pilot_z1 - imu_pilot_z0, at=(x, y, imu_pilot_z0), op='join', body='pilots', name=f'BNO055 M2 pilot {i}')
    model.combine('cut_pilots', 'deck', ['pilots'], op='cut', name='Cut M2 pilots')
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'electronics_deck_native')


@step(out="../../STEP/parts/electronics_deck_native.step")
def electronics_deck_native():
    return make()


if __name__ == "__main__":
    electronics_deck_native()
