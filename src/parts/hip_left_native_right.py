from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    import math
    import cadquery as cq
    SIDE = -1
    z0 = 87.55
    z1 = 113.2
    end_r = 12.825
    edge_r = 0.9
    ya_x = 5.4
    ya_z = 100.375
    yp0 = 45.9
    yp1 = 51.3
    y_horn = 51.975
    xa_y = 23.625
    x_horn = 30.375
    xp0 = 31.05
    xp1 = 36.45
    xp1c = 34.425
    in_cx = 22.95
    in_cy = 37.8
    in_r = 8.1
    out_cx = 23.625
    out_cy = 40.5
    out_r = 10.8
    s_cc = (43.79, 40.155)
    s_cr = 9.37
    s_vr = 20.5
    pad_r0 = 12.75
    pad_r1 = 15.65
    bar_z0 = 93.67
    bar_z1 = 107.08
    bar_x1 = 39.15
    bar_cr = 6.75
    bar_base_x = 34.425
    bar_base_hw = 8.0
    bar_taper = 20.0
    bar_edge_r = 0.9
    bar_fil_r = 3.5
    bar_fil_r2 = 0.8
    bolt_r = 7.0
    hole_r = 1.7
    rec_r = 3.2
    rec_d = 3.2
    cb_r = 3.4
    x_cb = 34.375
    y_cb = 47.975
    BIG = 200.0
    
    def arc_pt(c, r, ang):
        return (c[0] + r * math.cos(ang), c[1] + r * math.sin(ang))
    vc = (xp1 - s_vr, None)
    dx = s_cc[0] - vc[0]
    dy = math.sqrt((s_cr + s_vr) ** 2 - dx ** 2)
    vc = (vc[0], s_cc[1] - dy)
    ang = math.atan2(s_cc[1] - vc[1], s_cc[0] - vc[0])
    t_pt = arc_pt(vc, s_vr, ang)
    L = cq.Workplane('XY', origin=(0, 0, z0)).moveTo(-20, yp0).lineTo(in_cx, yp0).threePointArc(arc_pt((in_cx, in_cy), in_r, math.radians(45)), (in_cx + in_r, in_cy)).lineTo(xp0, 0).lineTo(xp1, 0).lineTo(xp1, vc[1]).threePointArc(arc_pt(vc, s_vr, ang / 2), t_pt).threePointArc(arc_pt(s_cc, s_cr, (math.pi + ang + math.pi) / 2), (xp1c, s_cc[1])).lineTo(xp1c, out_cy).threePointArc(arc_pt((out_cx, out_cy), out_r, math.radians(45)), (out_cx, yp1)).lineTo(-20, yp1).close().extrude(z1 - z0)
    ystad = cq.Workplane('XZ', origin=(0, 80, 0)).moveTo(ya_x, z0).lineTo(BIG, z0).lineTo(BIG, z1).lineTo(ya_x, z1).threePointArc((ya_x - end_r, ya_z), (ya_x, z0)).close().extrude(100)
    xstad = cq.Workplane('YZ', origin=(-30, 0, 0)).moveTo(xa_y, z0).lineTo(BIG, z0).lineTo(BIG, z1).lineTo(xa_y, z1).threePointArc((xa_y - end_r, ya_z), (xa_y, z0)).close().extrude(100)
    body = L.intersect(ystad).intersect(xstad)
    
    def under_pad(e):
        pts = [e.positionAt(t) for t in (0.0, 0.25, 0.5, 0.75, 1.0)]
        on_x = all((abs(p.x - xp0) < 0.001 and math.hypot(p.y - xa_y, p.z - ya_z) < pad_r1 for p in pts))
        on_y = all((abs(p.y - yp1) < 0.001 and math.hypot(p.x - ya_x, p.z - ya_z) < pad_r1 for p in pts))
        on_pad = all((math.hypot(p.y - xa_y, p.z - ya_z) < pad_r1 + 0.01 and x_horn - 0.001 < p.x < xp0 + 0.001 for p in pts)) or all((math.hypot(p.x - ya_x, p.z - ya_z) < pad_r1 + 0.01 and yp1 - 0.001 < p.y < y_horn + 0.001 for p in pts))
        return on_x or on_y or on_pad
    s_body = body.val()
    body = cq.Workplane().add(s_body.fillet(edge_r, [e for e in s_body.Edges() if not under_pad(e)]))
    
    def bar_outline():
        return cq.Workplane('XY', origin=(0, 0, bar_z0)).moveTo(out_cx, 47.5).lineTo(out_cx, yp1).lineTo(bar_x1 - bar_cr, yp1).threePointArc(arc_pt((bar_x1 - bar_cr, yp1 - bar_cr), bar_cr, math.radians(45)), (bar_x1, yp1 - bar_cr)).lineTo(bar_x1, 5).lineTo(34.2, 5).lineTo(34.2, 38.5).lineTo(31.5, 44).lineTo(27, 47.5).close()
    bar = bar_outline().extrude(bar_z1 - bar_z0)
    tap = cq.Workplane('YZ', origin=(bar_base_x, 0, 0)).moveTo(xa_y, ya_z - bar_base_hw).lineTo(80, ya_z - bar_base_hw).lineTo(80, ya_z + bar_base_hw).lineTo(xa_y, ya_z + bar_base_hw).threePointArc((xa_y - bar_base_hw, ya_z), (xa_y, ya_z - bar_base_hw)).close().extrude(bar_x1 - bar_base_x + 0.5, taper=bar_taper)
    tap = tap.union(cq.Workplane('XY').box(20, 50, 40, centered=(False, False, True)).translate((bar_base_x - 20 + 0.001, 30, ya_z)))
    bar = bar.intersect(tap)
    bar = bar.faces('>X').edges().fillet(bar_edge_r)
    body = body.union(bar)
    from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
    bar_xy = bar_outline().extrude(bar_z1 - bar_z0 + 2 * bar_fil_r + 2).translate((0, 0, -bar_fil_r - 1))
    s0 = body.val()
    mk = BRepFilletAPI_MakeFillet(s0.wrapped)
    for e in s0.Edges():
        pts = [e.positionAt(t) for t in (0.0, 0.25, 0.5, 0.75, 1.0)]
        if all((abs(p.x - xp1) < 0.001 and bar_z0 - 0.001 < p.z < bar_z1 + 0.001 for p in pts)):
            mk.Add(bar_fil_r2, e.wrapped)
    mk.Build()
    body = cq.Workplane().add(cq.Shape.cast(mk.Shape()))
    
    def fil_section(w, rho, s, zb, t, r):
        return w.moveTo(rho, zb).lineTo(rho + s * r, zb).threePointArc((rho + s * r - s * r * math.sqrt(0.5), zb + t * r - t * r * math.sqrt(0.5)), (rho, zb + t * r)).close()
    
    def wedge(c, a0, a1, R=60.0):
        n = 8
        pts = [c] + [(c[0] + R * math.cos(a0 + (a1 - a0) * i / n), c[1] + R * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
        return cq.Workplane('XY', origin=(0, 0, 80)).polyline(pts).close().extrude(40)
    
    def arc_fillet(c, R, s, a0, a1, r=bar_fil_r):
        ring = None
        for zb, t in ((bar_z1, 1), (bar_z0, -1)):
            sec = fil_section(cq.Workplane('XZ'), R, s, zb, t, r)
            piece = sec.revolve(360, (0, 0, 0), (0, 1, 0))
            ring = piece if ring is None else ring.union(piece)
        lo, hi = (min(a0, a1), max(a0, a1))
        return ring.translate((c[0], c[1], 0)).intersect(wedge(c, lo, hi))
    
    def line_fillet(x, y0, y1, s, r=bar_fil_r):
        out = None
        for zb, t in ((bar_z1, 1), (bar_z0, -1)):
            piece = fil_section(cq.Workplane('XZ', origin=(0, y1, 0)), x, s, zb, t, r).extrude(y1 - y0)
            out = piece if out is None else out.union(piece)
        return out
    blend = arc_fillet(vc, s_vr, 1, 0.0, ang, bar_fil_r2)
    blend = blend.union(arc_fillet(s_cc, s_cr, -1, ang + math.pi, math.pi))
    blend = blend.union(line_fillet(xp1c, s_cc[1], out_cy, 1))
    blend = blend.union(arc_fillet((out_cx, out_cy), out_r, 1, 0.0, math.pi / 2))
    body = body.union(blend.intersect(bar_xy))
    pad_h = xp0 - x_horn
    pad_k = (pad_r1 - pad_r0) / pad_h
    xpad = cq.Solid.makeCone(pad_r1 + pad_k * edge_r, pad_r0, pad_h + edge_r, cq.Vector(xp0 + edge_r, xa_y, ya_z), cq.Vector(-1, 0, 0))
    ypad = cq.Solid.makeCone(pad_r1 + pad_k * edge_r, pad_r0, pad_h + edge_r, cq.Vector(ya_x, yp1 - edge_r, ya_z), cq.Vector(0, 1, 0))
    body = body.union(cq.Workplane().add(xpad).intersect(xstad)).union(cq.Workplane().add(ypad).intersect(ystad))
    xpts = [(xa_y + bolt_r, ya_z), (xa_y - bolt_r, ya_z), (xa_y, ya_z + bolt_r), (xa_y, ya_z - bolt_r)]
    ypts = [(ya_x + bolt_r, ya_z), (ya_x - bolt_r, ya_z), (ya_x, ya_z + bolt_r), (ya_x, ya_z - bolt_r)]
    body = body.cut(cq.Workplane('YZ', origin=(x_horn - 1, 0, 0)).center(xa_y, ya_z).circle(rec_r).extrude(rec_d + 1))
    body = body.cut(cq.Workplane('YZ', origin=(x_horn - 1, 0, 0)).pushPoints(xpts).circle(hole_r).extrude(12))
    body = body.cut(cq.Workplane('YZ', origin=(x_cb, 0, 0)).pushPoints(xpts).circle(cb_r).extrude(8))
    body = body.cut(cq.Workplane('XZ', origin=(0, y_horn + 1, 0)).center(ya_x, ya_z).circle(rec_r).extrude(rec_d + 1))
    body = body.cut(cq.Workplane('XZ', origin=(0, y_horn + 1, 0)).pushPoints(ypts).circle(hole_r).extrude(10))
    body = body.cut(cq.Workplane('XZ', origin=(0, y_cb, 0)).pushPoints(ypts).circle(cb_r).extrude(5))
    if SIDE < 0:
        body = body.mirror('XZ')
    from lib.steve_compat import Model
    model = Model()
    model.add('printed_body', body)
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'hip_left_native')


@step(out="../../STEP/parts/hip_left_native_right.step")
def hip_left_native_right():
    return make()


if __name__ == "__main__":
    hip_left_native_right()
