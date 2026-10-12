from __future__ import annotations
from cadgen import step
from lib.cadquery_adapter import to_build123d


def make():
    import math
    import cadquery as cq
    from lib.cadquery_adapter import Model
    SIDE = 1
    hip_x = 5.4
    hip_z = 100.375
    knee_x = -42.899
    knee_z = 70.675
    corner_x = -41.938
    y_open = 57.375
    y_back = 95.175
    r_open = 18.181
    draft = 3.0
    wall = 1.35
    back_r = 5.0
    skirt_y = 83.975
    diag_y = 84.975
    diag_x0 = -19.88
    diag_x1 = -17.36
    diag_x2 = 7.24
    diag_x3 = 9.96
    ext_slope = 0.36397
    top_edge_x0 = -26.836
    left_edge_z0 = 104.798
    left_tip_r = 13.502
    left_tip_z = 93.375
    top_tip_x = -15.6
    tail_r = 1.7
    pad_r = 6.747
    pad_t = 0.15
    boss_r = 3.243
    boss_y = 87.175
    boss_fillet = 1.25
    hole_r = 1.1
    cb_r = 2.2
    cb_y = 90.175
    rib_y = 84.475
    bosses = [(-32.649, 78.975), (-53.149, 78.975), (-32.649, 103.425), (-53.149, 103.425), (-2.9, 110.625), (-2.9, 90.125), (-27.35, 110.625), (-27.35, 90.125)]
    ribs = [(-57.111, -55.511, 97.175, 102.175), (-57.111, -55.511, 64.675, 69.675), (-45.899, -39.899, 106.037, 107.637), (-45.899, -39.899, 58.713, 60.313), (-31.562, -29.962, 107.375, 111.375), (-30.287, -28.687, 76.675, 80.675), (-30.287, -28.687, 64.675, 69.675), (-26.1, -21.1, 86.163, 87.763), (-26.1, -21.1, 112.987, 114.587), (6.4, 11.4, 86.163, 87.763), (6.4, 11.4, 112.987, 114.587), (15.762, 17.362, 97.375, 103.375)]
    top_curve = [(-12.576, 57.677), (-8.382, 59.35), (-4.637, 62.755), (-2.839, 65.97), (-2.09, 67.989)]
    model = Model()
    
    def side(s):
        return s.mirror('XZ') if SIDE < 0 else s
    
    def y_cyl(r, y0, y1, x, z):
        return cq.Workplane().add(cq.Solid.makeCylinder(r, y1 - y0, cq.Vector(x, y0, z), cq.Vector(0, 1, 0)))
    circ = [(hip_x, hip_z), (corner_x, hip_z), (knee_x, knee_z)]
    
    def hull_wire(w, r):
        n = len(circ)
        tang = []
        for i in range(n):
            (px, pz), (qx, qz) = (circ[i], circ[(i + 1) % n])
            L = math.hypot(qx - px, qz - pz)
            nx, nz = ((qz - pz) / L, -(qx - px) / L)
            tang.append(((px + r * nx, pz + r * nz), (qx + r * nx, qz + r * nz), (nx, nz)))
        loc = lambda p: (-p[0], p[1])
        w = w.moveTo(*loc(tang[0][0]))
        for i in range(n):
            a, b, nrm = tang[i]
            w = w.lineTo(*loc(b))
            nxt = tang[(i + 1) % n]
            cx, cz = circ[(i + 1) % n]
            mx, mz = (nrm[0] + nxt[2][0], nrm[1] + nxt[2][1])
            k = math.hypot(mx, mz)
            w = w.threePointArc(loc((cx + r * mx / k, cz + r * mz / k)), loc(nxt[0]))
        return w.close()
    open_plane = cq.Plane(origin=(0, y_open, 0), xDir=(-1, 0, 0), normal=(0, 1, 0))
    outer = hull_wire(cq.Workplane(open_plane), r_open).extrude(y_back - y_open, taper=draft)
    outer = outer.faces('>Y').edges().fillet(back_r)
    tray = outer.faces('<Y').shell(-wall)
    model.add('tray', side(tray))
    s20 = 1 / math.hypot(ext_slope, 1)
    t_top = (ext_slope * s20, s20)
    c_top = (top_edge_x0 + ext_slope * skirt_y, skirt_y)
    L = tail_r / math.tan(math.radians(55))
    bis = ((-t_top[0] + 1) / math.hypot(-t_top[0] + 1, -t_top[1]), -t_top[1] / math.hypot(-t_top[0] + 1, -t_top[1]))
    cc = (c_top[0] + bis[0] * tail_r / math.sin(math.radians(55)), c_top[1] + bis[1] * tail_r / math.sin(math.radians(55)))
    keep_top = cq.Workplane('XY', origin=(0, 0, 100)).moveTo(-80, 50).lineTo(top_tip_x, 50).lineTo(top_tip_x, y_open).spline(top_curve, tangents=[(1, 0), t_top], includeCurrent=True).lineTo(c_top[0] - L * t_top[0], c_top[1] - L * t_top[1]).threePointArc((cc[0] - bis[0] * tail_r, cc[1] - bis[1] * tail_r), (c_top[0] + L, c_top[1])).lineTo(40, skirt_y).lineTo(40, 90).lineTo(-80, 90).close().extrude(25)
    t_l = (s20, -ext_slope * s20)
    c_l = (skirt_y, left_edge_z0 - ext_slope * skirt_y)
    bl = (-t_l[0] / math.hypot(-t_l[0], -t_l[1] - 1), (-t_l[1] - 1) / math.hypot(-t_l[0], -t_l[1] - 1))
    cl = (c_l[0] + bl[0] * tail_r / math.sin(math.radians(55)), c_l[1] + bl[1] * tail_r / math.sin(math.radians(55)))
    tip_c = (y_open + left_tip_r, left_tip_z)
    foot = (tip_c[0] - left_tip_r * ext_slope * s20, tip_c[1] - left_tip_r * s20)
    tip_m = (tip_c[0] - left_tip_r * math.cos(math.radians(55)), tip_c[1] - left_tip_r * math.sin(math.radians(55)))
    keep_left = cq.Workplane('YZ', origin=(-75, 0, 0)).moveTo(50, 140).lineTo(50, left_tip_z).lineTo(y_open, left_tip_z).threePointArc(tip_m, foot).lineTo(c_l[0] - L * t_l[0], c_l[1] - L * t_l[1]).threePointArc((cl[0] - bl[0] * tail_r, cl[1] - bl[1] * tail_r), (c_l[0], c_l[1] - L)).lineTo(skirt_y, 30).lineTo(90, 30).lineTo(90, 140).close().extrude(27)
    trim = cq.Workplane('XY').box(200, skirt_y - 40, 200, centered=(True, False, True)).translate((0, 40, 100))
    trim = trim.cut(keep_top).cut(keep_left)
    model.add('trim', side(trim), op='cut')
    dg = math.atan2(hip_z - knee_z, hip_x - knee_x)
    diag_plane = cq.Plane(origin=(0, 0, 77.472), xDir=(math.cos(dg), 0, math.sin(dg)), normal=(-math.sin(dg), 0, math.cos(dg)))
    s_ = lambda x: x / math.cos(dg)
    notch = cq.Workplane(diag_plane).polyline([(s_(diag_x0), 83.0), (s_(diag_x0), skirt_y), (s_(diag_x1), diag_y), (s_(diag_x2), diag_y), (s_(diag_x3), skirt_y), (s_(diag_x3), 83.0)]).close().extrude(4, both=True)
    model.add('diag_notch', side(notch), op='cut')
    plate_in = y_back - wall
    feat = y_cyl(pad_r, plate_in - pad_t, plate_in + 0.2, hip_x, hip_z).union(y_cyl(pad_r, plate_in - pad_t, plate_in + 0.2, knee_x, knee_z))
    for x0, x1, z0, z1 in ribs:
        feat = feat.union(cq.Workplane('XY').box(x1 - x0, plate_in + 0.2 - rib_y, z1 - z0, centered=False).translate((x0, rib_y, z0)))
    f = boss_fillet
    ring_prof = cq.Workplane('XY').moveTo(boss_r, plate_in + 0.05).lineTo(boss_r, plate_in - f).threePointArc((boss_r + f - f * math.sqrt(0.5), plate_in - f + f * math.sqrt(0.5)), (boss_r + f, plate_in)).lineTo(boss_r + f, plate_in + 0.05).close()
    ring = ring_prof.revolve(360, (0, 0, 0), (0, 1, 0))
    for bx, bz in bosses:
        feat = feat.union(y_cyl(boss_r, boss_y, plate_in + 0.2, bx, bz))
        feat = feat.union(ring.translate((bx, 0, bz)))
    feat = feat.intersect(outer)
    model.add('plate_features', side(feat), op='join')
    holes = None
    for bx, bz in bosses:
        h = y_cyl(hole_r, boss_y - 1, cb_y + 0.1, bx, bz).union(y_cyl(cb_r, cb_y, y_back + 1, bx, bz))
        holes = h if holes is None else holes.union(h)
    model.add('screw_holes', side(holes), op='cut')
    model.material('pla', color='#99deef')
    result = model.result()
    return to_build123d(model, 'upper_leg_left_native')


@step(out="../../STEP/parts/upper_leg_left_native_seated.step")
def upper_leg_left_native_seated():
    return make()


if __name__ == "__main__":
    upper_leg_left_native_seated()
