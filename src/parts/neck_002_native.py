from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    import math
    import cadquery as cq
    x_min = 20.8
    x_max = 49.4
    z_min = 187.019
    z_max = 212.519
    corner_r = 4.5
    y_face = 14.4
    y_pad = 14.8
    y_floor = 15.9
    y_top = 20.3
    edge_r = 0.65
    hole_x0 = 24.85
    hole_x1 = 45.35
    hole_z0 = 192.769
    hole_z1 = 206.769
    hole_r = 1.1
    cb_r = 2.2
    cb_y = 16.1
    boss_r = 2.6
    slot_x0 = 27.7
    slot_x1 = 42.5
    wall_draft = 8.93
    bar_z = 199.769
    bar_hw = 1.437
    bar_draft = 22.5
    root_r = 0.75
    tp_hx = 5.322
    tp_hz = 5.689
    tp_y = 15.75
    xc = (x_min + x_max) / 2
    zc = (z_min + z_max) / 2
    pts = [(hole_x0, hole_z0), (hole_x0, hole_z1), (hole_x1, hole_z0), (hole_x1, hole_z1)]
    
    def prism_xz(profile, y0, y1):
        return profile(cq.Workplane('XZ', origin=(0, y1, 0))).extrude(y1 - y0)
    body = prism_xz(lambda w: w.center(xc, zc).rect(x_max - x_min, z_max - z_min), y_face, y_top)
    body = body.edges('|Y').fillet(corner_r).faces('>Y or <Y').edges().fillet(edge_r)
    facing = prism_xz(lambda w: w.center(xc, zc).rect(60, 60), y_face - 1, y_pad)
    facing = facing.cut(prism_xz(lambda w: w.pushPoints(pts).circle(boss_r), y_face - 1, y_pad))
    body = body.cut(facing)
    kw = math.tan(math.radians(wall_draft))
    kb = math.tan(math.radians(bar_draft))
    y_lo = y_pad - 0.8
    trap = cq.Workplane('XY', origin=(0, 0, z_min - 1)).polyline([(slot_x0 - kw * (y_pad - y_lo), y_lo), (slot_x1 + kw * (y_pad - y_lo), y_lo), (slot_x1 - kw * (y_floor - y_pad), y_floor), (slot_x0 + kw * (y_floor - y_pad), y_floor)]).close().extrude(z_max - z_min + 2)
    bar = cq.Workplane('YZ', origin=(x_min - 1, 0, 0)).polyline([(y_lo, bar_z - bar_hw + kb * (y_pad - y_lo)), (y_lo, bar_z + bar_hw - kb * (y_pad - y_lo)), (y_floor + 0.5, bar_z + bar_hw + kb * (y_floor + 0.5 - y_pad)), (y_floor + 0.5, bar_z - bar_hw - kb * (y_floor + 0.5 - y_pad))]).close().extrude(x_max - x_min + 2)
    pocket = trap.cut(bar)
    pocket = pocket.edges(cq.selectors.BoxSelector((0, y_pad - 0.3, z_min - 0.5), (100, 30, z_max + 0.5), boundingbox=False)).fillet(root_r)
    body = body.cut(pocket)
    for z0, z1 in ((z_min, bar_z - tp_hz), (bar_z + tp_hz, z_max)):
        body = body.union(cq.Workplane('XY', origin=(0, 0, z0)).center(35.1, (tp_y + y_floor + 0.05) / 2).rect(2 * tp_hx, y_floor + 0.05 - tp_y).extrude(z1 - z0))
    body = body.cut(prism_xz(lambda w: w.pushPoints(pts).circle(hole_r), y_face - 1, y_top + 1))
    body = body.cut(prism_xz(lambda w: w.pushPoints(pts).circle(cb_r), cb_y, y_top + 1))
    from lib.cadquery_adapter import Model
    model = Model()
    model.add('printed_body', body)
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'neck_002_native')


@step(out="../../STEP/parts/neck_002_native.step")
def neck_002_native():
    return make()


if __name__ == "__main__":
    neck_002_native()
