from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    import math
    import cadquery as cq
    cx = 35.1
    cz = 199.769
    plate_w = 28.6
    plate_h = 18.0
    corner_r = 4.5
    y_back = -22.4
    y_front = -18.0
    back_r = 0.7
    bevel_d = 0.5
    bevel_in = 0.2
    rec_x0 = 25.5
    rec_z0 = 205.67
    rec_floor = -19.9
    rec_draft = 12.0
    rec_fillet = 0.6
    hole_dx = 10.25
    hole_dz = 3.25
    cb_d = 4.4
    cb_y = -19.5
    hole_d = 2.2
    boss_r = 2.6
    boss_y = -17.6
    holes = [(cx + sx * hole_dx, cz + sz * hole_dz) for sx in (-1, 1) for sz in (-1, 1)]
    
    def xz(y):
        return cq.Workplane('XZ', origin=(0, y, 0))
    y_mid = y_front - bevel_d
    base = xz(y_mid).center(cx, cz).rect(plate_w, plate_h).extrude(y_mid - y_back)
    base = base.edges('|Y').fillet(corner_r).faces('<Y').edges().fillet(back_r)
    bevel_ang = math.degrees(math.atan(bevel_in / bevel_d))
    front_pl = cq.Plane(origin=(0, y_mid, 0), xDir=(1, 0, 0), normal=(0, 1, 0))
    bevel = cq.Workplane(front_pl).center(cx, -cz).sketch().rect(plate_w, plate_h).vertices().fillet(corner_r).finalize().extrude(bevel_d, taper=bevel_ang)
    body = base.union(bevel)
    body = body.union(xz(boss_y).pushPoints(holes).circle(boss_r).extrude(boss_y - y_front))
    rec_w = 2 * (cx - rec_x0)
    rec_len = 8.0
    for s in (1, -1):
        zc = cz + s * (rec_z0 - cz + rec_len / 2)
        cut = xz(y_front).center(cx, zc).rect(rec_w, rec_len).extrude(y_front - rec_floor, taper=rec_draft)
        body = body.cut(cut)
    floor_edges = [e for e in body.faces(cq.selectors.NearestToPointSelector((cx, rec_floor, cz + 7.5))).edges().vals()]
    floor_edges += [e for e in body.faces(cq.selectors.NearestToPointSelector((cx, rec_floor, cz - 7.5))).edges().vals()]
    floor_edges = [e for e in floor_edges if abs(abs(e.Center().z - cz) - plate_h / 2) > 0.05]
    corner_edges = [e for e in body.edges().vals() if rec_floor < e.Center().y < y_front and abs(abs(e.Center().x - cx) - (cx - rec_x0)) < 0.6 and (abs(abs(e.Center().z - cz) - (rec_z0 - cz)) < 0.6)]
    body = body.newObject(floor_edges + corner_edges).fillet(rec_fillet)
    body = body.cut(xz(cb_y).pushPoints(holes).circle(cb_d / 2).extrude(cb_y - y_back + 1))
    body = body.cut(xz(boss_y + 1).pushPoints(holes).circle(hole_d / 2).extrude(boss_y - y_back + 2))
    from lib.cadquery_adapter import Model
    model = Model()
    model.add('printed_body', body)
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'neck_001_native')


@step(out="../../STEP/parts/neck_001_native.step")
def neck_001_native():
    return make()


if __name__ == "__main__":
    neck_001_native()
