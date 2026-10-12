from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    import cadquery as cq
    from lib.cadquery_adapter import Model
    pack_x0 = -62.6
    pack_x1 = -39.8
    pack_hy = 19.25
    pack_z0 = 75.0
    pack_z1 = 145.6
    pack_edge_r = 2.0
    wall_x0 = -39.3
    wall_x1 = -37.3
    wall_hy = 25.0
    wall_z0 = 141.8
    wall_z1 = 153.17
    ear_x0 = -45.36
    ear_y0 = 23.0
    ear_z0 = 145.8
    ear_z1 = 152.0
    relief_x = -40.1
    relief_y = 17.25
    relief_r = 2.0
    relief_z1 = 149.5
    hole_y = 10.0
    hole_z = 145.0
    hole_r = 1.35
    model = Model()
    pack = cq.Workplane('XY').box(pack_x1 - pack_x0, 2 * pack_hy, pack_z1 - pack_z0, centered=False).translate((pack_x0, -pack_hy, pack_z0)).edges('|Z').fillet(pack_edge_r)
    model.add('pack', pack, op='new', body='pack')
    model.box('wall', length=wall_x1 - wall_x0, width=2 * wall_hy, height=wall_z1 - wall_z0, at=(wall_x0, -wall_hy, wall_z0), centered=False, op='new', body='donor_block')
    for s in (-1, 1):
        model.box(f'ear_{s + 1}', length=wall_x0 - ear_x0, width=wall_hy - ear_y0, height=ear_z1 - ear_z0, at=(ear_x0, min(s * ear_y0, s * wall_hy), ear_z0), centered=False, op='join', body='donor_block')
    for s in (-1, 1):
        model.cylinder(f'pin_relief_{s + 1}', radius=relief_r, height=relief_z1 - wall_z0 + 0.5, at=(relief_x, s * relief_y, wall_z0 - 0.5), op='cut', body='donor_block')
        model.cylinder(f'm25_hole_{s + 1}', radius=hole_r, height=wall_x1 - wall_x0 + 1, at=(wall_x0 - 0.5, s * hole_y, hole_z), direction=(1, 0, 0), op='cut', body='donor_block')
    result = model.result()
    return to_build123d(model, 'npf550_pack_native')


@step(out="../../STEP/parts/npf550_pack_native.step")
def npf550_pack_native():
    return make()


if __name__ == "__main__":
    npf550_pack_native()
