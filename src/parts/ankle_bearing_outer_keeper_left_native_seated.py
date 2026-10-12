from __future__ import annotations
from cadgen import step
from lib.cadquery_adapter import to_build123d


def make():
    import cadquery as cq
    from lib.cadquery_adapter import Model
    SIDE = 1
    axis_x = -42.899
    axis_z = 3.975
    y_inner = 38.905
    y_outer = 40.505
    outer_r = 13.15
    chamfer = 0.2
    bore_r = 6.9
    crush_in_r = 8.7
    crush_out_r = 9.05
    crush_taper = 0.05
    crush_h = 0.1
    screw_offset = 11.0
    screw_r = 1.1
    
    def ring_profile():
        pts = [(bore_r, y_inner), (outer_r - chamfer, y_inner), (outer_r, y_inner + chamfer), (outer_r, y_outer), (crush_out_r, y_outer), (crush_out_r - crush_taper, y_outer + crush_h), (crush_in_r + crush_taper, y_outer + crush_h), (crush_in_r, y_outer), (bore_r, y_outer)]
        vs = [cq.Vector(axis_x + r, SIDE * y, axis_z) for r, y in pts]
        face = cq.Face.makeFromWires(cq.Wire.makePolygon(vs, close=True))
        return cq.Solid.revolve(face, 360, cq.Vector(axis_x, 0, axis_z), cq.Vector(axis_x, 1, axis_z))
    model = Model()
    model.add('ring', ring_profile())
    for n, dx in (('a', screw_offset), ('b', -screw_offset)):
        model.cylinder(f'screw_hole_{n}', radius=screw_r, height=y_outer - y_inner + 2, at=(axis_x + dx, SIDE * (y_inner - 1), axis_z), direction=(0, SIDE, 0), op='cut')
    model.material('pla', color='#f96c0a')
    result = model.result()
    return to_build123d(model, 'ankle_bearing_outer_keeper_left_native')


@step(out="../../STEP/parts/ankle_bearing_outer_keeper_left_native_seated.step")
def ankle_bearing_outer_keeper_left_native_seated():
    return make()


if __name__ == "__main__":
    ankle_bearing_outer_keeper_left_native_seated()
