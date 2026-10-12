from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    import cadquery as cq
    piv_x = 91.5
    piv_z = 270.0
    tilt = 0.5
    back_x = 69.77176
    z_top = 2.43144
    z_under = 1.34218
    z_bot = -1.61846
    wall_kx = 0.069763
    wall_kz = 0.05303
    wall_c = 57.71466
    wall_in_kx = 0.07
    wall_in_c = 55.672
    lip_kz = 0.125212
    lip_c = 22.03783
    lip_in_x = 20.1213
    r_edge = 0.945
    r_in_corner = 2.0245
    x_ext = -26.0
    
    def y_out(x, z):
        return wall_kx * x + wall_kz * z + wall_c
    
    def x_lip(z):
        return lip_kz * z + lip_c
    
    def quad(z):
        xf = x_lip(z)
        return [(x_ext, -y_out(x_ext, z), z), (xf, -y_out(xf, z), z), (xf, y_out(xf, z), z), (x_ext, y_out(x_ext, z), z)]
    
    def planar_prism(qb, qt):
        v = lambda p: cq.Vector(*p)
        faces = [cq.Face.makeFromWires(cq.Wire.makePolygon([v(p) for p in qb], close=True)), cq.Face.makeFromWires(cq.Wire.makePolygon([v(p) for p in qt], close=True))]
        for i in range(4):
            j = (i + 1) % 4
            faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon([v(qb[i]), v(qb[j]), v(qt[j]), v(qt[i])], close=True)))
        return cq.Solid.makeSolid(cq.Shell.makeShell(faces)).fix()
    outer = cq.Workplane('XY').add(planar_prism(quad(z_bot), quad(z_top)))
    outer = outer.edges(cq.selectors.BoxSelector((x_lip(z_bot) - 0.5, -70, z_bot - 0.1), (x_lip(z_top) + 0.5, 70, z_top + 0.1))).edges('not(|Y)').fillet(r_edge)
    outer = outer.faces('>Z').edges(cq.selectors.BoxSelector((x_ext + 0.5, -70, z_top - 0.01), (30, 70, z_top + 0.01))).fillet(r_edge)
    outer = outer.faces('<Z').edges(cq.selectors.BoxSelector((x_ext + 0.5, -70, z_bot - 0.01), (30, 70, z_bot + 0.01))).fillet(r_edge)
    y_in = lambda x: wall_in_kx * x + wall_in_c
    pocket = cq.Workplane('XY').workplane(offset=z_bot - 2).polyline([(x_ext - 2, -y_in(x_ext - 2)), (lip_in_x, -y_in(lip_in_x)), (lip_in_x, y_in(lip_in_x)), (x_ext - 2, y_in(x_ext - 2))]).close().extrude(z_under - (z_bot - 2))
    pocket = pocket.edges('|Z').edges('>X').fillet(r_in_corner)
    slab = cq.Workplane('XY').box(80, 160, z_top + 1 - z_bot, centered=(True, True, False)).translate((0, 0, z_bot)).cut(pocket)
    slab = slab.edges(cq.selectors.BoxSelector((x_ext + 0.5, -59, z_under - 0.01), (25, 59, z_under + 0.01))).fillet(r_edge)
    slab = slab.edges(cq.selectors.BoxSelector((x_ext + 0.5, -59, z_bot - 0.01), (25, 59, z_bot + 0.01))).fillet(r_edge)
    part = outer.intersect(slab)
    part = part.rotate((0, 0, 0), (0, 1, 0), tilt).translate((piv_x, 0, piv_z))
    back_cutter = cq.Workplane('XY').box(10, 200, 200).translate((back_x - 5, 0, piv_z))
    from lib.cadquery_adapter import Model
    model = Model()
    model.add('printed_body', part.cut(back_cutter))
    model.material('fdm-tpu', color='#f79507')
    result = model.result()
    return to_build123d(model, 'soft_mouth_top_native')


@step(out="../../STEP/parts/soft_mouth_top_native.step")
def soft_mouth_top_native():
    return make()


if __name__ == "__main__":
    soft_mouth_top_native()
