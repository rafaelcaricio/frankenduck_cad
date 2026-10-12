import cadquery as cq

# soft_mouth_top, native rebuild of the Blender mesh (world frame, mm).
# Built in a local frame (world rotated -tilt about +Y around the pivot), then
# rotated back and cut by the flat world-vertical back face.
piv_x = 91.5  # mm Local-frame pivot X (world)
piv_z = 270.0  # mm Local-frame pivot Z (world)
tilt = 0.5  # deg Rotation about +Y, local -> world
back_x = 69.77176  # mm World X of the flat back cut
z_top = 2.43144  # mm Top face height (local)
z_under = 1.34218  # mm Plate underside height (local)
z_bot = -1.61846  # mm Wall and lip bottom height (local)
wall_kx = 0.069763  # mm/mm Wall outer face slope in plan (y = kx*x + kz*z + c)
wall_kz = 0.05303  # mm/mm Wall outer face draft (3.04 deg)
wall_c = 57.71466  # mm Wall outer face offset at x=0, z=0
wall_in_kx = 0.07  # mm/mm Wall inner face slope in plan (vertical face)
wall_in_c = 55.672  # mm Wall inner face offset at x=0
lip_kz = 0.125212  # mm/mm Lip outer face draft (7.14 deg)
lip_c = 22.03783  # mm Lip outer face X at z=0
lip_in_x = 20.1213  # mm Lip inner face X (vertical face)
r_edge = 0.945  # mm Radius of every edge round
r_in_corner = 2.0245  # mm Inner vertical corner radius
x_ext = -26.0  # mm Local back extension, removed by the back cut


def y_out(x, z):
    return wall_kx * x + wall_kz * z + wall_c


def x_lip(z):
    return lip_kz * z + lip_c


def quad(z):
    xf = x_lip(z)
    return [(x_ext, -y_out(x_ext, z), z), (xf, -y_out(xf, z), z),
            (xf, y_out(xf, z), z), (x_ext, y_out(x_ext, z), z)]


def planar_prism(qb, qt):
    v = lambda p: cq.Vector(*p)
    faces = [cq.Face.makeFromWires(cq.Wire.makePolygon([v(p) for p in qb], close=True)),
             cq.Face.makeFromWires(cq.Wire.makePolygon([v(p) for p in qt], close=True))]
    for i in range(4):
        j = (i + 1) % 4
        faces.append(cq.Face.makeFromWires(
            cq.Wire.makePolygon([v(qb[i]), v(qb[j]), v(qt[j]), v(qt[i])], close=True)))
    return cq.Solid.makeSolid(cq.Shell.makeShell(faces)).fix()


# Outer body: plate plus walls and lip, drafted outer faces, rounded outer edges.
outer = cq.Workplane("XY").add(planar_prism(quad(z_bot), quad(z_top)))
outer = (outer.edges(cq.selectors.BoxSelector((x_lip(z_bot) - 0.5, -70, z_bot - 0.1),
                                              (x_lip(z_top) + 0.5, 70, z_top + 0.1)))
         .edges("not(|Y)").fillet(r_edge))
outer = (outer.faces(">Z").edges(cq.selectors.BoxSelector((x_ext + 0.5, -70, z_top - 0.01),
                                                          (30, 70, z_top + 0.01))).fillet(r_edge))
outer = (outer.faces("<Z").edges(cq.selectors.BoxSelector((x_ext + 0.5, -70, z_bot - 0.01),
                                                          (30, 70, z_bot + 0.01))).fillet(r_edge))

# Inner body: slab with the underside pocket, rounded pocket corners, ceiling and mouth.
y_in = lambda x: wall_in_kx * x + wall_in_c
pocket = (cq.Workplane("XY").workplane(offset=z_bot - 2)
          .polyline([(x_ext - 2, -y_in(x_ext - 2)), (lip_in_x, -y_in(lip_in_x)),
                     (lip_in_x, y_in(lip_in_x)), (x_ext - 2, y_in(x_ext - 2))]).close()
          .extrude(z_under - (z_bot - 2)))
pocket = pocket.edges("|Z").edges(">X").fillet(r_in_corner)
slab = (cq.Workplane("XY").box(80, 160, z_top + 1 - z_bot, centered=(True, True, False))
        .translate((0, 0, z_bot)).cut(pocket))
slab = slab.edges(cq.selectors.BoxSelector((x_ext + 0.5, -59, z_under - 0.01),
                                           (25, 59, z_under + 0.01))).fillet(r_edge)
slab = slab.edges(cq.selectors.BoxSelector((x_ext + 0.5, -59, z_bot - 0.01),
                                           (25, 59, z_bot + 0.01))).fillet(r_edge)

# Intersection gives the lip's overlapping bottom rounds exactly as in the mesh.
part = outer.intersect(slab)
part = part.rotate((0, 0, 0), (0, 1, 0), tilt).translate((piv_x, 0, piv_z))
back_cutter = cq.Workplane("XY").box(10, 200, 200).translate((back_x - 5, 0, piv_z))
from steve import Model

# Flexible print: generic FDM TPU; exact grade and hardness remain unverified.
model = Model()
model.add("printed_body", part.cut(back_cutter))
model.material("fdm-tpu", color="#f79507")
result = model.result()

