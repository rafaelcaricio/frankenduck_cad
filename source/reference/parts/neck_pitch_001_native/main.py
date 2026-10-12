# neck_pitch_001_native: upper neck link of the Frankenduck (carries the head).
# Native rebuild of the Blender mesh neck_pitch.001, robot world frame, mm, Z up.
# Every dimension below was measured from the mesh (sections, circle fits).
# Raw CadQuery: the part needs region-selected fillets of different radii, a
# spline blend, an intersect and staged cuts.
import math
import cadquery as cq

axis_x = 35.1  # mm X of the neck servo horn axis (axis runs along Y)
axis_z = 235.769  # mm Z of the neck servo horn axis
bolt_r = 7.0  # mm Horn screw bolt-circle radius (both horns and the yaw boss)
hole_d = 3.4  # mm M3 clearance hole diameter
x_min = 21.6  # mm Part X minimum
x_max = 48.6  # mm Part X maximum
z_top = 258.005  # mm Top cut plane of the walls and the boss
z_top_raw = 258.505  # mm Wall outline top before the top cut
wall_corner_r = 9.0  # mm Wall outline corner radius (XZ)
plate_z0 = 251.505  # mm Top plate underside
plate_z1 = 255.505  # mm Top plate top face
plate_y0 = -25.3  # mm Top plate Y minimum
plate_y1 = 21.6  # mm Top plate Y maximum
plate_corner_r = 5.45  # mm Top plate corner radius (XY)
plate_edge_r = 1.8  # mm Top plate top/bottom edge fillet
dw_y0 = 17.6  # mm Driven-side (+Y) wall inner face
dw_y1 = 21.6  # mm Driven-side (+Y) wall outer face
dw_z0 = 222.269  # mm Driven-side wall bottom
dw_edge_r = 1.5  # mm Driven-side wall perimeter fillet (both faces)
dw_pocket_y = 20.3  # mm Driven-side horn pocket floor
dw_pocket_r = 3.2  # mm Driven-side pocket round end radius
dw_pocket_hw = 3.0  # mm Driven-side pocket 45 deg band half width
dw_bore_d = 6.4  # mm Driven-side centre bore diameter
pw_y0 = -25.3  # mm Passive-side (-Y) outer plate outer face
pw_y1 = -22.3  # mm Passive-side outer plate inner face
pw_step_z = 246.481  # mm Passive-side outer plate step level at the X edges
pw_nx0 = 23.9  # mm Passive-side narrow part X minimum
pw_nx1 = 46.3  # mm Passive-side narrow part X maximum
pw_blend_z = 245.217  # mm Z where the step blend meets the narrow side
disc_r = 11.2  # mm Passive-side round end / horn disc radius
pw_top_r = 0.6  # mm Passive-side outer plate top edge fillet
pw_upper_r = 1.2  # mm Passive-side outer plate upper edge fillet (both faces)
disc_y0 = -22.3  # mm Horn disc back face
disc_y1 = -18.8  # mm Horn disc inner face
cap_r = 17.2  # mm Horn disc top cap arc radius
cap_top = 246.231  # mm Horn disc top cap height
cap_blend_r = 3.38  # mm Horn disc cap-to-circle blend radius
disc_edge_r = 0.63  # mm Horn disc inner face edge fillet
pk1_y = -19.95  # mm Disc pocket 1 floor (round end + band, half width = radius)
pk1_r = 4.6  # mm Disc pocket 1 radius
pk2_y = -20.35  # mm Disc pocket 2 floor
pk2_r = 3.1  # mm Disc pocket 2 radius
pk3_y = -20.95  # mm Disc centre recess floor
pk3_r = 3.1  # mm Disc centre recess radius
cb_y = -21.8  # mm Passive-side counterbore floor
cb_d = 6.8  # mm Counterbore diameter (passive side and plate underside)
cb_roof_r = 5.0  # mm Arched roof radius of the top passive counterbore
cb_roof_z = 240.731  # mm Arched roof centre Z (roof top at 245.731)
boss_x = 35.1  # mm Yaw boss centre X
boss_y = 0.0  # mm Yaw boss centre Y
boss_z1 = 256.105  # mm Yaw boss tier step height
boss_lobe_r = 4.2  # mm Yaw boss lobe radius
boss_low_r = 9.0  # mm Yaw boss lower tier centre disc radius
boss_up_r = 8.0  # mm Yaw boss upper tier centre disc radius
plate_cb_z = 254.005  # mm Plate screw counterbore top (bored from below)
c_d0 = 6.0  # mm Centre bore lower diameter
c_z0 = 252.205  # mm Centre bore lower step height
c_d1 = 5.76  # mm Centre bore middle diameter
c_z1 = 254.805  # mm Centre bore upper step height
c_d2 = 6.4  # mm Centre bore upper diameter


def prism_xz(profile, y0, y1):
    # XZ workplane: local x = X, local y = Z, normal -Y; extrude from y1 down to y0
    return profile(cq.Workplane("XZ", origin=(0, y1, 0))).extrude(y1 - y0)


horn_pts = [(axis_x - bolt_r, axis_z), (axis_x + bolt_r, axis_z),
            (axis_x, axis_z - bolt_r), (axis_x, axis_z + bolt_r)]


def band(w, hw, length=25.0):
    # 45 deg band of half width hw from the axis toward -X, -Z
    k = 1 / math.sqrt(2)
    pts = [(axis_x - k * hw, axis_z + k * hw), (axis_x + k * hw, axis_z - k * hw),
           (axis_x + k * hw - k * length, axis_z - k * hw - k * length),
           (axis_x - k * hw - k * length, axis_z + k * hw - k * length)]
    return w.polyline(pts).close()


def slot_cut(y0, y1, r, hw):
    a = prism_xz(lambda w: w.center(axis_x, axis_z).circle(r), y0, y1)
    return a.union(prism_xz(lambda w: band(w, hw), y0, y1))


# top plate
plate = (cq.Workplane("XY", origin=(0, 0, plate_z0))
         .center((x_min + x_max) / 2, (plate_y0 + plate_y1) / 2)
         .rect(x_max - x_min, plate_y1 - plate_y0).extrude(plate_z1 - plate_z0)
         .edges("|Z").fillet(plate_corner_r)
         .faces(">Z or <Z").edges().fillet(plate_edge_r))

# driven-side (+Y) wall
dwall = prism_xz(lambda w: w.center((x_min + x_max) / 2, (dw_z0 + z_top_raw) / 2)
                 .rect(x_max - x_min, z_top_raw - dw_z0), dw_y0, dw_y1)
dwall = dwall.edges("|Y").fillet(wall_corner_r).faces(">Y or <Y").edges().fillet(dw_edge_r)

# passive-side (-Y) outer plate
cxl, cxr = x_min + wall_corner_r, x_max - wall_corner_r
czc = z_top_raw - wall_corner_r
c45 = wall_corner_r * math.cos(math.pi / 4)
blend_l = [(x_min, pw_step_z), (22.43, 246.096), (23.07, 245.803), (23.541, 245.559),
           (23.801, 245.364), (pw_nx0, pw_blend_z)]  # step blend, mesh points
blend_r = [(2 * axis_x - x, z) for x, z in reversed(blend_l)]


def pw_profile(w):
    w = w.moveTo(x_min, czc).lineTo(*blend_l[0]).spline(blend_l[1:], includeCurrent=True)
    w = w.lineTo(pw_nx0, axis_z).threePointArc((axis_x, axis_z - disc_r), (pw_nx1, axis_z))
    w = w.lineTo(*blend_r[0]).spline(blend_r[1:], includeCurrent=True).lineTo(x_max, czc)
    w = w.threePointArc((cxr + c45, czc + c45), (cxr, z_top_raw)).lineTo(cxl, z_top_raw)
    w = w.threePointArc((cxl - c45, czc + c45), (x_min, czc))
    return w.close()


top_cut = cq.Workplane("XY", origin=(0, 0, z_top)).rect(200, 200).extrude(5)
upper_edges = cq.selectors.BoxSelector((0, -40, pw_step_z + 0.05), (100, 40, 300), boundingbox=False)
pwall = prism_xz(pw_profile, pw_y0, pw_y1)
pwall = pwall.faces("<Y").edges(upper_edges).fillet(pw_upper_r)
pwall = pwall.faces(">Y").edges(upper_edges).fillet(pw_upper_r)
pwall = pwall.cut(top_cut)
pwall = pwall.faces(">Z").edges("|X").fillet(pw_top_r)

# passive horn disc: R11.2 circle capped by an R17.2 arc
disc = prism_xz(lambda w: w.center(axis_x, axis_z).circle(disc_r), disc_y0, disc_y1)
disc = disc.intersect(prism_xz(lambda w: w.center(axis_x, cap_top - cap_r).circle(cap_r), disc_y0 - 1, disc_y1 + 1))
disc = disc.edges("|Y").fillet(cap_blend_r).faces(">Y").edges().fillet(disc_edge_r)

body = plate.union(dwall).union(pwall).union(disc)


# yaw servo boss: two tiers, centre disc plus four lobes
def boss_tier(z0, z1, rc):
    t = cq.Workplane("XY", origin=(0, 0, z0)).center(boss_x, boss_y).circle(rc).extrude(z1 - z0)
    lobes = [(boss_x - bolt_r, boss_y), (boss_x + bolt_r, boss_y), (boss_x, boss_y - bolt_r), (boss_x, boss_y + bolt_r)]
    return t.union(cq.Workplane("XY", origin=(0, 0, z0)).pushPoints(lobes).circle(boss_lobe_r).extrude(z1 - z0))


body = body.union(boss_tier(plate_z1, boss_z1, boss_low_r)).union(boss_tier(boss_z1, z_top, boss_up_r))
body = body.cut(top_cut)

# driven side: horn pocket, centre bore, screw holes
body = body.cut(slot_cut(dw_y0 - 1, dw_pocket_y, dw_pocket_r, dw_pocket_hw))
body = body.cut(prism_xz(lambda w: w.center(axis_x, axis_z).circle(dw_bore_d / 2), dw_pocket_y - 0.1, dw_y1 + 1))
body = body.cut(prism_xz(lambda w: w.pushPoints(horn_pts).circle(hole_d / 2), dw_y0 - 1, dw_y1 + 1))

# passive side: stepped disc pockets, screw holes, counterbores (top one has an arched roof)
body = body.cut(slot_cut(pk1_y, disc_y1 + 1, pk1_r, pk1_r))
body = body.cut(slot_cut(pk2_y, disc_y1 + 1, pk2_r, pk2_r))
body = body.cut(prism_xz(lambda w: w.center(axis_x, axis_z).circle(pk3_r), pk3_y, disc_y1 + 1))
body = body.cut(prism_xz(lambda w: w.pushPoints(horn_pts).circle(hole_d / 2), pw_y0 - 1, disc_y1 + 1))
cbores = prism_xz(lambda w: w.pushPoints(horn_pts).circle(cb_d / 2), pw_y0 - 1, cb_y)
roof = prism_xz(lambda w: w.center(axis_x, cb_roof_z).circle(cb_roof_r), pw_y0 - 1, cb_y)
top_box = prism_xz(lambda w: w.center(axis_x, axis_z + bolt_r).rect(cb_d + 1, cb_d + 1), pw_y0 - 1, cb_y)
body = body.cut(cbores.cut(top_box.cut(roof)))

# top plate: yaw horn screw holes with counterbores from below, stepped centre bore
boss_pts = [(boss_x - bolt_r, boss_y), (boss_x + bolt_r, boss_y), (boss_x, boss_y - bolt_r), (boss_x, boss_y + bolt_r)]
body = body.cut(cq.Workplane("XY", origin=(0, 0, plate_z0 - 1)).pushPoints(boss_pts).circle(hole_d / 2).extrude(10))
body = body.cut(cq.Workplane("XY", origin=(0, 0, plate_z0 - 1)).pushPoints(boss_pts).circle(cb_d / 2).extrude(plate_cb_z - plate_z0 + 1))
body = body.cut(cq.Workplane("XY", origin=(boss_x, boss_y, plate_z0 - 1)).circle(c_d0 / 2).extrude(c_z0 - plate_z0 + 1))
body = body.cut(cq.Workplane("XY", origin=(boss_x, boss_y, plate_z0 - 1)).circle(c_d1 / 2).extrude(10))
body = body.cut(cq.Workplane("XY", origin=(boss_x, boss_y, c_z1)).circle(c_d2 / 2).extrude(10))

from source_api import Model

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model = Model()
model.add("printed_body", body)
model.material("pla", color="#8a8a90")
result = model.result()

