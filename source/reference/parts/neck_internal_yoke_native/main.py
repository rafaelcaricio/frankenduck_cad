"""neck_internal_yoke_native: Frankenduck neck yoke (stage B, the canonical part), robot world frame, Z up, mm.

Native port of cad_blender/code_cad/neck_internal_yoke/build.py --stage B (build123d), same coordinates.
The yoke carries the lower neck servo on the trunk deck; horn axis X 35.1, Z 163.769 along Y.
Construction (as the source):
  1. nine Y bands, each ONE intersection: plate plan prism (+ disc strip or gusset triangle) with a Y-extruded
     side profile made by exact 2D booleans (plate strip + tangent shoulder sketch - cable channel + disc);
     bands touch only along Y planes;
  2. cuts: anchor bores and front wells, horn bores, head lanes, driven centre pocket and channel,
     passive pockets and traces, passive rim rounds (outer sector, horn-face 360 deg);
  3. edge breaks on the passive rib next to the cable channel (R1.2 corners, R0.8 floor), front wells re-cut.
No heat-set insert pockets: the anchors are plain M3 through bores (the inserts sit in the trunk deck).
Raw CadQuery is used for the band solids, the 45 deg slots, the rim-round revolves and the edge breaks.
"""
import math

import cadquery as cq
from source_api import Model

# ---------------------------------------------------------------- horn axis and plate
axis_x = 35.1  # mm Lower-neck horn axis X
axis_z = 163.769  # mm Lower-neck horn axis Z
plate_z0 = 164.45  # mm Deck contact face
plate_z1 = 168.45  # mm Seat plane (screw heads, IMU bridge contact)
plate_x_rear = -20.5  # mm Plate rear edge
plate_x_front = 25.0  # mm Plate front edge (rail front)
plate_half_width = 34.0  # mm Plate half width
plate_corner_r = 7.0  # mm Plate outer corner radius
opening_x_rear = -10.5  # mm Inner opening rear edge
opening_half_width = 22.0  # mm Inner opening half width
opening_corner_r = 5.0  # mm Inner opening corner radius
opening_front_arc_x = 10.0  # mm Inner opening front arc centre X
opening_front_arc_absy = 17.0  # mm Inner opening front arc centre |Y|
opening_front_arc_r = 5.0  # mm Inner opening front arc radius
driven_inner_y = 17.6  # mm Plate inner edge, driven side
passive_inner_y = -20.6  # mm Plate inner edge, passive side
notch_x = -0.2  # mm Edge notch centre X
notch_absy = 33.875  # mm Edge notch centre |Y|
notch_r = 2.7  # mm Edge notch radius
# ---------------------------------------------------------------- anchors
anchors_rear = [(-16.0, 28.0), (-16.0, -28.0)]  # mm Rear anchor screw axes (X, Y)
anchors_front = [(18.0, 28.0), (18.0, -28.0)]  # mm Front anchor screw axes (X, Y)
anchor_bore_r = 1.7  # mm M3 anchor through bore radius
front_well_r = 3.1  # mm Front screw-head well radius
# ---------------------------------------------------------------- horn discs
driven_face_y = 17.6  # mm Driven horn face plane
driven_thickness = 6.5  # mm Driven disc thickness (17.6 .. 24.1)
driven_disc_r = 12.5  # mm Driven disc radius
passive_face_y = -18.8  # mm Passive horn seat face plane
passive_thickness = 6.5  # mm Passive cap thickness (-18.8 .. -25.3)
passive_disc_r = 11.2  # mm Passive cap radius
passive_face_rim_round = 0.65  # mm Passive seat-face rim round
horn_pcd_r = 7.0  # mm Horn screw pitch radius (PCD 14)
horn_bore_r = 1.7  # mm Horn screw bore radius
driven_head_seat_y = 21.6  # mm Driven M3x6 head seat plane (4.0 mm grip)
head_lane_r = 3.4  # mm Head and driver lane radius
head_lane_abs_y = 40.0  # mm Head lane outer end |Y|
driven_centre_pocket_r = 3.2  # mm Driven centre pocket radius
driven_centre_pocket_depth = 2.7  # mm Driven centre pocket depth from the horn face
driven_channel_half_width = 3.0  # mm Driven centre channel half width
driven_channel_length = 30.0  # mm Driven centre channel length
passive_head_seat_y = -21.8  # mm Passive head seat plane
passive_outer_rim_round = 0.65  # mm Passive outer rim round (sector only)
rim_sector_a1 = 178.75  # deg Outer rim round sector start (from +X toward +Z)
rim_sector_a2 = 229.5  # deg Outer rim round sector end
passive_pocket_big_r = 4.6  # mm Passive big pocket radius
passive_pocket_big_depth = 1.15  # mm Passive big pocket depth
passive_pocket_small_r = 3.1  # mm Passive small pocket radius
passive_pocket_small_depth = 2.15  # mm Passive small pocket depth
passive_trace_wide_half = 4.6  # mm Passive wide trace half width
passive_trace_wide_depth = 1.15  # mm Passive wide trace depth
passive_trace_narrow_half = 3.1  # mm Passive narrow trace half width
passive_trace_narrow_depth = 1.55  # mm Passive narrow trace depth
passive_trace_length = 22.0  # mm Passive trace length
driven_web = [(22.0547, 164.45), (24.8391, 164.45), (25.9792, 154.641), (28.736, 153.0103)]  # mm Driven web Bezier (X, Z)
# ---------------------------------------------------------------- stage B shoulder and gusset
deck_clear_x = 24.3  # mm Below the plate, new material only at X >= this
sh_x_back = 6.0  # mm Shoulder rear end
sh_ramp_x0 = 8.5  # mm Ramp line meets the plate top here
sh_ramp_angle = 33.0  # deg Shoulder ramp angle
sh_root_fillet_r = 3.0  # mm Plate top to ramp fillet
sh_top_fillet_r = 8.0  # mm Ramp to plateau fillet
sh_plateau_below_disc_top = 0.3  # mm Plateau below the disc top
sh_x_end = 33.5  # mm Plateau end (into the disc)
sh_bottom_z_beyond_rail = 167.3  # mm Shoulder bottom beyond the rail (above the rear head lane)
gusset_rail_y = 27.0  # mm |Y| where the driven gusset leaves the rail front
gusset_tip_x = 31.0  # mm Gusset tip X on the disc outer face
neck_clear_y1 = 21.8  # mm Driven neck-clearance strip end Y (from the horn face)
neck_clear_r = 21.6  # mm Driven neck-clearance cylinder radius about the horn axis
passive_shoulder_y = -23.6  # mm Passive shoulder starts here (servo case keep-out)
passive_gusset_rail_y = 29.3  # mm Passive gusset rail |Y|
passive_gusset_rail_x = 24.5  # mm Passive gusset rail X
# ---------------------------------------------------------------- passive cable channel
ch_inner_u = (23.6, 25.3)  # mm Inner channel band |Y|
ch_leads_u = (25.3, 29.3)  # mm Lead channel band |Y|
ch_rear_top = (9.5, 171.2)  # mm Channel sloped rear end top (X, Z)
ch_inner_floor_x = (12.0, 23.0)  # mm Inner channel flat floor X
ch_leads_floor_x = (12.0, 21.8)  # mm Lead channel flat floor X
ch_inner_exit_x = 27.98  # mm Inner channel exit ramp corner X
ch_leads_exit_x = 26.98  # mm Lead channel exit ramp corner X
ch_exit_blend_r = 4.0  # mm Exit ramp blend radius
rib_chamfer = ((23.0, 175.0), (24.8, 171.4))  # mm Passive outer rib front chamfer line (X, Z)
rib_corner_fillet_r = 1.2  # mm Passive rib vertical corner fillet
rib_floor_fillet_r = 0.8  # mm Passive rib channel floor edge fillet
# ---------------------------------------------------------------- construction extents
band_z0 = 150.0  # mm Plan prism bottom
band_z1 = 190.0  # mm Plan prism top


# ================================================================ geometry helpers (raw CadQuery)
def V(x, y, z):
    return cq.Vector(float(x), float(y), float(z))


def _face(edges):
    return cq.Face.makeFromWires(cq.Wire.assembleEdges(edges))


def seg_face(segs):
    """segs: ('L', p, q) or ('B', [p0..p3]) 3D points, consecutive and closed."""
    edges = []
    for s in segs:
        if s[0] == "L":
            if (V(*s[1]) - V(*s[2])).Length > 1e-9:
                edges.append(cq.Edge.makeLine(V(*s[1]), V(*s[2])))
        else:
            edges.append(cq.Edge.makeBezier([V(*p) for p in s[1]]))
    return _face(edges)


def norm_y(face):
    """orient a face lying in a Y plane with its normal along +Y."""
    return face if face.normalAt().y > 0 else cq.Face(face.wrapped.Reversed())


def ext(face, vec):
    s = cq.Solid.extrudeLinear(face, vec)
    if s.Volume() < 0:
        s = cq.Solid(s.wrapped.Reversed())
    return s


def zext(face, z0, z1):
    return ext(face, V(0, 0, z1 - z0))


def yext(face, y0, y1):
    return ext(face, V(0, y1 - y0, 0))


def cyl_z(x, y, r, z0, z1):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def cyl_y(x, z, r, y0, y1):
    return cq.Solid.makeCylinder(r, abs(y1 - y0), V(x, y0, z), V(0, 1 if y1 > y0 else -1, 0))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def rounded_rect_face(x0, x1, y0, y1, r, z):
    k = r * (1 - 1 / math.sqrt(2))
    e = [cq.Edge.makeLine(V(x0 + r, y0, z), V(x1 - r, y0, z)),
         cq.Edge.makeThreePointArc(V(x1 - r, y0, z), V(x1 - k, y0 + k, z), V(x1, y0 + r, z)),
         cq.Edge.makeLine(V(x1, y0 + r, z), V(x1, y1 - r, z)),
         cq.Edge.makeThreePointArc(V(x1, y1 - r, z), V(x1 - k, y1 - k, z), V(x1 - r, y1, z)),
         cq.Edge.makeLine(V(x1 - r, y1, z), V(x0 + r, y1, z)),
         cq.Edge.makeThreePointArc(V(x0 + r, y1, z), V(x0 + k, y1 - k, z), V(x0, y1 - r, z)),
         cq.Edge.makeLine(V(x0, y1 - r, z), V(x0, y0 + r, z)),
         cq.Edge.makeThreePointArc(V(x0, y0 + r, z), V(x0 + k, y0 + k, z), V(x0 + r, y0, z))]
    return _face(e)


def poly_face_xy(pts, z):
    p = [V(x, y, z) for x, y in pts]
    return _face([cq.Edge.makeLine(p[i], p[(i + 1) % len(p)]) for i in range(len(p))])


def poly_face_xz(pts, y):
    p = [V(x, y, z) for x, z in pts]
    return norm_y(_face([cq.Edge.makeLine(p[i], p[(i + 1) % len(p)]) for i in range(len(p))]))


def rect_xz(x0, x1, z0, z1, y):
    return poly_face_xz([(x0, z0), (x1, z0), (x1, z1), (x0, z1)], y)


def circle_xz(cx, cz, r, y):
    return norm_y(cq.Face.makeFromWires(cq.Wire.makeCircle(r, V(cx, y, cz), V(0, 1, 0))))


def single_face(shape):
    fs = [norm_y(f) for f in shape.Faces()]
    if len(fs) == 1:
        return fs[0]
    u = fs[0].fuse(*fs[1:]).clean()
    fs = [norm_y(f) for f in u.Faces()]
    assert len(fs) == 1, f"profile is {len(fs)} faces"
    return fs[0]


def arc_mid(c, r, a, b):
    m = ((a[0] + b[0]) / 2 - c[0], (a[1] + b[1]) / 2 - c[1])
    n = math.hypot(*m)
    return (c[0] + r * m[0] / n, c[1] + r * m[1] / n)


_plan_cache = []


def plan_prism():
    """plate plan (outline, inner opening, edge notches) extruded over Z band_z0..band_z1 (built once)."""
    if _plan_cache:
        return _plan_cache[0]
    z0, z1 = band_z0, band_z1
    hw, ax, orr = plate_half_width, opening_front_arc_x, opening_corner_r
    outer = zext(rounded_rect_face(plate_x_rear, plate_x_front, -hw, hw, plate_corner_r, z0), z0, z1)
    zo0, zo1 = z0 - 1, z1 + 1
    rear = zext(rounded_rect_face(opening_x_rear, ax + orr, -opening_half_width, opening_half_width, orr, zo0), zo0, zo1)
    rear = rear.intersect(box(opening_x_rear - 1, ax, -opening_half_width - 1, opening_half_width + 1, zo0, zo1))
    tools = rear.fuse(cyl_z(ax, opening_front_arc_absy, opening_front_arc_r, zo0, zo1),
                      cyl_z(ax, -opening_front_arc_absy, opening_front_arc_r, zo0, zo1),
                      box(ax, 80, passive_inner_y, driven_inner_y, zo0, zo1))
    body = outer.cut(tools)
    for sy in (1, -1):
        body = body.cut(cyl_z(notch_x, sy * notch_absy, notch_r, zo0, zo1))
    _plan_cache.append(body)
    return body


def shoulder_face(disc_r, y):
    """tangent shoulder sketch in the plane Y=y: plate top, root fillet, ramp, top fillet, plateau."""
    zp, zb = plate_z1, plate_z0
    ph = math.radians(sh_ramp_angle)
    n = (-math.sin(ph), math.cos(ph))
    x0, rr, rt = sh_ramp_x0, sh_root_fillet_r, sh_top_fillet_r
    z_top = axis_z + disc_r - sh_plateau_below_disc_top
    xcr = x0 - rr * (1 - math.cos(ph)) / math.sin(ph)
    cr = (xcr, zp + rr)
    tr_plate, tr_line = (xcr, zp), (cr[0] - rr * n[0], cr[1] - rr * n[1])
    xct = x0 + ((z_top - rt - zp) * math.cos(ph) + rt) / math.sin(ph)
    ct = (xct, z_top - rt)
    tt_line, tt_plat = (ct[0] + rt * n[0], ct[1] + rt * n[1]), (xct, z_top)
    p = lambda q: V(q[0], y, q[1])
    xb, xe, xr, zb2 = sh_x_back, sh_x_end, plate_x_front, sh_bottom_z_beyond_rail
    e = [cq.Edge.makeLine(V(xb, y, zb), V(xb, y, zp)),
         cq.Edge.makeLine(V(xb, y, zp), p(tr_plate)),
         cq.Edge.makeThreePointArc(p(tr_plate), p(arc_mid(cr, rr, tr_plate, tr_line)), p(tr_line)),
         cq.Edge.makeLine(p(tr_line), p(tt_line)),
         cq.Edge.makeThreePointArc(p(tt_line), p(arc_mid(ct, rt, tt_line, tt_plat)), p(tt_plat)),
         cq.Edge.makeLine(p(tt_plat), V(xe, y, z_top)),
         cq.Edge.makeLine(V(xe, y, z_top), V(xe, y, zb2)),
         cq.Edge.makeLine(V(xe, y, zb2), V(xr, y, zb2)),
         cq.Edge.makeLine(V(xr, y, zb2), V(xr, y, zb)),
         cq.Edge.makeLine(V(xr, y, zb), V(xb, y, zb))]
    return norm_y(_face(e)), z_top


def web_cut_face(y, x_back=5.0, z_low=140.0):
    """region behind/below the driven web curve."""
    (x0, z0), _, _, (x3, z3) = driven_web
    return norm_y(seg_face([("L", (x_back, y, z0), (x0, y, z0)), ("B", [(px, y, pz) for px, pz in driven_web]),
                            ("L", (x3, y, z3), (x3, y, z_low)), ("L", (x3, y, z_low), (x_back, y, z_low)),
                            ("L", (x_back, y, z_low), (x_back, y, z0))]))


def channel_face(z_plateau, y, band):
    """passive cable channel band: sloped rear end, flat floor on the plate top, exit ramp blended into the plateau."""
    (xf0, xf1), xc = (ch_inner_floor_x, ch_inner_exit_x) if band == "inner" else (ch_leads_floor_x, ch_leads_exit_x)
    xr0, zr0 = ch_rear_top
    zf, rb = plate_z1, ch_exit_blend_r
    beta = math.atan2(z_plateau - zf, xc - xf1)
    t = rb * math.tan(beta / 2)
    t1 = (xc - t * math.cos(beta), z_plateau - t * math.sin(beta))
    t2 = (xc + t, z_plateau)
    mid = arc_mid((t2[0], z_plateau - rb), rb, t1, t2)
    xe, ztop = 60.0, 200.0
    p = lambda x, z: V(x, y, z)
    e = [cq.Edge.makeLine(p(xr0, ztop), p(xr0, zr0)), cq.Edge.makeLine(p(xr0, zr0), p(xf0, zf)),
         cq.Edge.makeLine(p(xf0, zf), p(xf1, zf)), cq.Edge.makeLine(p(xf1, zf), p(*t1)),
         cq.Edge.makeThreePointArc(p(*t1), p(*mid), p(*t2)), cq.Edge.makeLine(p(*t2), p(xe, z_plateau)),
         cq.Edge.makeLine(p(xe, z_plateau), p(xe, ztop)), cq.Edge.makeLine(p(xe, ztop), p(xr0, ztop))]
    return norm_y(_face(e))


def chamfer_face(y):
    (x0, z0), (x1, z1) = rib_chamfer
    k = (z1 - z0) / (x1 - x0)
    return poly_face_xz([(x0, z0), (60.0, z0 + k * (60.0 - x0)), (60.0, 200.0), (x0, 200.0)], y)


def band_profile(side, y, cap_r=None, channel=None, with_disc=True, chamfer=False):
    disc_r = driven_disc_r if side > 0 else passive_disc_r
    prof = rect_xz(plate_x_rear - 1.0, plate_x_front, plate_z0, plate_z1, y)
    sh, z_top = shoulder_face(disc_r, y)
    if cap_r:
        sh = single_face(sh.intersect(circle_xz(axis_x, axis_z, cap_r, y)))
    if channel:
        sh = single_face(sh.cut(channel_face(z_top, y, channel)))
    if chamfer:
        sh = single_face(sh.cut(chamfer_face(y)))
    prof = prof.fuse(sh)
    if with_disc:
        disc = circle_xz(axis_x, axis_z, disc_r, y)
        if side > 0:
            disc = single_face(disc.cut(rect_xz(axis_x - disc_r - 1.0, deck_clear_x, axis_z - disc_r - 1.0, plate_z0, y)))
            disc = single_face(disc.cut(web_cut_face(y)))
        prof = prof.fuse(disc)
    return single_face(prof)


def band(side, ya, yb, cap_r=None, channel=None, disc=False, extra=None, chamfer=False):
    """one Y band: plan prism (+ disc strip or gusset) intersected with the Y-extruded side profile."""
    lo, hi = min(ya, yb), max(ya, yb)
    plan = plan_prism()
    y_out = driven_face_y + driven_thickness if side > 0 else passive_face_y - passive_thickness
    if extra == "disc":
        plan = plan.fuse(box(22.0, 50.0, lo, hi, band_z0, band_z1))
    elif extra == "gusset":
        if side > 0:
            tri = [(plate_x_front - 0.5, y_out), (plate_x_front - 0.5, gusset_rail_y), (plate_x_front, gusset_rail_y), (gusset_tip_x, y_out)]
        else:
            tri = [(passive_gusset_rail_x, y_out), (passive_gusset_rail_x, -passive_gusset_rail_y), (gusset_tip_x, y_out)]
        plan = plan.fuse(zext(poly_face_xy(tri, band_z0), band_z0, band_z1))
    plan = plan.intersect(box(-30.0, 60.0, lo, hi, band_z0, band_z1))
    if ya == 0.0 or (side < 0 and ya == passive_face_y):  # crossbar / passive cap strip: no shoulder
        prof = rect_xz(plate_x_rear - 1.0, plate_x_front, plate_z0, plate_z1, ya)
        if disc:
            prof = single_face(prof.fuse(circle_xz(axis_x, axis_z, passive_disc_r, ya)))
    else:
        prof = band_profile(side, ya, cap_r=cap_r, channel=channel, with_disc=disc, chamfer=chamfer)
    return plan.intersect(yext(prof, ya, yb))


def slot45(half_w, length, y0, y1):
    """box from the horn axis toward (+X, +Z)/sqrt2, |perp| <= half_w, Y from y0 to y1."""
    u = (1 / math.sqrt(2), 1 / math.sqrt(2))
    n = (-1 / math.sqrt(2), 1 / math.sqrt(2))
    c = (axis_x, axis_z)
    pts = [(c[0] - n[0] * half_w, c[1] - n[1] * half_w),
           (c[0] - n[0] * half_w + u[0] * length, c[1] - n[1] * half_w + u[1] * length),
           (c[0] + n[0] * half_w + u[0] * length, c[1] + n[1] * half_w + u[1] * length),
           (c[0] + n[0] * half_w, c[1] + n[1] * half_w)]
    return yext(poly_face_xz(pts, y0), y0, y1)


def rim_round_cut(yo, rr, a1, a2, inward=1):
    """material outside an R-round on the passive cap rim (face plane Y=yo), revolved over a1..a2 deg about the
    horn axis. inward +1: cap material at Y > yo; -1: at Y < yo (built mirrored)."""
    if inward < 0:
        return rim_round_cut(-yo, rr, a1, a2, 1).mirror("XZ")
    R, e = passive_disc_r, 0.3
    th = math.radians(a2)
    pt = lambda r, y: V(axis_x + r * math.cos(th), y, axis_z + r * math.sin(th))
    mid = (R - rr + rr * math.cos(math.radians(-45.0)), yo + rr + rr * math.sin(math.radians(-45.0)))
    edges = [cq.Edge.makeLine(pt(R - rr, yo - e), pt(R + e, yo - e)),
             cq.Edge.makeLine(pt(R + e, yo - e), pt(R + e, yo + rr)),
             cq.Edge.makeLine(pt(R + e, yo + rr), pt(R, yo + rr)),
             cq.Edge.makeThreePointArc(pt(R, yo + rr), pt(*mid), pt(R - rr, yo)),
             cq.Edge.makeLine(pt(R - rr, yo), pt(R - rr, yo - e))]
    w = cq.Wire.assembleEdges(edges)
    return cq.Solid.revolve(w, [], a2 - a1, V(axis_x, 0, axis_z), V(axis_x, 1, axis_z))


def rib_edge_breaks(body):
    """fillet the passive rib edges the cable channel leaves (inner face Y = -ch_leads_u[1])."""
    yr, zf = -ch_leads_u[1], plate_z1
    wx, wy = anchors_front[1]

    def sel(fn):
        return [e for e in body.Edges() if fn(e.startPoint(), e.endPoint(), e)]

    vert = lambda a, b: abs(a.x - b.x) < 1e-4 and abs(a.y - b.y) < 1e-4
    off_well = lambda e: math.hypot(e.positionAt(0.5).x - wx, e.positionAt(0.5).y - wy) > front_well_r + 0.05
    corner = lambda a, b, e: vert(a, b) and abs(a.y - yr) < 1e-3 and min(a.z, b.z) > zf - 0.01 and max(a.z, b.z) > zf + 1.0
    floor = lambda a, b, e: (abs(a.y - yr) < 1e-3 and abs(b.y - yr) < 1e-3 and not vert(a, b)
                             and abs(a.z - zf) < 1e-3 and abs(b.z - zf) < 1e-3 and off_well(e))
    for kx, ky in [(round(e.startPoint().x, 3), round(e.startPoint().y, 3)) for e in sel(corner)]:
        for r in (rib_corner_fillet_r, 0.8 * rib_corner_fillet_r, 0.6 * rib_corner_fillet_r, 0.4 * rib_corner_fillet_r):
            cur = [e for e in sel(corner) if abs(e.startPoint().x - kx) < 1e-3 and abs(e.startPoint().y - ky) < 1e-3]
            try:
                nb = body.fillet(r, cur)
                if nb.isValid() and len(nb.Solids()) == 1:
                    body = nb
                    break
            except Exception:
                continue
    es = sel(floor)
    if es:
        body = body.fillet(rib_floor_fillet_r, es)
    return body


# ================================================================ model
model = Model()

# ---------------------------------------------------------------- 1. bands (touch along Y planes only)
y_dr_out = driven_face_y + driven_thickness
y_ps_out = passive_face_y - passive_thickness
model.add("band_d0", band(1, 0.0, driven_face_y), op="new", body="yoke", name="Driven crossbar band (plate)")
model.add("band_d1", band(1, driven_face_y, neck_clear_y1, cap_r=neck_clear_r, disc=True, extra="disc"), op="join", body="yoke",
          name="Driven band in the neck clearance strip")
model.add("band_d2", band(1, neck_clear_y1, y_dr_out, disc=True, extra="disc"), op="join", body="yoke", name="Driven outer disc band")
model.add("band_d3", band(1, y_dr_out, plate_half_width + 0.5, extra="gusset"), op="join", body="yoke", name="Driven rail band with gusset")
model.add("band_p0", band(-1, 0.0, passive_face_y), op="join", body="yoke", name="Passive crossbar band (plate)")
model.add("band_p1", band(-1, passive_face_y, passive_shoulder_y, disc=True, extra="disc"), op="join", body="yoke",
          name="Passive cap band (no shoulder)")
model.add("band_p2", band(-1, passive_shoulder_y, y_ps_out, channel="inner", disc=True, extra="disc"), op="join", body="yoke",
          name="Passive cap band with plug channel")
model.add("band_p3", band(-1, y_ps_out, -ch_leads_u[1], channel="leads", extra="gusset"), op="join", body="yoke",
          name="Passive lead-channel band with gusset")
model.add("band_p4", band(-1, -ch_leads_u[1], -plate_half_width - 0.5, chamfer=True), op="join", body="yoke",
          name="Passive outer rib band")

# ---------------------------------------------------------------- 2. cuts
for i, (x, y) in enumerate(anchors_rear + anchors_front):
    model.cylinder(f"anchor_bore_{i}", anchor_bore_r, band_z1 - plate_z0 + 1.0, at=(x, y, plate_z0 - 1.0), op="cut", body="yoke",
                   name=f"Anchor M3 bore {i}")
for i, (x, y) in enumerate(anchors_front):
    model.cylinder(f"front_well_{i}", front_well_r, band_z1 - plate_z1, at=(x, y, plate_z1), op="cut", body="yoke",
                   name=f"Front screw well {i}")
horn_points = [(axis_x - horn_pcd_r, axis_z), (axis_x + horn_pcd_r, axis_z), (axis_x, axis_z - horn_pcd_r), (axis_x, axis_z + horn_pcd_r)]
for i, (x, z) in enumerate(horn_points):
    model.cylinder(f"driven_bore_{i}", horn_bore_r, driven_thickness + 2.0, at=(x, driven_face_y - 1.0, z), direction=(0, 1, 0),
                   op="cut", body="yoke", name=f"Driven horn bore {i}")
    model.cylinder(f"driven_lane_{i}", head_lane_r, head_lane_abs_y - driven_head_seat_y, at=(x, driven_head_seat_y, z),
                   direction=(0, 1, 0), op="cut", body="yoke", name=f"Driven head lane {i}")
model.cylinder("driven_pocket", driven_centre_pocket_r, driven_centre_pocket_depth + 1.0, at=(axis_x, driven_face_y - 1.0, axis_z),
               direction=(0, 1, 0), op="cut", body="yoke", name="Driven centre pocket")
model.add("driven_channel", slot45(driven_channel_half_width, driven_channel_length, driven_face_y - 1.0,
                                   driven_face_y + driven_centre_pocket_depth), op="cut", body="yoke", name="Driven centre channel")
for i, (x, z) in enumerate(horn_points):
    model.cylinder(f"passive_bore_{i}", horn_bore_r, passive_thickness + 2.0, at=(x, passive_face_y + 1.0, z), direction=(0, -1, 0),
                   op="cut", body="yoke", name=f"Passive horn bore {i}")
    model.cylinder(f"passive_lane_{i}", head_lane_r, head_lane_abs_y + passive_head_seat_y, at=(x, passive_head_seat_y, z),
                   direction=(0, -1, 0), op="cut", body="yoke", name=f"Passive head lane {i}")
model.add("outer_rim_round", rim_round_cut(y_ps_out, passive_outer_rim_round, rim_sector_a1, rim_sector_a2), op="cut", body="yoke",
          name="Passive outer rim round (sector)")
model.add("face_rim_round", rim_round_cut(passive_face_y, passive_face_rim_round, 0.0, 360.0, inward=-1), op="cut", body="yoke",
          name="Passive seat-face rim round")
model.cylinder("passive_pocket_big", passive_pocket_big_r, passive_pocket_big_depth + 1.0, at=(axis_x, passive_face_y + 1.0, axis_z),
               direction=(0, -1, 0), op="cut", body="yoke", name="Passive big pocket")
model.cylinder("passive_pocket_small", passive_pocket_small_r, passive_pocket_small_depth + 1.0, at=(axis_x, passive_face_y + 1.0, axis_z),
               direction=(0, -1, 0), op="cut", body="yoke", name="Passive small pocket")
model.add("passive_trace_wide", slot45(passive_trace_wide_half, passive_trace_length, passive_face_y + 1.0,
                                       passive_face_y - passive_trace_wide_depth), op="cut", body="yoke", name="Passive wide trace")
model.add("passive_trace_narrow", slot45(passive_trace_narrow_half, passive_trace_length, passive_face_y + 1.0,
                                         passive_face_y - passive_trace_narrow_depth), op="cut", body="yoke", name="Passive narrow trace")

# ---------------------------------------------------------------- 3. edge breaks, then the front wells re-cut
model.add("rib_edge_breaks", rib_edge_breaks(model.bodies.pop("yoke")), op="new", body="yoke",
          name="Passive rib edge breaks (R1.2 corners, R0.8 floor)")
for i, (x, y) in enumerate(anchors_front):
    model.cylinder(f"front_well_recut_{i}", front_well_r, band_z1 - plate_z1, at=(x, y, plate_z1), op="cut", body="yoke",
                   name=f"Front screw well re-cut {i}")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#8a8a90")
result = model.result()

