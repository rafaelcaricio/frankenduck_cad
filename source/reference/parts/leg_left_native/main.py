# leg_shin_native: Frankenduck shin (holds the knee horn and the ankle bearing boss).
# Native rebuild of the Blender meshes leg.001 (left, SIDE = 1) and leg.002 (right, SIDE = -1),
# robot world frame, mm, Z up. leg.002 is the exact mirror of leg.001 about Y = 0 (measured,
# max 0.004 mm), so both documents share this source and differ only in SIDE.
# Every dimension below was measured from the mesh (sections, circle fits, depth maps).
# Geometry is written for the left part; the right part mirrors the finished solid about the XZ plane.
import math
import cadquery as cq

SIDE = 1  # 1 = left shin (leg.001, +Y), -1 = right shin (leg.002, mirror about Y = 0)

cx = -42.899  # mm X of the shin centre line (knee and ankle axes)
knee_z = 70.675  # mm Z of the knee servo horn axis (axis along Y)
ankle_z = 3.975  # mm Z of the ankle bearing boss axis (axis along Y)
half_w = 13.5  # mm Half width of the shin outline; also the round top radius
bot_z = -0.525  # mm Bottom edge of the outline
bot_corner_r = 4.0  # mm Bottom outline corner radius
bulge_r = 8.633  # mm Bottom edge bulge radius under the ankle boss
bulge_cz = 7.457  # mm Bottom edge bulge centre Z
y_core = 48.88  # mm Plate -Y face (lower region)
y_top = 51.98  # mm Plate +Y face (head region)
y_top_low = 51.88  # mm Plate +Y face below the neck recess
y_neck = 51.28  # mm Neck recess floor (+Y)
neck_z0 = 41.1  # mm Neck recess lower edge
neck_z1 = 57.1  # mm Neck recess ramp start
ramp_z1 = 60.5  # mm Neck recess ramp end (back at y_top)
fl_y = 47.875  # mm Flange -Y face (lower flange)
fl_y_up = 48.18  # mm Flange -Y face (neck and head)
fl_split_z = 44.5  # mm Z where the flange face steps from fl_y to fl_y_up (mesh: 0.3 mm ramp over z 41.6..47.4)
sp_y = 44.82  # mm Spine -Y face
sp_y_mid = 46.35  # mm Spine wall reference height
sp_hw = 8.625  # mm Spine half width at the reference height, below fl_split_z
sp_hw_up = 8.22  # mm Spine half width at the reference height, above fl_split_z
sp_head_r = 11.83  # mm Spine round head radius at the reference height
sp_neck_r = 3.0  # mm Spine neck (column to head) outline fillet at the reference height
sp_draft = 33.4  # deg Spine wall draft from the Y axis, below fl_split_z
sp_top_r = 1.24  # mm Spine top edge fillet (convex), below fl_split_z
sp_base_r = 1.28  # mm Spine base fillet (concave, into the flange), below fl_split_z
sp_draft_up = 19.6  # deg Spine wall draft from the Y axis, neck and head
sp_top_r_up = 1.37  # mm Spine top edge fillet, neck and head
sp_base_r_up = 3.32  # mm Spine base fillet, neck and head
end_z_mid = 24.58  # mm Z of the drafted bottom plane at the reference height
end_slope = 0.784  # mm/mm Bottom plane dZ per mm of Y (38 deg from the Y axis)
boss_r = 3.9  # mm Ankle boss radius
boss_y = 40.66  # mm Ankle boss end face
boss_ch = 0.2  # mm Ankle boss end chamfer
collar_r = 4.75  # mm Ankle boss collar radius
collar_y = 46.61  # mm Ankle boss collar face
collar_blend_r = 2.1  # mm Collar-to-plate concave blend radius
boss_hole_r = 0.6  # mm Ankle boss pin hole radius
boss_hole_dx = 1.8  # mm Ankle boss pin hole offset from the axis (along X)
boss_hole_y = 44.71  # mm Ankle boss pin hole bottom
ring_ro = 4.8  # mm +Y bearing ring outer radius
ring_y = 53.18  # mm +Y bearing ring top face
bore_r = 3.2  # mm +Y bearing bore radius (ankle) and knee centre bore radius
bore_y = 49.23  # mm Ankle bore floor
knee_bore_y = 48.78  # mm Knee centre bore floor
horn_bolt = 7.0  # mm Knee horn screw bolt-circle radius
horn_hole_r = 1.7  # mm Knee horn screw hole radius
horn_cb_r = 3.4  # mm Knee horn screw recess radius (-Y side)
horn_cb_y = 47.98  # mm Knee horn screw recess floor
srv_dx = 10.25  # mm Servo screw hole offset from the centre line (X)
srv_hole_r = 1.1  # mm Servo screw hole radius
srv_z_low = 12.275  # mm Lower servo screw holes Z
srv_z_mid = 36.725  # mm Middle servo screw holes Z
srv_cb_r = 2.55  # mm Middle screw head clearance radius (-Y side, down to fl_y)
srv_z_up = 46.725  # mm Upper blind holes Z
srv_up_y = 50.33  # mm Upper blind hole floor
win_hw = 10.5  # mm Window half width
win_z0 = 14.725  # mm Window bottom
win_z1 = 22.975  # mm Window top
win_r = 2.5  # mm Window corner radius
slot_hw = 3.375  # mm Slot half width
slot_z0 = 41.65  # mm Slot bottom
slot_z1 = 55.15  # mm Slot top
slot_r = 2.04  # mm Slot corner radius
pk_hw = 9.5  # mm +Y pocket half width
pk_z0 = 21.0  # mm +Y pocket bottom (opens into the window, whose top is the real edge)
pk_y = 49.78  # mm +Y pocket floor
pk_ear_r = 3.25  # mm Material ear radius around the middle servo screws
strip_z0 = 41.0  # mm +Y strip bottom
strip_z1 = 44.03  # mm +Y strip top
strip_y = 49.47  # mm +Y strip floor
strip_corner_r = 1.9  # mm Material discs at the strip upper corners
strip_corner_dx = 10.8  # mm Strip corner disc offset from the centre line
strip_corner_z = 44.35  # mm Strip corner disc Z
lip_x0 = -47.62  # mm Slot lip pocket X minimum
lip_x1 = -38.17  # mm Slot lip pocket X maximum
lip_r = 2.71  # mm Slot lip pocket top corner radius
lip_y = 49.95  # mm Slot lip pocket floor


def side(s):
    return s.mirror("XZ") if SIDE < 0 else s


def prism_xz(profile, y0, y1):
    # XZ workplane: local x = X, local y = Z, normal -Y; extrude from y1 down to y0
    return profile(cq.Workplane("XZ", origin=(0, y1, 0))).extrude(y1 - y0)


def rr_prism(x0, x1, z0, z1, r, y0, y1):
    b = cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    return b.edges("|Y").fillet(r)


def outline(w):
    # rectangle with round top (R = half_w on the knee axis), bottom corners and bottom bulge
    xl, xr = cx - half_w, cx + half_w
    bx = math.sqrt(bulge_r ** 2 - (bulge_cz - bot_z) ** 2)
    w = w.moveTo(xl, knee_z).lineTo(xl, bot_z + bot_corner_r)
    w = w.radiusArc((xl + bot_corner_r, bot_z), -bot_corner_r)
    w = w.lineTo(cx - bx, bot_z).threePointArc((cx, bulge_cz - bulge_r), (cx + bx, bot_z))
    w = w.lineTo(xr - bot_corner_r, bot_z).radiusArc((xr, bot_z + bot_corner_r), -bot_corner_r)
    w = w.lineTo(xr, knee_z).threePointArc((cx, knee_z + half_w), (xl, knee_z))
    return w.close()


def spine_wire(y, z_low, hw0, draft):
    d = (y - sp_y_mid) * math.tan(math.radians(draft))  # outward offset of the drafted wall at this Y
    hw, R, rn = hw0 + d, sp_head_r + d, sp_neck_r - d
    zf = knee_z - math.sqrt((R + rn) ** 2 - (hw + rn) ** 2)
    pts = {}
    for s in (1, -1):
        F = (cx + s * (hw + rn), zf)
        L = math.hypot(F[0] - cx, F[1] - knee_z)
        P = (cx + (F[0] - cx) * R / L, knee_z + (F[1] - knee_z) * R / L)
        a0 = math.atan2(0, -s)  # from fillet centre toward the column line
        a1 = math.atan2(knee_z - F[1], cx - F[0])
        am = (a0 + a1) / 2
        pts[s] = (F, P, (F[0] + rn * math.cos(am), F[1] + rn * math.sin(am)))
    wp = cq.Workplane("XZ", origin=(0, y, 0))
    w = wp.moveTo(cx - hw, z_low).lineTo(cx + hw, z_low).lineTo(cx + hw, zf)
    w = w.threePointArc(pts[1][2], pts[1][1]).threePointArc((cx, knee_z + R), pts[-1][1])
    w = w.threePointArc(pts[-1][2], (cx - hw, zf)).close()
    return w.wires().val()


# plate core and +Y face levels
sil = prism_xz(outline, 30.0, 60.0)  # silhouette prism, used for clipping
body = prism_xz(outline, y_core, y_top)
plus_cut = (cq.Workplane("YZ", origin=(cx - 20, 0, 0))
            .polyline([(y_top_low, -5), (y_top_low, neck_z0), (y_neck, neck_z0), (y_neck, neck_z1),
                       (y_top, ramp_z1), (60, ramp_z1), (60, -5)]).close().extrude(40))
body = body.cut(plus_cut)

# -Y relief: flat flange, drafted spine, drafted bottom plane, spine edge fillets.
# The mesh flange rises 0.3 mm (fl_y -> fl_y_up) over z 41.6..47.4; the relief is built twice on a
# flat flange (fl_y below fl_split_z, fl_y_up above) because the spine base fillet fails across the ramp.
zb = lambda y: end_z_mid - end_slope * (y - sp_y_mid)
end_cut = (cq.Workplane("YZ", origin=(cx - 20, 0, 0))
           .polyline([(30, zb(30)), (y_core + 1, zb(y_core + 1)), (y_core + 1, -10), (30, -10)]).close().extrude(40))


def relief_part(base_y, hw0, draft, r_top, r_base):
    flange = prism_xz(outline, base_y, y_core + 0.5)
    spine = cq.Solid.makeLoft([spine_wire(sp_y, 10.0, hw0, draft), spine_wire(y_core + 0.5, 10.0, hw0, draft)], True)
    rel = flange.union(cq.Workplane().add(spine).intersect(sil)).cut(end_cut)
    rel = rel.faces("<Y").edges().fillet(r_top)

    def inside(p):
        d = min(p.x - (cx - half_w), (cx + half_w) - p.x)
        if p.z > knee_z:
            d = min(d, half_w - math.hypot(p.x - cx, p.z - knee_z))
        return d

    def base_edge(e):  # spine wall to flange junction, inside the outline, above the bottom plane
        pts = [e.positionAt(t) for t in (0.1, 0.5, 0.9)]
        return abs(pts[1].y - base_y) < 0.01 and min(inside(q) for q in pts) > 0.2 and pts[1].z > 24.0

    return rel.edges().filter(base_edge).fillet(r_base)


below = cq.Workplane("XY").box(100, 100, 100).translate((cx, 50, fl_split_z - 50))
above = cq.Workplane("XY").box(100, 100, 100).translate((cx, 50, fl_split_z + 50))
body = body.union(relief_part(fl_y, sp_hw, sp_draft, sp_top_r, sp_base_r).intersect(below))
body = body.union(relief_part(fl_y_up, sp_hw_up, sp_draft_up, sp_top_r_up, sp_base_r_up).intersect(above))

# ankle boss (revolved about the ankle axis), clipped by the outline
bp = [(0, boss_y), (boss_r - boss_ch, boss_y), (boss_r, boss_y + boss_ch), (boss_r, collar_y),
      (collar_r, collar_y), (collar_r, collar_y + (y_core - collar_y) - collar_blend_r)]
boss = (cq.Workplane("XY").polyline(bp)
        .radiusArc((collar_r + collar_blend_r, y_core), collar_blend_r)
        .lineTo(collar_r + collar_blend_r, y_core + 0.5).lineTo(0, y_core + 0.5).close()
        .revolve(360, (0, 0, 0), (0, 1, 0)))
boss = boss.translate((cx, 0, ankle_z)).intersect(sil)
body = body.union(boss)
body = body.union(cq.Workplane("XZ", origin=(0, ring_y, 0)).center(cx, ankle_z).circle(ring_ro).extrude(ring_y - y_top_low + 0.2))

# through cuts: window, slot, knee horn screws, servo screws
body = body.cut(rr_prism(cx - win_hw, cx + win_hw, win_z0, win_z1, win_r, 30, 60))
body = body.cut(rr_prism(cx - slot_hw, cx + slot_hw, slot_z0, slot_z1, slot_r, 30, 60))
horn = [(cx - horn_bolt, knee_z), (cx + horn_bolt, knee_z), (cx, knee_z - horn_bolt), (cx, knee_z + horn_bolt)]
body = body.cut(prism_xz(lambda w: w.pushPoints(horn).circle(horn_hole_r), 30, 60))
body = body.cut(prism_xz(lambda w: w.pushPoints(horn).circle(horn_cb_r), 30, horn_cb_y))
srv = [(cx + s * srv_dx, z) for s in (1, -1) for z in (srv_z_low, srv_z_mid)]
body = body.cut(prism_xz(lambda w: w.pushPoints(srv).circle(srv_hole_r), 30, 60))
body = body.cut(prism_xz(lambda w: w.pushPoints([(cx + s * srv_dx, srv_z_mid) for s in (1, -1)]).circle(srv_cb_r), 30, fl_y))

# blind bores: knee centre (+Y), ankle bearing (+Y), boss pin holes (-Y), upper servo holes (+Y)
body = body.cut(prism_xz(lambda w: w.center(cx, knee_z).circle(bore_r), knee_bore_y, 60))
body = body.cut(prism_xz(lambda w: w.center(cx, ankle_z).circle(bore_r), bore_y, 60))
body = body.cut(prism_xz(lambda w: w.pushPoints([(cx - boss_hole_dx, ankle_z), (cx + boss_hole_dx, ankle_z)]).circle(boss_hole_r), 30, boss_hole_y))
body = body.cut(prism_xz(lambda w: w.pushPoints([(cx + s * srv_dx, srv_z_up) for s in (1, -1)]).circle(srv_hole_r), srv_up_y, 60))

# +Y servo pockets: main pocket, strip, slot lip
ears = prism_xz(lambda w: w.pushPoints([(cx + s * srv_dx, srv_z_mid) for s in (1, -1)]).circle(pk_ear_r), 30, 60)
corners = prism_xz(lambda w: w.pushPoints([(cx + s * strip_corner_dx, strip_corner_z) for s in (1, -1)]).circle(strip_corner_r), 30, 60)
pocket = prism_xz(lambda w: w.center(cx, (pk_z0 + strip_z1) / 2).rect(2 * pk_hw, strip_z1 - pk_z0), pk_y, 60)
body = body.cut(pocket.cut(ears).cut(corners))
strip = prism_xz(lambda w: w.center(cx, (strip_z0 + strip_z1) / 2).rect(2 * pk_hw, strip_z1 - strip_z0), strip_y, 60)
body = body.cut(strip.cut(corners))
lip = rr_prism(lip_x0, lip_x1, strip_z0, slot_z1, lip_r, lip_y, 60)  # lower part lies inside the deeper strip
body = body.cut(lip)

from source_api import Model

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model = Model()
model.add("printed_body", side(body))
model.material("pla", color="#8a8a90")
result = model.result()

