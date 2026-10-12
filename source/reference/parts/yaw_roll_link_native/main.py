"""yaw_roll_link (R05) - native rebuild of the Frankenduck head yaw-roll link, robot world frame, mm, Z up.

Box around the yaw servo: rear wall bolted to the roll servo horn, front wall carrying the printed F688 journal,
top plate on the yaw servo back lands, -Y skirt. Geometry follows link() in build_parts_r05.py exactly (same world
coordinates); the script's polyline circles (72-gons) and polyline corner roundings (10 deg steps) are true arcs here.
Change from the script (user + coordinator decision 2026-10-04): the journal screw's M3 heat-set insert pocket and
the M3 clearance through the journal are replaced by ONE self-forming M3 hole entered from the screw side (journal
end face) and running through the front wall; the M3x10 screw forms its thread over x 49.8..58.85.
"""
import math
import cadquery as cq
from source_api import Model

roll_z = 280.58  # mm Head roll axis Z (axis is world X through y 0)
rear_x0 = 14.935  # mm Rear wall face on the roll servo horn
rear_x1 = 22.24  # mm Rear wall inner face (0.5 mm to the yaw servo case)
front_x0 = 48.0  # mm Front wall inner face
front_x1 = 52.5  # mm Front wall outer face
bed_z = 268.3  # mm Flat feet of both walls (print bed)
top_z0 = 293.205  # mm Top plate underside, on the yaw servo back lands
top_t = 4.0  # mm Top plate thickness
disc_r = 13.0  # mm Rear wall disc radius, and front wall disc radius on the -Y half
front_disc_r_plus = 9.0  # mm Front wall disc radius on the +Y half (clears the jaw servo plugs)
plate_y_min = -36.0  # mm Top plate and rear web -Y edge
skirt_y0 = -38.4  # mm -Y skirt outer face
skirt_y1 = -35.4  # mm -Y skirt inner face
skirt_z0 = 285.0  # mm -Y skirt bottom
shoulder_r = 5.2  # mm Shoulder ring radius for the F688 inner race
shoulder_x1 = 53.0  # mm Shoulder ring end (journal start)
journal_d = 7.95  # mm Printed journal diameter for the F688 inner race
journal_x1 = 58.85  # mm Journal end face
m3_clr = 3.4  # mm M3 clearance diameter
horn_bolt_off = 7.0  # mm Roll horn screw offset from the roll axis (4 screws, +-Y and +-Z)
head_well_r = 3.4  # mm Button head well radius
horn_grip = 4.0  # mm Rear wall grip under the screw heads
centre_pocket_r = 3.2  # mm Roll horn centre-screw head pocket radius
centre_pocket_depth = 3.2  # mm Roll horn centre-screw head pocket depth from the rear face
yaw_x = 35.1  # mm Yaw servo axis X (y 0)
boss_r = 3.5  # mm Hole for the bare passive boss of the yaw servo
plug_xl = 9.5  # mm Plug window half length in X
plug_y0 = -17.7  # mm Plug window -Y edge
plug_y1 = -11.2  # mm Plug window +Y edge
cover_xl = 9.4  # mm Raised-cover pocket half length in X
cover_y0 = -30.0  # mm Raised-cover pocket -Y edge
cover_y1 = -12.2  # mm Raised-cover pocket +Y edge
cover_d = 2.2  # mm Raised-cover pocket depth from the plate underside
window_r = 1.0  # mm Corner radius of the plug window and the cover pocket
m2_clr = 2.4  # mm M2 clearance diameter
csk_r = 2.0  # mm Countersink radius at the plate top
csk_r_min = 0.95  # mm Countersink radius at its bottom
csk_depth = 1.06  # mm Countersink depth below the plate top (cone starts 0.01 above it)
# Self-forming M3 thread hole (no heat-set insert): bore d + margin with three ridges the screw forms its thread
# into (Thomas Sanladerer's design; same function as top_head_shell_native, adapted to the X axis).
sft_d = 3.0  # mm Screw nominal diameter (M3)
sft_margin = 0.15  # mm Bore oversize over the screw diameter
sft_ridge_h = 0.4  # mm Ridge height into the bore (ridge tips at d + margin - 2 x ridge)
sft_taper = 3.0  # mm Entry length over which the ridges grow from 0 to full height
sft_ridges = 3  # Number of ridges, evenly spaced
journal_hole_entry_x = 58.85  # mm Journal screw hole entry face (journal end, screw side)
journal_hole_end_x = 47.0  # mm Journal screw hole end (through the front wall, 1 mm past its inner face)

top_z1 = top_z0 + top_t
yaw_pilots = [(yaw_x + 10.27, -8.30), (yaw_x - 10.27, -8.30), (yaw_x - 10.26, -32.74)]


def rounded_outline(s, tag, pts, radii):
    """Closed polygon with rounded corners (convex or concave) as sketch lines and CCW arcs; the corner geometry
    is the one round_poly() in cadlib.py approximates with points (tangent length clamped to 0.45 x edge)."""
    n = len(pts)
    corners = []
    for i in range(n):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
        ri = radii[i]
        ax, ay = p0[0] - p1[0], p0[1] - p1[1]
        bx, by = p2[0] - p1[0], p2[1] - p1[1]
        la, lb = math.hypot(ax, ay), math.hypot(bx, by)
        ax, ay, bx, by = ax / la, ay / la, bx / lb, by / lb
        ang = math.acos(max(-1.0, min(1.0, ax * bx + ay * by)))
        if ri <= 0 or ang < 1e-3 or abs(ang - math.pi) < 1e-3:
            corners.append((p1, p1, None))
            continue
        t = min(ri / math.tan(ang / 2), 0.45 * la, 0.45 * lb)
        rad = t * math.tan(ang / 2)
        q0 = (p1[0] + ax * t, p1[1] + ay * t)
        q1 = (p1[0] + bx * t, p1[1] + by * t)
        mx, my = ax + bx, ay + by
        ml = math.hypot(mx, my)
        k = rad / math.sin(ang / 2) / ml
        c = (p1[0] + mx * k, p1[1] + my * k)
        a0 = math.degrees(math.atan2(q0[1] - c[1], q0[0] - c[0]))
        a1 = math.degrees(math.atan2(q1[1] - c[1], q1[0] - c[0]))
        da = (a1 - a0 + 180.0) % 360.0 - 180.0
        arc = (c, rad, a0, a0 + da) if da > 0 else (c, rad, a1, a1 - da)
        corners.append((q0, q1, arc))
    for i in range(n):
        q0, q1, arc = corners[i]
        if arc is not None:
            s.arc(f"{tag}_a{i}", arc[0], arc[1], arc[2], arc[3])
        nxt = corners[(i + 1) % n][0]
        if math.hypot(nxt[0] - q1[0], nxt[1] - q1[1]) > 1e-6:
            s.line(f"{tag}_l{i}", q1, nxt)


def self_forming_hole_x(y, z, x_face, x_end, d=sft_d, margin=sft_margin, ridge_h=sft_ridge_h, taper=sft_taper,
                        ridges=sft_ridges, overshoot=2.0):
    """Cutting tool for a self-forming thread hole on an X-parallel axis at (y, z) (top_head_shell_native's
    self_forming_hole with the axis moved from Z to X). The screw enters at x_face and the hole runs to x_end
    (either direction). Bore radius r = (d + margin) / 2; each ridge is what a large cylinder (radius R = r + 5)
    leaves when it overlaps the bore by ridge_h, so the ridge tips sit at r - ridge_h. Over the first `taper` mm
    from x_face the overlap grows from 0 to ridge_h. The bore runs `overshoot` mm outside the entry face."""
    s = 1.0 if x_end > x_face else -1.0
    r = (d + margin) / 2.0
    big_r = r + 5.0
    off = big_r + r - ridge_h
    x_out = x_face - s * overshoot
    x_lo, x_hi = min(x_out, x_end), max(x_out, x_end)
    ax = cq.Vector(1, 0, 0)
    tool = cq.Solid.makeCylinder(r, x_hi - x_lo, cq.Vector(x_lo, y, z), ax)
    dx = cq.Vector(s, 0, 0)
    for i in range(int(ridges)):
        a = 2.0 * math.pi * i / ridges
        cy, cz = y + off * math.cos(a), z + off * math.sin(a)
        entry = cq.Solid.makeCylinder(big_r - ridge_h, overshoot, cq.Vector(x_out, cy, cz), dx)
        ramp = cq.Solid.makeCone(big_r - ridge_h, big_r, taper, cq.Vector(x_face, cy, cz), dx)
        full = cq.Solid.makeCylinder(big_r, abs(x_end - x_face) - taper, cq.Vector(x_face + s * taper, cy, cz), dx)
        tool = tool.cut(entry.fuse(ramp).fuse(full).clean())
    return tool.clean()


# chord where the r13 discs meet the print bed
bed_dz = bed_z - roll_z
bed_w = math.sqrt(disc_r ** 2 - bed_dz ** 2)
bed_a = math.degrees(math.atan2(bed_dz, bed_w))  # angle of the +Y bed corner (negative)

model = Model()
# --- rear wall: r13 disc on the roll horn, flat foot at the bed, plus the web up to the top plate
model.plane("pl_rear", on="YZ", offset=rear_x0, name="Rear wall face (roll horn)")
with model.sketch("sk_rear_disc", on="pl_rear", name="Rear wall disc") as s:
    s.arc("disc", (0.0, roll_z), disc_r, bed_a, 180.0 - bed_a)
    s.line("foot", (-bed_w, bed_z), (bed_w, bed_z))
model.extrude("rear_disc", "sk_rear_disc", distance=rear_x1 - rear_x0, op="new", body="link", name="Rear wall disc")
with model.sketch("sk_rear_web", on="pl_rear", name="Rear wall web") as s:
    rounded_outline(s, "web", [(plate_y_min, 285.0), (-12.0, 274.0), (9.4, 274.0), (9.4, top_z0), (6.7, top_z1),
                               (plate_y_min, top_z1)], [2, 4, 0, 0, 0, 2])
model.extrude("rear_web", "sk_rear_web", distance=rear_x1 - rear_x0, op="join", body="link", name="Rear wall web")
# --- front wall: r13 on the -Y half (load path to the top plate), r9 on the +Y half (under the jaw servo plugs)
model.plane("pl_front", on="YZ", offset=front_x0, name="Front wall inner face")
with model.sketch("sk_front_disc", on="pl_front", name="Front wall disc") as s:
    s.arc("minus", (0.0, roll_z), disc_r, 90.0, 180.0 - bed_a)
    s.line("foot", (-bed_w, bed_z), (0.0, bed_z))
    s.line("step_low", (0.0, bed_z), (0.0, roll_z - front_disc_r_plus))
    s.arc("plus", (0.0, roll_z), front_disc_r_plus, -90.0, 90.0)
    s.line("step_high", (0.0, roll_z + front_disc_r_plus), (0.0, roll_z + disc_r))
model.extrude("front_disc", "sk_front_disc", distance=front_x1 - front_x0, op="join", body="link", name="Front wall disc")
with model.sketch("sk_front_web", on="pl_front", name="Front wall web") as s:
    rounded_outline(s, "web", [(-28.0, 288.0), (-8.0, 284.0), (0.0, 284.0), (0.0, top_z1), (-28.0, top_z1)],
                    [2, 3, 0, 0, 0])
model.extrude("front_web", "sk_front_web", distance=front_x1 - front_x0, op="join", body="link", name="Front wall web")
# --- top plate on the yaw servo back lands; +Y edge trimmed to y 0 over x 41..52.5
model.plane("pl_top", on="XY", offset=top_z0, name="Top plate underside")
with model.sketch("sk_top", on="pl_top", name="Top plate") as s:
    rounded_outline(s, "top", [(rear_x0, plate_y_min), (front_x0, plate_y_min), (front_x0, -28.0), (front_x1, -28.0),
                               (front_x1, 0.0), (44.5, 0.0), (41.0, 6.7), (rear_x0, 6.7)], [2, 0, 0, 0, 0, 2, 2, 0])
model.extrude("top_plate", "sk_top", distance=top_t, op="join", body="link", name="Top plate")
model.box("skirt", front_x0 - rear_x1, skirt_y1 - skirt_y0, top_z1 - skirt_z0,
          at=((rear_x1 + front_x0) / 2, (skirt_y0 + skirt_y1) / 2, (skirt_z0 + top_z1) / 2), op="join", body="link",
          name="-Y skirt (locates the yaw servo in Y)")
model.cylinder("shoulder", shoulder_r, shoulder_x1 - front_x1, at=(front_x1, 0.0, roll_z), direction=(1, 0, 0),
               op="join", body="link", name="F688 inner race shoulder")
model.cylinder("journal", journal_d / 2, journal_x1 - shoulder_x1, at=(shoulder_x1, 0.0, roll_z), direction=(1, 0, 0),
               op="join", body="link", name="Printed F688 journal")
# --- rear wall: 4 roll horn screws (clearance + button head wells), centre-screw head pocket
for i, (dy, dz) in enumerate(((0, horn_bolt_off), (0, -horn_bolt_off), (horn_bolt_off, 0), (-horn_bolt_off, 0))):
    model.cylinder(f"horn_clr_{i}", m3_clr / 2, rear_x1 - rear_x0 + 2.0, at=(rear_x0 - 1.0, dy, roll_z + dz),
                   direction=(1, 0, 0), op="cut", body="link", name=f"Roll horn M3 clearance {i}")
    model.cylinder(f"horn_well_{i}", head_well_r, rear_x1 + 1.0 - (rear_x0 + horn_grip), at=(rear_x0 + horn_grip, dy, roll_z + dz),
                   direction=(1, 0, 0), op="cut", body="link", name=f"Roll horn screw head well {i}")
model.cylinder("centre_pocket", centre_pocket_r, centre_pocket_depth + 1.0, at=(rear_x0 - 1.0, 0.0, roll_z),
               direction=(1, 0, 0), op="cut", body="link", name="Roll horn centre-screw head pocket")
# --- journal screw: self-forming M3 hole from the journal end through the front wall (replaces insert + clearance)
model.add("journal_hole", self_forming_hole_x(0.0, roll_z, journal_hole_entry_x, journal_hole_end_x), op="cut",
          body="link", name="Journal screw self-forming M3 hole")
# --- top plate: passive-boss hole, plug window, raised-cover pocket, 3 countersunk M2 clearances
model.cylinder("boss_hole", boss_r, top_t + 2.0, at=(yaw_x, 0.0, top_z0 - 1.0), op="cut", body="link",
               name="Yaw passive-boss hole")
model.plane("pl_win", on="XY", offset=top_z0 - 1.0, name="Top plate cut base")
with model.sketch("sk_plug", on="pl_win", name="Yaw plug window") as s:
    rounded_outline(s, "plug", [(yaw_x - plug_xl, plug_y0), (yaw_x + plug_xl, plug_y0), (yaw_x + plug_xl, plug_y1),
                                (yaw_x - plug_xl, plug_y1)], [window_r] * 4)
model.extrude("plug_window", "sk_plug", distance=top_t + 2.0, op="cut", body="link", name="Yaw plug window")
with model.sketch("sk_cover", on="pl_win", name="Yaw raised-cover pocket") as s:
    rounded_outline(s, "cov", [(yaw_x - cover_xl, cover_y0), (yaw_x + cover_xl, cover_y0), (yaw_x + cover_xl, cover_y1),
                               (yaw_x - cover_xl, cover_y1)], [window_r] * 4)
model.extrude("cover_pocket", "sk_cover", distance=cover_d + 1.0, op="cut", body="link", name="Yaw raised-cover pocket")
for i, (x, y) in enumerate(yaw_pilots):
    model.cylinder(f"m2_clr_{i}", m2_clr / 2, top_t + 2.0, at=(x, y, top_z0 - 1.0), op="cut", body="link",
                   name=f"Yaw servo M2 clearance {i}")
    model.cone(f"m2_csk_{i}", csk_r_min, csk_r, csk_depth, at=(x, y, top_z1 + 0.01 - csk_depth), op="cut", body="link",
               name=f"Yaw servo M2 countersink {i}")
# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#8a8a90")
result = model.result()

