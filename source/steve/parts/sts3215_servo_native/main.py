# sts3215_servo_native: Feetech STS3215 servo (C001), one reusable part for every
# Frankenduck servo instance. Native rebuild of the vendor B-rep ST3215.step
# (solids 0, 1, 2 = case front/middle/rear, solid 6 = horn), mm.
#
# LOCAL FRAME (the Blender servo mesh frame used by body_servos.json):
#   local = R * step + t, R = [[0,0,1],[-1,0,0],[0,-1,0]], t = (0, -25.5, 9.6)
#   (the Blender mesh matches the vendor STEP to 0.007 mm with t_x = 0).
#   Output horn axis = local Z axis through x = 0, y = 0. The horn disc sits at
#   z = 0..2.5 (horn face at z = 0, pointing -Z); the case runs z = 2.1..37.8 with the
#   idler (rear) boss on the same axis at z = 37.8. Body length runs along -Y
#   (y = -35.06..10.06). Mounting ears: horn-side flange z = 3.2..4.7, rear flange
#   z = 33.7..35.2, M2 holes at x = +-10.25.
# Every dimension below was measured from the vendor solids (B-rep faces, sections).
import cadquery as cq

# horn (output side)
horn_r = 9.6  # mm Horn disc radius
horn_t = 2.5  # mm Horn disc thickness (z = 0..horn_t)
horn_ch = 0.2  # mm Horn disc edge chamfer
hub_r = 5.3  # mm Horn hub radius
hub_z1 = 4.5  # mm Horn hub top
horn_bolt_r = 7.0  # mm Horn screw bolt-circle radius
horn_hole_r = 1.25  # mm Horn screw hole radius
horn_bore_r = 1.6  # mm Horn centre screw hole radius
horn_cb_r = 3.1  # mm Horn centre counterbore radius
horn_cb_z0 = 1.5  # mm Horn centre counterbore floor
shaft_r = 2.6  # mm Output spline shaft radius (joins horn to case)
shaft_z0 = 2.3  # mm Output shaft bottom (centre counterbore roof)

# case outline (XY)
case_hw = 12.31  # mm Case half width (middle and rear shells)
case_y0 = -35.06  # mm Case bottom edge (far from the horn)
case_y1 = 10.06  # mm Case top edge (horn end)
case_ch = 19.73  # mm Top corner 45 deg chamfer line |x| + y = case_ch
case_r = 2.0  # mm Case vertical corner fillet radius
front_inset = 0.12  # mm Front shell outline inset relative to the middle shell
groove_h = 0.1  # mm Height of the open assembly grooves between the case shells
groove_d = 0.72  # mm Depth of the assembly grooves (to the shell lips)
groove1_z = 10.7  # mm Front/middle shell groove bottom
groove2_z = 23.6  # mm Middle/rear shell groove bottom
case_z0 = 4.7  # mm Front shell face (horn side)
front_z1 = 10.7  # mm Front shell to middle shell joint
case_z1 = 33.7  # mm Rear cover top face

# horn-side boss and flange
boss_hw = 6.95  # mm Horn-side boss half width
boss_y0 = -34.05  # mm Horn-side boss bottom edge
boss_arc_r = 11.36  # mm Horn-side boss concave arc radius around the horn axis
boss_z0 = 2.1  # mm Horn-side boss face
pad_hw = 5.5  # mm Horn-side boss label recess half width
pad_y0 = -30.5  # mm Label recess bottom edge
pad_y1 = -14.5  # mm Label recess top edge
pad_d = 0.15  # mm Label recess depth (also the connector top recess depth)
pad_r = 0.5  # mm Label recess corner radius
fl_z0 = 3.2  # mm Horn-side flange face
fl_hw = 11.96  # mm Flange ear half width
fl_y0 = -34.7  # mm Flange bottom edge
fl_y1 = -5.68  # mm Flange ear top edge
fl_arc_r = 11.0  # mm Flange concave arc radius around the horn axis
fl_nhw = 8.1  # mm Flange narrow tail half width
fl_ear_y0 = -30.85  # mm Flange ear bottom edge
fl_taper_h = 1.2  # mm Flange outer edge taper height (outer sides, bottom, ear tops, arc)
fl_taper_w = 0.35  # mm Flange outer edge taper width
notch_x = 10.0  # mm Case screw head notch centre X
notch_y = -32.75  # mm Case screw head notch centre Y
notch_r = 1.9  # mm Case screw head notch / rear slot screw boss radius
mount_x = 10.25  # mm Mounting hole X (both flanges)
mount_y_top = -8.3  # mm Upper mounting hole Y (both flanges)
mount_y_front = -29.0  # mm Lower mounting hole Y, horn-side flange
mount_y_rear = -32.75  # mm Lower mounting hole Y, rear flange
mount_r = 0.8  # mm Mounting hole radius

# rear cover: slot, flange, connector housing, idler boss
slot_hw = 10.76  # mm Rear slot half width
slot_y0 = -33.51  # mm Rear slot bottom edge
slot_y1 = -11.65  # mm Rear slot top edge
slot_z0 = 32.2  # mm Rear slot floor
rp_z1 = 35.2  # mm Rear flange top
rp_hw = 11.89  # mm Rear flange half width
rp_y0 = -34.64  # mm Rear flange bottom edge
rp_y1 = -5.8  # mm Rear flange ear top edge
rp_r = 1.8  # mm Rear flange bottom corner radius
rp_ear_x = 9.46  # mm Rear flange ear inner corner X
rp_nhw = 6.5  # mm Rear flange centre notch half width
rp_ny = -9.1  # mm Rear flange notch diagonal end Y
rp_wy0 = -16.45  # mm Rear flange wide notch bottom
rp_wy1 = -11.5  # mm Rear flange wide notch top
rp_whw = 9.72  # mm Rear flange wide notch half width
con_hw = 9.1  # mm Connector housing half width
con_y0 = -29.6  # mm Connector housing bottom edge
con_y1 = -17.7  # mm Connector housing base top edge
con_y1t = -19.8  # mm Connector housing top-face top edge (sloped side)
con_z1 = 37.1  # mm Connector housing top
con_in_hw = 8.42  # mm Connector housing / flange window half width
con_in_y0 = -28.9  # mm Connector window bottom edge
con_in_y1 = -18.6  # mm Connector window top edge
con_in_z1 = 35.7  # mm Connector housing inner roof
cable_hw = 1.0  # mm Rear cable notch half width
cable_y0 = 5.5  # mm Rear cable notch start Y (runs to the top edge)
con_rhw = 6.0  # mm Connector top recess half width
con_ry0 = -27.5  # mm Connector top recess bottom edge
con_ry1 = -21.5  # mm Connector top recess top edge
idl_r0 = 5.0  # mm Idler boss base ring radius
idl_z0 = 33.9  # mm Idler boss base ring top
idl_r = 3.0  # mm Idler boss radius
idl_z1 = 37.55  # mm Idler boss chamfer start (chamfer = idl_r - idl_rt)
idl_rt = 2.75  # mm Idler boss top radius
idl_z2 = 37.8  # mm Idler boss top (part maximum Z)
idl_hole_r = 1.3  # mm Idler boss hole radius
idl_hole_z0 = 32.4  # mm Idler boss hole floor


def at_z(z):
    return cq.Workplane("XY", origin=(0, 0, z))


def case_outline(w, d):
    # rectangle with 45 deg chamfered top corners, inset by d
    hw, y0, y1, c = case_hw - d, case_y0 + d, case_y1 - d, case_ch - d * (2 ** 0.5)
    pts = [(-hw, y0), (hw, y0), (hw, c - hw), (c - y1, y1), (y1 - c, y1), (-hw, c - hw)]
    return w.polyline(pts).close()


def block(z0, z1, d):
    return case_outline(at_z(z0), d).extrude(z1 - z0).edges("|Z").fillet(case_r)


body = block(front_z1, case_z1, 0.0).union(block(case_z0, front_z1, front_inset))
for gz in (groove1_z, groove2_z):  # open grooves where the shells meet
    body = body.cut(block(gz, gz + groove_h, -1.0).cut(block(gz, gz + groove_h, groove_d)))

# horn-side boss: rectangle with a concave arc around the horn
ya = -(boss_arc_r ** 2 - boss_hw ** 2) ** 0.5
boss = (at_z(boss_z0).moveTo(-boss_hw, boss_y0).lineTo(boss_hw, boss_y0).lineTo(boss_hw, ya)
        .threePointArc((0, -boss_arc_r), (-boss_hw, ya)).close().extrude(fl_z0 - boss_z0 + 0.01))

# horn-side mounting flange
xa = (fl_arc_r ** 2 - fl_y1 ** 2) ** 0.5
fl = (at_z(fl_z0).moveTo(-fl_nhw, fl_y0).lineTo(fl_nhw, fl_y0).lineTo(fl_nhw, notch_y)
      .radiusArc((notch_x, notch_y + notch_r), notch_r).lineTo(fl_hw, fl_ear_y0).lineTo(fl_hw, fl_y1)
      .lineTo(xa, fl_y1).threePointArc((0, -fl_arc_r), (-xa, fl_y1)).lineTo(-fl_hw, fl_y1)
      .lineTo(-fl_hw, fl_ear_y0).lineTo(-notch_x, fl_ear_y0)
      .radiusArc((-fl_nhw, notch_y), notch_r).close().extrude(case_z0 - fl_z0 + 0.01))


def tapered(e):
    # bottom edges of the outer sides, bottom edge, ear tops and arc (not the screw notches)
    c = e.Center()
    return abs(c.z - fl_z0) < 0.01 and (abs(abs(c.x) - fl_hw) < 0.01 or c.y < fl_y0 + 0.1 or c.y > fl_y1 - 6)


fl = fl.newObject([e for e in fl.faces("<Z").edges().vals() if tapered(e)]).chamfer(fl_taper_h, fl_taper_w)
body = body.union(boss).union(fl)
body = body.cut(at_z(boss_z0).center(0, (pad_y0 + pad_y1) / 2).rect(2 * pad_hw, pad_y1 - pad_y0)
                .extrude(pad_d).edges("|Z").fillet(pad_r))

# the flange pocket inside the vendor case is a sealed void (not external shape): not modelled
front_holes = [(x, y) for x in (-mount_x, mount_x) for y in (mount_y_top, mount_y_front)]
body = body.cut(at_z(fl_z0 - 1).pushPoints(front_holes).circle(mount_r).extrude(case_z0 - fl_z0 + 1))

# rear slot under the rear flange, around the two case screw bosses
slot = at_z(slot_z0).center(0, (slot_y0 + slot_y1) / 2).rect(2 * slot_hw, slot_y1 - slot_y0).extrude(case_z1 - slot_z0)
slot = slot.cut(at_z(slot_z0 - 1).pushPoints([(-notch_x, notch_y), (notch_x, notch_y)]).circle(notch_r).extrude(case_z1 - slot_z0 + 2))
body = body.cut(slot)

# rear mounting flange: U-shaped plate with a wide notch over the slot
rp = (at_z(case_z1).moveTo(-rp_hw, rp_y1).lineTo(-rp_ear_x, rp_y1).lineTo(-rp_nhw, rp_ny)
      .lineTo(-rp_nhw, rp_wy1).lineTo(-rp_whw, rp_wy1).lineTo(-rp_whw, rp_wy0).lineTo(rp_whw, rp_wy0)
      .lineTo(rp_whw, rp_wy1).lineTo(rp_nhw, rp_wy1).lineTo(rp_nhw, rp_ny).lineTo(rp_ear_x, rp_y1)
      .lineTo(rp_hw, rp_y1).lineTo(rp_hw, rp_y0).lineTo(-rp_hw, rp_y0).close()
      .extrude(rp_z1 - case_z1).edges("|Z and <Y").fillet(rp_r))
body = body.union(rp)

# connector housing: sloped +Y face, hollow underneath, window through the flange
con = (cq.Workplane("YZ", origin=(-con_hw, 0, 0)).polyline(
    [(con_y0, rp_z1 - 0.01), (con_y1, rp_z1 - 0.01), (con_y1t, con_z1), (con_y0, con_z1)]).close().extrude(2 * con_hw))
body = body.union(con)
win = (at_z(slot_z0 + 0.5).center(0, (con_in_y0 + con_in_y1) / 2)
       .rect(2 * con_in_hw, con_in_y1 - con_in_y0).extrude(con_in_z1 - slot_z0 - 0.5))
body = body.cut(win)
body = body.cut(at_z(con_z1 - pad_d).center(0, (con_ry0 + con_ry1) / 2).rect(2 * con_rhw, con_ry1 - con_ry0)
                .extrude(1).edges("|Z").fillet(pad_r))
body = body.cut(at_z(slot_z0).center(0, (cable_y0 + case_y1 + 1) / 2).rect(2 * cable_hw, case_y1 + 1 - cable_y0).extrude(5))
rear_holes = [(x, y) for x in (-mount_x, mount_x) for y in (mount_y_top, mount_y_rear)]
body = body.cut(at_z(slot_z0).pushPoints(rear_holes).circle(mount_r).extrude(rp_z1 - slot_z0 + 1))

# idler boss on the horn axis
idl = at_z(idl_z0 - 0.01).circle(idl_r).extrude(idl_z2 - idl_z0 + 0.01).faces(">Z").edges().chamfer(idl_r - idl_rt)
idl = idl.union(at_z(case_z1 - 0.01).circle(idl_r0).extrude(idl_z0 - case_z1 + 0.01))
body = body.union(idl).cut(at_z(idl_hole_z0).circle(idl_hole_r).extrude(idl_z2 - idl_hole_z0 + 1))

# horn: disc with chamfered edges, hub, four screw holes, stepped centre bore, spline shaft
horn = at_z(0).circle(horn_r).extrude(horn_t).edges().chamfer(horn_ch)
horn = horn.union(at_z(horn_t - 0.01).circle(hub_r).extrude(hub_z1 - horn_t + 0.01).faces(">Z").edges().chamfer(horn_ch))
horn = horn.cut(at_z(-1).polarArray(horn_bolt_r, 0, 360, 4).circle(horn_hole_r).extrude(horn_t + 2))
horn = horn.cut(at_z(-1).circle(horn_bore_r).extrude(hub_z1 + 2))
horn = horn.cut(at_z(horn_cb_z0).circle(horn_cb_r).extrude(shaft_z0 - horn_cb_z0))
shaft = at_z(shaft_z0).circle(shaft_r).extrude(case_z0 - shaft_z0 + 0.01)
body = body.union(horn).union(shaft)

from steve import Model

# No material (user decision 2026-10-10): this envelope is a mixed hardware proxy, so its solid carries no
# mass. The Frankenduck design gives each servo instance its measured mass as a lumped box
# (mass_record.py). It was ABS with colour #101010 for display only.
model = Model()
model.add("servo_envelope", body, body="servo")
result = model.result()

