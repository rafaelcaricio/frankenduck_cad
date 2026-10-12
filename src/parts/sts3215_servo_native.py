from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    import cadquery as cq
    horn_r = 9.6
    horn_t = 2.5
    horn_ch = 0.2
    hub_r = 5.3
    hub_z1 = 4.5
    horn_bolt_r = 7.0
    horn_hole_r = 1.25
    horn_bore_r = 1.6
    horn_cb_r = 3.1
    horn_cb_z0 = 1.5
    shaft_r = 2.6
    shaft_z0 = 2.3
    case_hw = 12.31
    case_y0 = -35.06
    case_y1 = 10.06
    case_ch = 19.73
    case_r = 2.0
    front_inset = 0.12
    groove_h = 0.1
    groove_d = 0.72
    groove1_z = 10.7
    groove2_z = 23.6
    case_z0 = 4.7
    front_z1 = 10.7
    case_z1 = 33.7
    boss_hw = 6.95
    boss_y0 = -34.05
    boss_arc_r = 11.36
    boss_z0 = 2.1
    pad_hw = 5.5
    pad_y0 = -30.5
    pad_y1 = -14.5
    pad_d = 0.15
    pad_r = 0.5
    fl_z0 = 3.2
    fl_hw = 11.96
    fl_y0 = -34.7
    fl_y1 = -5.68
    fl_arc_r = 11.0
    fl_nhw = 8.1
    fl_ear_y0 = -30.85
    fl_taper_h = 1.2
    fl_taper_w = 0.35
    notch_x = 10.0
    notch_y = -32.75
    notch_r = 1.9
    mount_x = 10.25
    mount_y_top = -8.3
    mount_y_front = -29.0
    mount_y_rear = -32.75
    mount_r = 0.8
    slot_hw = 10.76
    slot_y0 = -33.51
    slot_y1 = -11.65
    slot_z0 = 32.2
    rp_z1 = 35.2
    rp_hw = 11.89
    rp_y0 = -34.64
    rp_y1 = -5.8
    rp_r = 1.8
    rp_ear_x = 9.46
    rp_nhw = 6.5
    rp_ny = -9.1
    rp_wy0 = -16.45
    rp_wy1 = -11.5
    rp_whw = 9.72
    con_hw = 9.1
    con_y0 = -29.6
    con_y1 = -17.7
    con_y1t = -19.8
    con_z1 = 37.1
    con_in_hw = 8.42
    con_in_y0 = -28.9
    con_in_y1 = -18.6
    con_in_z1 = 35.7
    cable_hw = 1.0
    cable_y0 = 5.5
    con_rhw = 6.0
    con_ry0 = -27.5
    con_ry1 = -21.5
    idl_r0 = 5.0
    idl_z0 = 33.9
    idl_r = 3.0
    idl_z1 = 37.55
    idl_rt = 2.75
    idl_z2 = 37.8
    idl_hole_r = 1.3
    idl_hole_z0 = 32.4
    
    def at_z(z):
        return cq.Workplane('XY', origin=(0, 0, z))
    
    def case_outline(w, d):
        hw, y0, y1, c = (case_hw - d, case_y0 + d, case_y1 - d, case_ch - d * 2 ** 0.5)
        pts = [(-hw, y0), (hw, y0), (hw, c - hw), (c - y1, y1), (y1 - c, y1), (-hw, c - hw)]
        return w.polyline(pts).close()
    
    def block(z0, z1, d):
        return case_outline(at_z(z0), d).extrude(z1 - z0).edges('|Z').fillet(case_r)
    body = block(front_z1, case_z1, 0.0).union(block(case_z0, front_z1, front_inset))
    for gz in (groove1_z, groove2_z):
        body = body.cut(block(gz, gz + groove_h, -1.0).cut(block(gz, gz + groove_h, groove_d)))
    ya = -(boss_arc_r ** 2 - boss_hw ** 2) ** 0.5
    boss = at_z(boss_z0).moveTo(-boss_hw, boss_y0).lineTo(boss_hw, boss_y0).lineTo(boss_hw, ya).threePointArc((0, -boss_arc_r), (-boss_hw, ya)).close().extrude(fl_z0 - boss_z0 + 0.01)
    xa = (fl_arc_r ** 2 - fl_y1 ** 2) ** 0.5
    fl = at_z(fl_z0).moveTo(-fl_nhw, fl_y0).lineTo(fl_nhw, fl_y0).lineTo(fl_nhw, notch_y).radiusArc((notch_x, notch_y + notch_r), notch_r).lineTo(fl_hw, fl_ear_y0).lineTo(fl_hw, fl_y1).lineTo(xa, fl_y1).threePointArc((0, -fl_arc_r), (-xa, fl_y1)).lineTo(-fl_hw, fl_y1).lineTo(-fl_hw, fl_ear_y0).lineTo(-notch_x, fl_ear_y0).radiusArc((-fl_nhw, notch_y), notch_r).close().extrude(case_z0 - fl_z0 + 0.01)
    
    def tapered(e):
        c = e.Center()
        return abs(c.z - fl_z0) < 0.01 and (abs(abs(c.x) - fl_hw) < 0.01 or c.y < fl_y0 + 0.1 or c.y > fl_y1 - 6)
    fl = fl.newObject([e for e in fl.faces('<Z').edges().vals() if tapered(e)]).chamfer(fl_taper_h, fl_taper_w)
    body = body.union(boss).union(fl)
    body = body.cut(at_z(boss_z0).center(0, (pad_y0 + pad_y1) / 2).rect(2 * pad_hw, pad_y1 - pad_y0).extrude(pad_d).edges('|Z').fillet(pad_r))
    front_holes = [(x, y) for x in (-mount_x, mount_x) for y in (mount_y_top, mount_y_front)]
    body = body.cut(at_z(fl_z0 - 1).pushPoints(front_holes).circle(mount_r).extrude(case_z0 - fl_z0 + 1))
    slot = at_z(slot_z0).center(0, (slot_y0 + slot_y1) / 2).rect(2 * slot_hw, slot_y1 - slot_y0).extrude(case_z1 - slot_z0)
    slot = slot.cut(at_z(slot_z0 - 1).pushPoints([(-notch_x, notch_y), (notch_x, notch_y)]).circle(notch_r).extrude(case_z1 - slot_z0 + 2))
    body = body.cut(slot)
    rp = at_z(case_z1).moveTo(-rp_hw, rp_y1).lineTo(-rp_ear_x, rp_y1).lineTo(-rp_nhw, rp_ny).lineTo(-rp_nhw, rp_wy1).lineTo(-rp_whw, rp_wy1).lineTo(-rp_whw, rp_wy0).lineTo(rp_whw, rp_wy0).lineTo(rp_whw, rp_wy1).lineTo(rp_nhw, rp_wy1).lineTo(rp_nhw, rp_ny).lineTo(rp_ear_x, rp_y1).lineTo(rp_hw, rp_y1).lineTo(rp_hw, rp_y0).lineTo(-rp_hw, rp_y0).close().extrude(rp_z1 - case_z1).edges('|Z and <Y').fillet(rp_r)
    body = body.union(rp)
    con = cq.Workplane('YZ', origin=(-con_hw, 0, 0)).polyline([(con_y0, rp_z1 - 0.01), (con_y1, rp_z1 - 0.01), (con_y1t, con_z1), (con_y0, con_z1)]).close().extrude(2 * con_hw)
    body = body.union(con)
    win = at_z(slot_z0 + 0.5).center(0, (con_in_y0 + con_in_y1) / 2).rect(2 * con_in_hw, con_in_y1 - con_in_y0).extrude(con_in_z1 - slot_z0 - 0.5)
    body = body.cut(win)
    body = body.cut(at_z(con_z1 - pad_d).center(0, (con_ry0 + con_ry1) / 2).rect(2 * con_rhw, con_ry1 - con_ry0).extrude(1).edges('|Z').fillet(pad_r))
    body = body.cut(at_z(slot_z0).center(0, (cable_y0 + case_y1 + 1) / 2).rect(2 * cable_hw, case_y1 + 1 - cable_y0).extrude(5))
    rear_holes = [(x, y) for x in (-mount_x, mount_x) for y in (mount_y_top, mount_y_rear)]
    body = body.cut(at_z(slot_z0).pushPoints(rear_holes).circle(mount_r).extrude(rp_z1 - slot_z0 + 1))
    idl = at_z(idl_z0 - 0.01).circle(idl_r).extrude(idl_z2 - idl_z0 + 0.01).faces('>Z').edges().chamfer(idl_r - idl_rt)
    idl = idl.union(at_z(case_z1 - 0.01).circle(idl_r0).extrude(idl_z0 - case_z1 + 0.01))
    body = body.union(idl).cut(at_z(idl_hole_z0).circle(idl_hole_r).extrude(idl_z2 - idl_hole_z0 + 1))
    horn = at_z(0).circle(horn_r).extrude(horn_t).edges().chamfer(horn_ch)
    horn = horn.union(at_z(horn_t - 0.01).circle(hub_r).extrude(hub_z1 - horn_t + 0.01).faces('>Z').edges().chamfer(horn_ch))
    horn = horn.cut(at_z(-1).polarArray(horn_bolt_r, 0, 360, 4).circle(horn_hole_r).extrude(horn_t + 2))
    horn = horn.cut(at_z(-1).circle(horn_bore_r).extrude(hub_z1 + 2))
    horn = horn.cut(at_z(horn_cb_z0).circle(horn_cb_r).extrude(shaft_z0 - horn_cb_z0))
    shaft = at_z(shaft_z0).circle(shaft_r).extrude(case_z0 - shaft_z0 + 0.01)
    body = body.union(horn).union(shaft)
    from lib.cadquery_adapter import Model
    model = Model()
    model.add('servo_envelope', body, body='servo')
    result = model.result()
    return to_build123d(model, 'sts3215_servo_native')


@step(out="../../STEP/parts/sts3215_servo_native.step")
def sts3215_servo_native():
    return make()


if __name__ == "__main__":
    sts3215_servo_native()
