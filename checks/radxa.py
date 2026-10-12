"""Saved-rest-pose Radxa fit checks. Run through ./cad check radxa."""

def mount():
    """Check the saved local revision, including jumper plugs and cable routing.

    Run from the project root with its pinned interpreter. The checks cover
    the stored rest pose, not fasteners, physical wire bending or a motion sweep.
    """
    from pathlib import Path
    import ast
    import json
    import math
    import sys
    import time
    import cadquery as cq
    from checks.common import read_scene

    ROOT = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(ROOT / "src"))
    from lib.radxa_clearance import (clearance_shapes, PCB_SEAT_Y, PLUG_SIZE,
                                     PLUG_CENTRE, CABLE_SIZE, CABLE_CENTRE)

    TOLERANCE_MM3 = 1e-5
    current = read_scene(ROOT / "STEP/frankenduck.step")
    before = read_scene(ROOT / "revisions/r91_import/STEP/frankenduck.step")
    report = dict(document_sha256=current.document_hash,
                  baseline_document_sha256=before.document_hash, units="mm",
                  overlap_tolerance_mm3=TOLERANCE_MM3, pose="saved rest",
                  changed_occurrences=["#o1.3", "#o1.6", "#o1.10", "#o1.15", "#o1.16"],
                  added_occurrences=["#o1.74"], pairs=[])
    checks = []


    from checks.common import bounds, volume, intersection

    def pair(a_name, a, b_name, b):
        row = intersection(a_name, a, b_name, b, TOLERANCE_MM3)
        checks.append(row["passed"])
        report["pairs"].append(row)
        if row["method"] != "disjoint exact bounds" or not row["passed"]:
            print(row, flush=True)
        return row


    occurrences = {}
    for occurrence in current.roots[0].children:
        native = occurrence.shape()
        occurrences[occurrence.ref.lstrip("#")] = (occurrence.label, native,
                                       cq.Shape.cast(native.wrapped))

    bracket = occurrences["o1.6"][2]
    board = occurrences["o1.15"][2]
    bulkhead = occurrences["o1.3"][2]
    speaker = occurrences["o1.16"][2]
    heatsink = occurrences["o1.74"][2]
    plug, cable = [cq.Shape.cast(s.wrapped) for s in clearance_shapes()]
    targets = [("bulkhead", bulkhead, {"o1.3"}),
               ("bracket", bracket, {"o1.3", "o1.6"}),
               ("board", board, {"o1.3", "o1.6", "o1.15"}),
               ("speaker", speaker, {"o1.3", "o1.6", "o1.15", "o1.16"}),
               ("heatsink_representation", heatsink, {"o1.3", "o1.6", "o1.15", "o1.16", "o1.74"}),
               ("gpio_jumper_plug_allowance", plug, {"o1.15"}),
               ("gpio_upward_cable_allowance", cable, set())]
    # The socket allowance intentionally contains the existing GPIO header.
    # The cable allowance must be clear of the board as well as the other parts.
    for name, shape, skip in targets:
        for ref, (label, _, neighbour) in occurrences.items():
            if ref not in skip:
                pair(name, shape, ref + ":" + label, neighbour)

    baseline_path = ROOT / "revisions/r91_import/checks/radxa_initial_overlap.json"
    baseline = json.loads(baseline_path.read_text()) if baseline_path.exists() else {}
    if baseline.get("document_sha256") != before.document_hash:
        old_board = cq.Shape.cast(before.resolve("#o1.15").shape().wrapped)
        old_bracket = cq.Shape.cast(before.resolve("#o1.6").shape().wrapped)
        old_frame = cq.Shape.cast(before.resolve("#o1.2").shape().wrapped)
        baseline = dict(document_sha256=before.document_hash, overlap_mm3={
            "board_bracket": volume(old_board.intersect(old_bracket)),
            "board_head_frame": volume(old_board.intersect(old_frame))})
        baseline_path.write_text(json.dumps(baseline, indent=2))
    report["before_overlap_mm3"] = baseline["overlap_mm3"]

    native_board = occurrences["o1.15"][1]
    seat_faces = [f for f in native_board.faces()
                  if f.geom_type.name == "PLANE" and f.area > 1500
                  and abs(f.center().Y - PCB_SEAT_Y) < 1e-4]
    assert len(seat_faces) == 1, "Expected one large PCB mounting face"
    seat = seat_faces[0]
    pcb_holes = []
    for wire in seat.inner_wires():
        b = wire.bounding_box()
        pcb_holes.append([(b.min.X+b.max.X)/2, (b.min.Z+b.max.Z)/2])
    pcb_holes.sort(key=lambda p: (round(p[0], 6), round(p[1], 6)))
    bracket_holes = []
    for edge in occurrences["o1.6"][1].edges():
        if edge.geom_type.name == "CIRCLE" and abs(edge.radius - 1.2) < 1e-6:
            centre = edge.arc_center
            candidate = [centre.X, centre.Z]
            if not any(math.dist(candidate, p) < 1e-6 for p in bracket_holes):
                bracket_holes.append(candidate)
    bracket_holes.sort(key=lambda p: (round(p[0], 6), round(p[1], 6)))
    assert len(pcb_holes) == len(bracket_holes) == 4
    hole_errors = [math.dist(a, b) for a, b in zip(pcb_holes, bracket_holes)]
    bracket_seat_faces = [f for f in occurrences["o1.6"][1].faces()
                          if f.geom_type.name == "PLANE" and f.area > 40
                          and abs(f.center().Y - PCB_SEAT_Y) < 1e-4]
    assert bracket_seat_faces, "Expected rail seating faces at the PCB datum"
    bracket_seat_y = bracket_seat_faces[0].center().Y
    seat_gap = abs(seat.center().Y - bracket_seat_y)
    checks += [max(hole_errors) < 1e-4, seat_gap < 1e-4]
    report["seating"] = dict(pcb_seat_y_mm=seat.center().Y,
                             bracket_seat_y_mm=bracket_seat_y,
                             plane_gap_mm=seat_gap, pcb_holes_xz_mm=pcb_holes,
                             bracket_holes_xz_mm=bracket_holes,
                             max_hole_axis_error_mm=max(hole_errors))
    face_seat_gap = abs(bounds(bracket)[1] - bounds(occurrences["o1.14"][2])[4])
    face_hole_centres = []
    for edge in occurrences["o1.6"][1].edges():
        if edge.geom_type.name == "CIRCLE" and abs(edge.radius - 1.7) < 1e-6:
            centre = edge.arc_center
            candidate = [centre.X, centre.Z]
            if not any(math.dist(candidate, p) < 1e-6 for p in face_hole_centres):
                face_hole_centres.append(candidate)
    face_hole_centres.sort(key=lambda p: (round(p[0], 6), round(p[1], 6)))
    expected_face_holes = [[-39.17,286.07], [38.92,286.07], [38.92,316.94]]
    face_axis_errors = [math.dist(a,b) for a,b in zip(face_hole_centres, expected_face_holes)]
    checks += [face_seat_gap < 1e-4, len(face_hole_centres) == 3,
               len(face_axis_errors) == 3 and max(face_axis_errors) < 1e-4]
    report["face_attachment"] = dict(plane_gap_mm=face_seat_gap,
                                      m3_centres_xz_mm=face_hole_centres,
                                      expected_centres_xz_mm=expected_face_holes,
                                      max_axis_error_mm=max(face_axis_errors,default=None))
    bracket_depth = bounds(bracket)[4] - bounds(bracket)[1]
    checks.append(abs(bracket_depth - 3.2) < 1e-5)
    report["short_pads"] = dict(depth_mm=bracket_depth, previous_tube_depth_mm=17.02,
                                projection_beyond_rail_mm=0.0,
                                screw_head_allowance="90 degree M3 flat head, nominal diameter 5.6 mm")
    # Check the protected shaft passages and both types of screw-head seats.
    protected_bores = []
    for kind, centres, radius in [("M2", pcb_holes, 1.0), ("M3", face_hole_centres, 1.5)]:
        for x, z in centres:
            shaft = cq.Solid.makeCylinder(radius, 3.2, cq.Vector(x,-93.015,z), cq.Vector(0,1,0))
            v = volume(shaft.intersect(bracket))
            protected_bores.append(dict(kind=kind, centre_xz_mm=[x,z], obstruction_mm3=v))
            checks.append(v <= TOLERANCE_MM3)
    for x,z in pcb_holes:
        head = cq.Solid.makeCylinder(2.0,1.6,cq.Vector(x,-93.015,z),cq.Vector(0,1,0))
        v=volume(head.intersect(bracket));checks.append(v <= TOLERANCE_MM3)
        protected_bores.append(dict(kind="M2 pan-head well",centre_xz_mm=[x,z],obstruction_mm3=v))
    for x,z in face_hole_centres:
        head = cq.Solid.makeCone(1.7,2.8,1.1,cq.Vector(x,-90.915,z),cq.Vector(0,1,0))
        v=volume(head.intersect(bracket));checks.append(v <= TOLERANCE_MM3)
        protected_bores.append(dict(kind="M3 flat-head seat",centre_xz_mm=[x,z],obstruction_mm3=v))
        for ref in ["o1.15","o1.20"]:
            pair("M3 flat head",head,ref+":"+occurrences[ref][0],occurrences[ref][2])
    report["protected_bores"] = protected_bores
    report["clearance_allowances"] = dict(
        plug=dict(size_mm=PLUG_SIZE, centre_mm=PLUG_CENTRE,
                  bounds_mm=bounds(plug), depth_from_pcb_seat_mm=PLUG_SIZE[1],
                  extends_past_existing_header_tip_mm=bounds(plug)[4]-bounds(board)[4]),
        cable=dict(size_mm=CABLE_SIZE, centre_mm=CABLE_CENTRE,
                   bounds_mm=bounds(cable), direction="upward +Z"),
        plug_to_ubec_clamp_vertical_gap_mm=bounds(plug)[2] - bounds(occurrences["o1.7"][2])[5],
        pcb_to_camera_separation_lower_bound_mm=bounds(board)[1] - bounds(occurrences["o1.18"][2])[4],
        assumptions="Individual female jumper leads; exact housing and wire dimensions unmeasured.")

    source = ast.parse((ROOT / "src/frankenduck.py").read_text())
    placement = next(ast.literal_eval(n.value) for n in source.body
                     if isinstance(n, ast.Assign)
                     and any(isinstance(t, ast.Name) and t.id == "RADXA_BOARD_AT"
                             for t in n.targets))
    original_placement = json.loads((ROOT / "source/assembly_data.json").read_text())["instances"][14]["at"]
    same_rotation = json.loads(json.dumps(placement[1])) == original_placement[1]
    checks.append(same_rotation)
    report["board_orientation"] = dict(rotation=placement[1], rotation_unchanged=same_rotation,
                                        usb_c="upward, retained from the imported board placement")

    native_speaker = occurrences["o1.16"][1]
    speaker_seats = [f for f in native_speaker.faces()
                     if f.geom_type.name == "PLANE" and f.area > 900
                     and abs(f.center().Y + 60.0) < 1e-5]
    assert len(speaker_seats) == 1
    speaker_holes = []
    for wire in speaker_seats[0].inner_wires():
        b=wire.bounding_box()
        if (b.min.Z+b.max.Z)/2 < 300:
            speaker_holes.append([(b.min.X+b.max.X)/2,(b.min.Z+b.max.Z)/2])
    speaker_holes.sort(key=lambda p: (round(p[0],6),round(p[1],6)))
    pilot_holes=[]
    for edge in occurrences["o1.3"][1].edges():
        if edge.geom_type.name=="CIRCLE" and abs(edge.radius-.8)<1e-6:
            c=edge.arc_center;p=[c.X,c.Z]
            if not any(math.dist(p,q)<1e-6 for q in pilot_holes):pilot_holes.append(p)
    pilot_holes.sort(key=lambda p: (round(p[0],6),round(p[1],6)))
    speaker_errors=[math.dist(a,b) for a,b in zip(speaker_holes,pilot_holes)]
    checks += [len(speaker_holes)==len(pilot_holes)==2,
               len(speaker_errors)==2 and max(speaker_errors)<1e-5]
    report["speaker_mount"] = dict(seat_y_mm=speaker_seats[0].center().Y,
                                    lower_board_holes_xz_mm=speaker_holes,
                                    pilot_holes_xz_mm=pilot_holes,
                                    max_axis_error_mm=max(speaker_errors,default=None),
                                    rearward_shift_mm=5.0,magnet_recess_remaining_plate_mm=5.25)
    roi=cq.Solid.makeCylinder(9.4,6.1,cq.Vector(0,-59.1,280.58),cq.Vector(0,1,0))
    prior_bulkhead=cq.Shape.cast(before.resolve("#o1.3").shape().wrapped)
    old_roi=prior_bulkhead.intersect(roi);new_roi=bulkhead.intersect(roi)
    roi_delta=volume(old_roi.cut(new_roi))+volume(new_roi.cut(old_roi))
    checks.append(roi_delta<=TOLERANCE_MM3)
    report["speaker_mount"]["protected_bearing_seat_change_mm3"]=roi_delta

    # Compare all other geometry placements with the frozen original artifact.
    unchanged = []
    for ref, (label, _, shape) in occurrences.items():
        if ref in {"o1.3", "o1.6", "o1.10", "o1.15", "o1.16", "o1.74"}:
            continue
        prior = cq.Shape.cast(before.resolve("#" + ref).shape().wrapped)
        error = max(abs(a-b) for a, b in zip(bounds(shape), bounds(prior)))
        dv = abs(volume(shape)-volume(prior))
        passed = error < 1e-5 and dv < max(.02, volume(prior)*1e-6)
        checks.append(passed)
        unchanged.append(dict(ref=ref, name=label, bound_error_mm=error,
                              volume_delta_mm3=dv, passed=passed))
    report["unchanged_occurrences"] = unchanged
    report["modified_parts"] = {
        name:dict(valid=shape.isValid(), solids=len(shape.Solids()),
                  volume_mm3=volume(shape), bounds_mm=bounds(shape))
        for name, shape in [("bulkhead",bulkhead), ("bracket", bracket), ("board", board), ("speaker",speaker), ("heatsink_representation",heatsink)]}
    checks += [shape.isValid() for shape in [bulkhead,bracket, board,speaker,heatsink]]
    checks.append(len(bracket.Solids()) == 1)
    checks.append(len(bulkhead.Solids()) == 1)
    report["passed"] = all(checks)
    (ROOT / "reports/latest/radxa_mount_fit.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items()
                      if k not in {"pairs", "unchanged_occurrences"}}, indent=2), flush=True)
    assert report["passed"], "Radxa mount, connector, cable or unchanged-geometry check failed"

    return report


def heatsink():
    """Check the official 6530B outer dimensions and the saved representation.

    Detailed fan/fin/foot/notch dimensions and cooler PCB seating are assumptions;
    passing this check is spatial evidence, not exact-vendor or thermal validation.
    """
    from pathlib import Path
    import sys,json,math
    import cadquery as cq
    from checks.common import read_scene

    ROOT = Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(ROOT/'src'))
    from lib.radxa_6530b_parameters import outer_envelope,fan_inlet_allowance,HOLE_GRID
    from lib.radxa_clearance import clearance_shapes

    scene=read_scene(ROOT/'STEP/frankenduck.step')
    prior=read_scene(ROOT/'revisions/radxa_jumper_bay_before_heatsink/STEP/frankenduck.step')
    report=dict(document_sha256=scene.document_hash,units='mm',geometry_status='Dimensioned representation; detailed geometry approximate',tolerance_mm3=1e-5,pairs=[],assumptions=dict(outer_size_mm=[65,30,11.5],mounting_grid_mm=HOLE_GRID,pcb_component_plane_y_mm=-88.065,base_standoff_mm=5.0,fan_case_mm=21.5,inlet_radius_mm=8.5,clear_inlet_approach_mm=2.0,foot_diameter_mm=4.8,gpio_notch_mm=[53,7]))
    checks=[]
    shapes={x.ref.lstrip('#'):(x.label,cq.Shape.cast(x.shape().wrapped)) for x in scene.roots[0].children}
    from checks.common import volume, intersection
    def pair(name, a, ref, b):
        row = intersection(name, a, ref, b, 1e-5)
        report['pairs'].append(row)
        checks.append(row['passed'])
        if not row['passed']:
            print(row, flush=True)

    env=cq.Shape.cast(outer_envelope().wrapped);air=cq.Shape.cast(fan_inlet_allowance().wrapped)
    for name,a in [('official_outer_envelope',env),('fan_inlet_allowance',air)]:
     for ref,(label,b) in shapes.items():
      if ref=='o1.74' or (name=='official_outer_envelope' and ref=='o1.15'):continue
      pair(name,a,ref+':'+label,b)
    # The full bounding envelope intentionally includes GPIO/SoC regions that
    # actual cooler cutouts leave open. Actual-representation checks cover these.
    for name,p in zip(['plug_turn_bay','upward_cable_path'],clearance_shapes()):
     pair('fan_inlet_allowance',air,name,cq.Shape.cast(p.wrapped))
    old=cq.Shape.cast(prior.resolve('#o1.10').shape().wrapped);new=shapes['o1.10'][1]
    added=volume(new.cut(old));removed=volume(old.cut(new))
    checks += [added<=1e-5,new.isValid(),len(new.Solids())==1]
    report['cover_change']=dict(material_added_mm3=added,material_removed_mm3=removed,valid=new.isValid(),solids=len(new.Solids()),wall_samples=[])
    for y,z in [(-76.565,317.5),(-77.5,317.5),(-79.0,317.5)]:
     ray=cq.Solid.makeCylinder(.01,10,cq.Vector(-50,y,z),cq.Vector(1,0,0))
     wall=[p.BoundingBox().xlen for p in new.intersect(ray).Solids()]
     checks.append(bool(wall) and min(wall)>=2.0)
     report['cover_change']['wall_samples'].append(dict(y_mm=y,z_mm=z,thickness_mm=wall))
    # Official 57.8 x 22.9 mm grid versus the coarser imported 58 x 23 mm grid.
    axis_error=math.hypot((58-HOLE_GRID[0])/2,(23-HOLE_GRID[1])/2)
    report['mounting']=dict(grid_axis_offset_mm=axis_error,bracket_m2_radial_clearance_mm=.2,nominal_m2_shaft_fits=axis_error<.2,kit_screw_length_mm=4.0,modelled_bracket_grip_mm=1.6,modelled_pcb_thickness_mm=1.75,kit_thread_engagement_with_bracket_mm=4-1.6-1.75,suggested_longer_screw_review='M2 x 6 gives approximately 2.65 mm engagement; verify actual heatsink thread depth')
    checks.append(axis_error<.2)
    report['passed']=all(checks)
    (ROOT/'reports/latest/radxa_heatsink_fit.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='pairs'},indent=2),flush=True)
    assert report['passed'],'Heatsink envelope, inlet or cover check failed'

    return report
