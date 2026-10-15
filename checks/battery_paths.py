"""Saved STEP checks for the stick battery's operating and service paths.

They cover the recorded walk/sit keyframes and the straight-down release at the
home pose; no hardware commands.
"""
from pathlib import Path
import json
import math

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reports/latest'
CHANGED = ('trunk_base', 'stick_pod', 'left_shell_native', 'right_shell_native', 'parkside_battery')


def _scene():
    import cadquery as cq
    from checks.common import read_scene, bounds
    scene = read_scene(ROOT / 'STEP/frankenduck.step')
    occurrences = {o.ref: o for o in scene.roots[0].children}
    shapes = {o.label: cq.Shape.cast(o.shape().wrapped) for o in occurrences.values()}
    return scene, occurrences, shapes, {name: bounds(shape) for name, shape in shapes.items()}


def motion():
    """All recorded walk/sit keyframes: 30 leg occurrences against the battery, pod, trunk and shells."""
    import itertools
    import numpy as np
    from checks.common import bound_gap, distance, volume, clip_data, joint_matrix, posed
    scene, occurrences, shapes, boxes = _scene()
    clips, operations = clip_data(ROOT)
    clip = clips['walk_sit_stand_walk_sit']
    legs = [op for op in operations if any(j['name'].startswith(('left_', 'right_')) for j in op['joints'])]
    assert len(legs) == 30, 'Leg occurrence coverage changed'

    def sat_gap(moving_box, m, other_box):
        # Exact boxes enclose the B-reps; projected separation is a conservative
        # lower bound on real part clearance, including after joint rotation.
        a = np.array(moving_box); b = np.array(other_box)
        ca = m[:3, :3] @ ((a[:3] + a[3:]) / 2) + m[:3, 3]; ea = (a[3:] - a[:3]) / 2
        cb = (b[:3] + b[3:]) / 2; eb = (b[3:] - b[:3]) / 2; R = m[:3, :3]
        axes = [*R.T, *np.eye(3), *[np.cross(x, y) for x in R.T for y in np.eye(3)]]
        best = 0
        for ax in axes:
            norm = np.linalg.norm(ax)
            if norm < 1e-9:
                continue
            ax = ax / norm
            best = max(best, abs((ca - cb) @ ax) - np.abs(ax @ R) @ ea - np.abs(ax) @ eb)
        return best

    corners = {n: np.array([(*p, 1) for p in itertools.product(*[(b[i], b[i+3]) for i in range(3)])]) for n, b in boxes.items()}
    minima = {name: {'gap_mm': 1000} for name in CHANGED}
    bad = []; count = 0; exact = 0
    for k, (pose, t) in enumerate(clip['keys']):
        for op in legs:
            name = occurrences[op['target']].label
            m = joint_matrix(op['joints'], pose)
            points = (m @ corners[name].T).T[:, :3]
            box = [*points.min(0), *points.max(0)]
            candidate = None
            for target in minima:
                count += 1
                if bound_gap(box, boxes[target]) >= 2 or sat_gap(boxes[name], m, boxes[target]) >= 2:
                    continue
                if candidate is None:
                    candidate = posed(shapes[name], op['joints'], pose)
                gap = distance(candidate, shapes[target]); exact += 1
                if gap < minima[target]['gap_mm']:
                    minima[target] = {'part': name, 'time_s': t, 'gap_mm': gap}
                if gap < 1 - 1e-5:
                    overlap = volume(candidate.intersect(shapes[target])) if gap < 1e-6 else 0
                    row = {'part': name, 'target': target, 'time_s': t, 'gap_mm': gap, 'overlap_mm3': overlap}
                    bad.append(row)
                    if len(bad) < 12:
                        print('MOTION_FAIL', row, flush=True)
        if k % 100 == 0:
            print('motion', k, len(clip['keys']), count, exact, flush=True)
    certified = min([2.0] + [row['gap_mm'] for row in minima.values()])
    for target, row in list(minima.items()):
        if row['gap_mm'] == 1000:
            minima[target] = {'lower_bound_mm': 2.0, 'method': 'all pairs separated by conservative bounds'}
        else:
            row['minimum_exact_tested_gap_mm'] = row.pop('gap_mm')
    result = {'document_sha256': scene.document_hash, 'units': 'mm', 'keyframes': len(clip['keys']), 'pairs': count,
              'exact_distances': exact, 'near_minima': minima, 'proven_sampled_gap_lower_bound_mm': certified,
              'minimum_gap_mm': 1.0, 'below_1mm': bad, 'passed': not bad,
              'scope': '30 leg occurrences against the battery, stick pod, trunk and both shells. Keyframes only; '
                       'linear interpolation is not certified. Distances of 2 mm or more may be rejected using conservative bounds.'}
    (OUT / 'battery_motion_verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print('MOTION_RESULT', {k: v for k, v in result.items() if k != 'below_1mm'}, flush=True)
    assert result['passed'], result['below_1mm']
    return result


def removal():
    """Straight-down release at the home pose, the hook travel through the lips and thumb room at the buttons.

    The battery splits into the material inside its plan column (the stick
    outline grown 0.3 mm, up to 0.1 mm over the stick top) and the pieces
    outside it. Column material stays in the column on the way down, so a
    column that meets no part is clear over the whole travel. Each outside
    piece stays inside its box stretched down by the travel; a swept box that
    meets no part proves that piece clear. Pieces whose swept box meets a part
    are sampled along the path with exact booleans. The stick model's latch is
    APPROXIMATE; overlap of the rigid hooks with the pod lips stands for the
    hooks flexing inward and is allowed only inside the lip zone, up to the
    flex room the pod provides.
    """
    import time
    import numpy as np
    import cadquery as cq
    from checks.common import bounds, bound_gap, distance, volume, pieces, pair_gap, clip_data, posed
    from lib import stick_pod as sp
    from parts import parkside_papk_12_a4 as stick
    start = time.monotonic()
    tolerance = 1e-5
    scene, occurrences, shapes, boxes = _scene()
    clips, operations = clip_data(ROOT)
    clip = clips['battery_removal']
    by_label = {occurrences[op['target']].label: op for op in operations}
    release = next(j for j in by_label['parkside_battery']['joints'] if j['name'] == 'battery_release')
    assert release['kind'] == 'slider' and tuple(release['axis']) == sp.RELEASE_DIRECTION == (0.0, 0.0, -1.0), release
    home = {k: v for k, v in clip['keys'][0][0].items() if k != 'battery_release'}
    travel = max(pose.get('battery_release', 0) for pose, _ in clip['keys'])
    assert travel == sp.RELEASE_TRAVEL, travel
    assert all({k: v for k, v in pose.items() if k != 'battery_release'} == home for pose, _ in clip['keys']), 'Release clip must hold one pose'

    # Every other occurrence at the clip's pose.
    placed = {}
    for name, shape in shapes.items():
        if name == 'parkside_battery':
            continue
        joints = by_label[name]['joints'] if name in by_label else []
        placed[name] = posed(shape, joints, home) if any(home.get(j['name'], 0) for j in joints) else shape
    placed_boxes = {name: bounds(shape) for name, shape in placed.items()}
    placed_pieces = {name: pieces(shape) for name, shape in placed.items()}
    pod = placed['stick_pod']

    # Column: from the stick bottom at full travel to 0.1 mm over the stick top.
    battery_box = boxes['parkside_battery']
    top = battery_box[5] - sp.STICK_ORIGIN[2] + 0.1
    bottom = battery_box[2] - sp.STICK_ORIGIN[2] - travel - 0.1
    column = sp.local_to_world(sp.prism('stick', sp.CLEARANCE / 2, bottom, top))
    column_box = bounds(column)
    column_rows = []
    for name in placed:
        if bound_gap(column_box, placed_boxes[name]) >= 3:
            continue
        near = [q for q, b in placed_pieces[name] if bound_gap(column_box, b) < 3]
        overlap = sum(volume(column.intersect(q)) for q in near)
        column_rows.append(dict(part=name, gap_lower_bound_mm=min(distance(column, q) for q in near),
                                overlap_mm3=overlap, passed=overlap < tolerance))
    column_proof = dict(stick_outline_grow_mm=sp.CLEARANCE / 2, world_z_mm=[column_box[2], column_box[5]],
                        parts_within_3mm=column_rows, passed=all(r['passed'] for r in column_rows))

    # Outside pieces and their swept boxes.
    outside = []
    for q, _ in pieces(shapes['parkside_battery']):
        rest = q.cut(column)
        if volume(rest) >= tolerance:
            outside += pieces(rest)
    swept = []
    for index, (q, b) in enumerate(outside):
        sb = [b[0], b[1], b[2] - travel, b[3], b[4], b[5]]
        box = cq.Solid.makeBox(sb[3] - sb[0], sb[4] - sb[1], sb[5] - sb[2], cq.Vector(*sb[:3]))
        meets = {}
        for name in placed:
            if bound_gap(sb, placed_boxes[name]) > 1e-7:
                continue
            hit = sum(volume(box.intersect(x)) for x, bx in placed_pieces[name] if bound_gap(sb, bx) <= 1e-7)
            if hit > tolerance:
                meets[name] = hit
        swept.append(dict(piece=index, volume_mm3=volume(q), bounds_mm=b, swept_box_meets_mm3=meets))
    print('swept', len(outside), [(r['piece'], list(r['swept_box_meets_mm3'])) for r in swept if r['swept_box_meets_mm3']],
          f'{time.monotonic() - start:.0f}s', flush=True)

    # Lip zone: where the rigid hook may pass through the pod's lips.
    face = -(stick.collar_inset + stick.tooth_depth[0])
    notch_face = face - stick.tooth_notch
    lip_inner = notch_face + sp.LIP_CLEAR
    lip_top = stick.tooth_catch - sp.LIP_GAP
    zone = sp.local_to_world(sp._tooth_band(lip_inner - 0.02, face + 0.02, stick.rim_height - 1.0, lip_top + 0.02))
    engagement = face - lip_inner

    # Exact booleans every 0.25 mm wherever a met piece's box touches the part.
    path = sorted(set(round(v, 3) for v in np.r_[np.arange(0, travel, 0.25), travel]))
    rows = []; extent = {-1: 0.0, 1: 0.0}; zone_overlap = 0.0; last_contact = {}
    for row in swept:
        q, b = outside[row['piece']]
        for name in row['swept_box_meets_mm3']:
            for d in path:
                moved_box = [b[0], b[1], b[2] - d, b[3], b[4], b[5] - d]
                near = [x for x, bx in placed_pieces[name] if bound_gap(moved_box, bx) <= 1e-7]
                if not near:
                    continue
                moved = q.translate(cq.Vector(0, 0, -d))
                inside = outside_overlap = 0.0
                for x in near:
                    common = moved.intersect(x)
                    if volume(common) < 1e-9:
                        continue
                    if name != 'stick_pod':
                        outside_overlap += volume(common)
                        continue
                    inside += volume(common.intersect(zone))
                    outside_overlap += volume(common.cut(zone))
                    for solid in common.Solids():
                        sb = bounds(solid)
                        side = 1 if sb[0] + sb[3] > 0 else -1
                        extent[side] = max(extent[side], sb[3] - sb[0])
                if inside + outside_overlap > 1e-9:
                    last_contact[name] = max(last_contact.get(name, 0.0), d)
                zone_overlap = max(zone_overlap, inside)
                rows.append(dict(travel_mm=d, piece=row['piece'], part=name, lip_zone_overlap_mm3=inside,
                                 overlap_mm3=outside_overlap, passed=outside_overlap < tolerance))
                if d % 5 == 0:
                    print('release', row['piece'], name, d, round(inside, 3), round(outside_overlap, 6),
                          f'{time.monotonic() - start:.0f}s', flush=True)

    # Flex room behind the hooks: free of pod material over the hook height.
    back = -(stick.collar_inset + stick.tooth_depth[1])
    room = sp.local_to_world(sp._tooth_band(back - sp.TOOTH_FLEX + 0.02, back, lip_top, stick.tooth_top))
    # Hook overlap lies inside the zone, so the inward deflection is at most
    # the zone's depth from the cap outline; the X extent only shows both
    # hooks engage, since the outline curves along the hook.
    room_overlap = volume(pod.intersect(room))
    deflection = engagement + 0.04
    flex = dict(parametric_engagement_mm=engagement, inward_deflection_bound_mm=deflection,
                flex_room_mm=sp.TOOTH_FLEX, flex_room_pod_overlap_mm3=room_overlap, lip_zone_overlap_max_mm3=zone_overlap,
                hook_overlap_x_extent_mm={'-X': extent[-1], '+X': extent[1]},
                passed=(min(extent.values()) > 0 and room_overlap < tolerance and
                        deflection <= sp.TOOTH_FLEX - 0.3 and engagement > 0.5))

    # Thumb and finger room outside both release buttons.
    cap = sp.cap_outline(0.0)
    y0, y1 = min(y for y, _ in stick.button_outline), max(y for y, _ in stick.button_outline)
    z0, z1 = min(z for _, z in stick.button_outline), max(z for _, z in stick.button_outline)
    half = max(abs(x) for x, y in cap if y0 - 1 <= y <= y1 + 1)
    thumbs = []
    for side in (-1, 1):
        x0, x1 = sorted((side * (half + 1.5), side * (half + 21.5)))
        finger = sp.local_to_world(cq.Solid.makeBox(x1 - x0, y1 - y0 + 2, z1 - z0, cq.Vector(x0, y0 - 1, z0)))
        hits = {}
        for name, other in placed_pieces.items():
            if bound_gap(bounds(finger), placed_boxes[name]) > 1e-7:
                continue
            result = pair_gap(pieces(finger), other, cap=1.0)
            if result['overlap_mm3'] > tolerance:
                hits[name] = result['overlap_mm3']
        thumbs.append(dict(side='+X' if side > 0 else '-X', local_x_mm=[x0, x1], local_y_mm=[y0 - 1, y1 + 1],
                           local_z_mm=[z0, z1], blocked_by=hits, passed=not hits))

    sampled = sorted({(r['piece'], r['part']) for r in rows})
    swept_ok = all(set(r['swept_box_meets_mm3']) <= {part for piece, part in sampled if piece == r['piece']} for r in swept)
    passed = (column_proof['passed'] and swept_ok and all(r['passed'] for r in rows) and flex['passed'] and
              all(t['passed'] for t in thumbs))
    result = dict(document_sha256=scene.document_hash, units='mm', pose='battery_removal clip (home)',
                  direction=list(sp.RELEASE_DIRECTION), travel_mm=travel, column=column_proof,
                  outside_pieces=swept, sampled_pairs=[dict(piece=piece, part=part) for piece, part in sampled],
                  sample_step_mm=0.25, last_contact_travel_mm=last_contact, latch=flex, thumb_room=thumbs,
                  rows=rows, passed=passed,
                  scope='Home pose; straight-down release over the full travel. Battery material inside the stick '
                        'outline grown 0.3 mm is proven clear of every part by its column; each piece outside it is '
                        'proven clear by its swept box or, where that box meets a part, checked with exact booleans '
                        'every 0.25 mm while their boxes touch. Rigid hooks may overlap the pod only inside the lip '
                        'zone. The latch and buttons are APPROXIMATE; no arm bending, button force or physical fit '
                        'is modelled.')
    (OUT / 'battery_removal_verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print('REMOVAL_RESULT', {k: v for k, v in result.items() if k not in ('rows',)}, flush=True)
    assert passed, ([r for r in rows if not r['passed']], column_proof, swept_ok, flex, thumbs)
    return result
