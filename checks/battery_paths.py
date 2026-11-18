"""Saved STEP checks for the Zeee 2S pack's operating and service paths.

They cover the recorded walk/sit keyframes and the pack removal at the home
pose (lift, then withdraw rearward); no hardware commands.
"""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reports/latest'
# Parts changed or added for the LiPo holder; all are fastened to the trunk.
CHANGED = ('trunk_base', 'lipo_pod', 'left_shell_native', 'right_shell_native', 'zeee_2s_battery',
           'lipo_pad_and_leads', 'lipo_mount_screws', 'lipo_mount_inserts')
MOTION_GAP = 1.0      # mm; moving parts to the changed parts at every recorded keyframe
PACK_SERVICE_GAP = 0.5  # mm; pack sweeps to every installed part
LEAD_SERVICE_GAP = 0.15  # mm; lead and plug envelopes to every installed part


def _scene():
    import cadquery as cq
    from checks.common import read_scene, bounds
    scene = read_scene(ROOT / 'STEP/frankenduck.step')
    occurrences = {o.ref: o for o in scene.roots[0].children}
    shapes = {o.label: cq.Shape.cast(o.shape().wrapped) for o in occurrences.values()}
    return scene, occurrences, shapes, {name: bounds(shape) for name, shape in shapes.items()}


def motion():
    """Every articulated occurrence at recorded walk/sit keyframes against the battery area."""
    import itertools
    import numpy as np
    from checks.common import bound_gap, distance, volume, clip_data, joint_matrix, posed
    scene, occurrences, shapes, boxes = _scene()
    clips, operations = clip_data(ROOT)
    clip = clips['walk_sit_stand_walk_sit']
    moving = [op for op in operations if op['joints']]
    legs = [op for op in moving if any(j['name'].startswith(('left_', 'right_')) for j in op['joints'])]
    data = json.loads((ROOT/'source/assembly_data.json').read_text())
    descendants = {}
    for mate in data['mates']:
        descendants.setdefault(mate['parent'].lstrip('#'), []).append(mate['child'].lstrip('#'))
    pending = [m['child'].lstrip('#') for m in data['mates'] if m['name'] in ('left_hip_yaw', 'right_hip_yaw')]
    assert len(pending) == 2, 'Both hip yaw chains must be declared'
    expected = set()
    while pending:
        name = pending.pop()
        if name not in expected:
            expected.add(name)
            pending.extend(descendants.get(name, []))
    actual = {occurrences[op['target']].label for op in legs}
    assert actual == expected and len(legs) == len(actual), ('Leg motion coverage differs from the mate graph', sorted(expected-actual), sorted(actual-expected))
    moving_names = {occurrences[op['target']].label for op in moving}
    assert not moving_names & set(CHANGED), ('A changed battery part moves', sorted(moving_names & set(CHANGED)))

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
    bad = []; count = 0; exact = 0; cache = {}
    for k, (pose, t) in enumerate(clip['keys']):
        for op in moving:
            name = occurrences[op['target']].label
            m = joint_matrix(op['joints'], pose)
            points = (m @ corners[name].T).T[:, :3]
            box = [*points.min(0), *points.max(0)]
            candidate = None
            for target in minima:
                count += 1
                if bound_gap(box, boxes[target]) >= 2 or sat_gap(boxes[name], m, boxes[target]) >= 2:
                    continue
                key = (name, target, tuple(pose.get(j['name'], 0) for j in op['joints']))
                if key in cache:
                    gap = cache[key]
                else:
                    if candidate is None:
                        candidate = posed(shapes[name], op['joints'], pose)
                    gap = cache[key] = distance(candidate, shapes[target]); exact += 1
                if gap < minima[target]['gap_mm']:
                    minima[target] = {'part': name, 'time_s': t, 'gap_mm': gap}
                if gap < MOTION_GAP - 1e-5:
                    if candidate is None:
                        candidate = posed(shapes[name], op['joints'], pose)
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
              'moving_occurrences': len(moving), 'leg_occurrences': len(legs), 'leg_occurrence_names': sorted(actual),
              'targets': list(CHANGED), 'exact_distances': exact, 'near_minima': minima,
              'proven_sampled_gap_lower_bound_mm': certified, 'minimum_gap_mm': MOTION_GAP, 'below_minimum': bad,
              'passed': not bad,
              'scope': f'{len(moving)} articulated occurrences ({len(legs)} leg occurrences checked against the declared mate graph) '
                       'against the trunk, LiPo holder, pack, pad and leads, mount screws and inserts and both shells. '
                       'Keyframes only; linear interpolation and arbitrary joint combinations are not certified. '
                       'Distances of 2 mm or more may be rejected using conservative bounds.'}
    (OUT / 'battery_motion_verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print('MOTION_RESULT', {k: v for k, v in result.items() if k != 'below_minimum'}, flush=True)
    assert result['passed'], result['below_minimum']
    return result


def removal():
    """Pack service at the home pose: lift it clear of the rim, then withdraw it rearward.

    The pack sweeps are exact boxes over the seated pack's exact bounds. Each
    lead and plug is lifted as the union of full-travel prisms over exact 1 mm
    Y slices of it, then withdrawn as its full box: conservative envelopes of the
    continuous translations, not sampled positions. The foam pad stays in the
    holder. No lid, latch or fastener is removed.
    """
    import cadquery as cq
    import numpy as np
    from checks.common import bounds, bound_gap, pieces, pair_gap, clip_data, posed
    from lib import lipo_shoulders as ls
    scene, occurrences, shapes, boxes = _scene()
    _, operations = clip_data(ROOT)
    joints = {occurrences[op['target']].label: op['joints'] for op in operations}
    home = json.loads((ROOT / 'source/assembly_data.json').read_text())['poses']['home']

    pack = boxes['zeee_2s_battery']   # nominal body plus its 0.1 mm rear colour panels
    nominal = [-ls.PACK_X / 2, ls.PACK_CENTER_Y - ls.PACK_Y / 2, ls.PACK_BOTTOM,
               ls.PACK_X / 2, ls.PACK_CENTER_Y + ls.PACK_Y / 2 + 0.1, ls.PACK_BOTTOM + ls.PACK_Z]
    assert max(abs(a - b) for a, b in zip(pack, nominal)) < 1e-4, ('pack is not seated at its nominal box', pack)
    assert pack[2] + ls.REMOVAL_TRAVEL > ls.MOUTH_Z + 2, 'the lift must take the pack bottom over the rear rim'
    hardware = sorted(shapes['lipo_pad_and_leads'].Solids(), key=lambda s: bounds(s)[2])
    pad, leads = hardware[0], hardware[1:]
    assert bounds(pad)[5] <= ls.PACK_BOTTOM + 1e-4 and leads, 'one pad under the pack, then the leads'
    assert all(bounds(q)[2] > ls.MOUTH_Z for q in leads), 'every lead and plug must leave above the holder mouth'

    obstacles = {}
    for name, shape in shapes.items():
        if name in ('zeee_2s_battery', 'lipo_pad_and_leads'):
            continue
        q = posed(shape, joints.get(name, []), home)
        obstacles[name] = (pieces(q), bounds(q))
    obstacles['lipo_pad (stays in the holder)'] = (pieces(pad), bounds(pad))

    def box(b, lift=0.0, back=0.0, z0=0.0):
        return cq.Solid.makeBox(b[3] - b[0], b[4] - b[1] + back, b[5] - b[2] + lift - z0,
                                cq.Vector(b[0], b[1], b[2] + z0))

    def against(item, stage, sweeps, minimum, skip=()):
        rows = []
        for name, (other, other_box) in obstacles.items():
            if name in skip:
                continue
            near = [s for s in sweeps if bound_gap(bounds(s), other_box) < 2]
            if not near:
                continue
            row = pair_gap([q for s in near for q in pieces(s)], other, cap=2.0)
            row.update(item=item, stage=stage, other=name, minimum_mm=minimum,
                       passed=row['overlap_mm3'] < 1e-5 and row['gap_mm'] >= minimum - 1e-5)
            rows.append(row)
            if not row['passed']:
                print('REMOVAL_FAIL', row, flush=True)
        return rows

    rows = []
    # The pack lifts off its pad; the pad is not an obstacle to the lift.
    rows += against('pack', 'lift', [box(pack, lift=ls.REMOVAL_TRAVEL)], PACK_SERVICE_GAP, skip=('lipo_pad (stays in the holder)',))
    lifted = [pack[0], pack[1], pack[2] + ls.REMOVAL_TRAVEL, pack[3], pack[4], pack[5] + ls.REMOVAL_TRAVEL]
    rows += against('pack', 'rearward', [box(lifted, back=ls.REAR_EXIT_TRAVEL)], PACK_SERVICE_GAP)
    for i, lead in enumerate(leads, 1):
        b = bounds(lead)
        prisms = []
        for y in np.arange(b[1], b[4], 1.0):
            slab = cq.Solid.makeBox(b[3] - b[0] + 2, min(1.0, b[4] - y) + 1e-6, b[5] - b[2] + 2,
                                    cq.Vector(b[0] - 1, y - 5e-7, b[2] - 1))
            piece = lead.intersect(slab)
            if piece.Solids():
                prisms.append(box(bounds(piece), lift=ls.REMOVAL_TRAVEL))
        assert prisms, ('empty lead', i)
        rows += against(f'lead {i}', 'lift', prisms, LEAD_SERVICE_GAP)
        up = [b[0], b[1], b[2] + ls.REMOVAL_TRAVEL, b[3], b[4], b[5] + ls.REMOVAL_TRAVEL]
        rows += against(f'lead {i}', 'rearward', [box(up, back=ls.REAR_EXIT_TRAVEL)], LEAD_SERVICE_GAP)
    clear_of_holder = pack[1] + ls.REAR_EXIT_TRAVEL > boxes['lipo_pod'][4] + 1.0
    result = dict(document_sha256=scene.document_hash, units='mm', pose='home',
                  lift_mm=ls.REMOVAL_TRAVEL, rearward_mm=ls.REAR_EXIT_TRAVEL, pack_bounds_mm=pack,
                  pack_front_clear_of_holder_rear_after_withdrawal=clear_of_holder,
                  pack_minimum_mm=PACK_SERVICE_GAP, lead_minimum_mm=LEAD_SERVICE_GAP, leads=len(leads), pairs=rows,
                  passed=clear_of_holder and all(r['passed'] for r in rows),
                  scope='Home pose. Pack lift and rearward withdrawal as exact swept boxes; leads and plugs as '
                        'conservative prism envelopes, against every other occurrence posed at home and the foam pad. '
                        'Lead and plug shapes are reference envelopes, not measured cable routes. No hand or tool '
                        'room, cable bending or physical fit test.')
    (OUT / 'battery_removal_verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print('REMOVAL_RESULT', {k: v for k, v in result.items() if k != 'pairs'},
          min(r['gap_mm'] for r in rows), flush=True)
    assert result['passed'], [r for r in rows if not r['passed']]
    return result
