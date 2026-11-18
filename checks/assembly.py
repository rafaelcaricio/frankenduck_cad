"""Current assembly declarations, animation and saved geometry inventory."""
from pathlib import Path
import json
import math

ROOT = Path(__file__).resolve().parents[1]


def metadata():
    data = json.loads((ROOT / 'source/assembly_data.json').read_text())
    side = json.loads((ROOT / 'STEP/frankenduck.step.json').read_text())
    expected = {mate['name']: mate for mate in data['mates']}
    actual = {mate['name']: mate for mate in side['kinematics']['mates']}
    assert len(expected) == len(data['mates']), 'Duplicate declared mate names'
    assert len(actual) == len(side['kinematics']['mates']), 'Duplicate saved mate names'
    assert expected.keys() == actual.keys(), 'Saved mate inventory differs'
    refs = {item['name']: f'o1.{i}' for i, item in enumerate(data['instances'], 1)}
    for name, mate in actual.items():
        declared = expected[name]
        for key in ('kind', 'parent', 'child'):
            assert mate[key] == declared[key], (name, key)
        assert mate['parentId'] == refs[declared['parent'][1:]], name
        assert mate['childId'] == refs[declared['child'][1:]], name
        if mate['kind'] in ('revolute', 'slider'):
            assert mate['axis']['origin'] == declared['origin'], name
            assert mate['axis']['dir'] == declared['direction'], name
            assert abs(math.sqrt(sum(v*v for v in declared['direction'])) - 1) < 1e-6, name
            assert mate['limits']['value'] == declared['limits'], name
    assert side['kinematics']['poses'] == data['poses'], 'Saved pose presets differ'

    source = (ROOT / 'src/animation.js').read_text()
    clips = json.loads(source.split('const DATA = ', 1)[1].split(';\nconst OPS = ', 1)[0])
    operations = json.loads(source.split('const OPS = ', 1)[1].split(';\n', 1)[0])
    assert side['animation']['source'] == source, 'Saved animation differs'
    joint_names = {name for name, mate in expected.items() if mate['kind'] in ('revolute', 'slider')}
    operation_refs = [operation['target'] for operation in operations]
    assert len(operation_refs) == len(set(operation_refs)), 'Duplicate animation target'
    assert set(operation_refs) == {'#' + ref for ref in refs.values()}, 'Animation target inventory differs'
    for operation in operations:
        for joint in operation['joints']:
            declared = expected[joint['name']]
            assert joint['axis'] == declared['direction'], joint['name']
            assert joint['origin'] == declared['origin'], joint['name']
            assert joint.get('kind', 'revolute') == declared['kind'], joint['name']
    for name, clip in clips.items():
        keys = clip['keys']
        assert keys and keys[0][1] == 0, name
        assert clip['duration'] == keys[-1][1], name
        assert all(a[1] < b[1] for a, b in zip(keys, keys[1:])), name
        for pose, timestamp in keys:
            assert pose.keys() <= joint_names, name
            assert math.isfinite(timestamp) and all(math.isfinite(v) for v in pose.values()), name
    board = next(operation for operation in operations if operation['target'] == '#o1.15')
    cooler = next(operation for operation in operations if operation['target'] == '#' + refs['radxa_6530b'])
    assert board['joints'] == cooler['joints'], 'Cooler must follow the board'

    report = dict(passed=True, mates=len(actual),
        revolute=sum(mate['kind'] == 'revolute' for mate in actual.values()),
        slider=sum(mate['kind'] == 'slider' for mate in actual.values()),
        fastened=sum(mate['kind'] == 'fastened' for mate in actual.values()),
        heatsink_animation_matches_board=True, poses=list(data['poses']),
        animations={name: dict(keyframes=len(clip['keys']), duration_s=clip['duration'], loop=clip['loop'])
                    for name, clip in clips.items()}, jaw_display_range_degrees=expected['jaw']['limits'])
    (ROOT / 'reports/latest/metadata_verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print(report)
    return report


def inventory():
    from checks.common import read_scene
    path = ROOT / 'STEP/frankenduck.step'
    scene = read_scene(path)
    data = json.loads((ROOT / 'source/assembly_data.json').read_text())
    root = scene.roots[0]
    assert len(root.children) == len(data['instances']), 'Saved occurrence count differs'
    rows = []
    for expected, actual in zip(data['instances'], root.children):
        assert actual.label == expected['name'], (actual.label, expected['name'])
        shape = actual.shape()
        box = shape.bounding_box()
        volume = sum(solid.volume for solid in shape.solids())
        extent = [*box.min, *box.max]
        assert shape.solids() and volume > 0 and math.isfinite(volume), actual.ref
        assert all(math.isfinite(v) for v in extent), actual.ref
        rows.append(dict(name=actual.label, ref=actual.ref, solids=len(shape.solids()),
                         volume_mm3=volume, extent_mm=extent))
    report = dict(passed=True, document_sha256=scene.document_hash,
        occurrences=len(rows), solids=sum(row['solids'] for row in rows),
        volume_mm3=sum(row['volume_mm3'] for row in rows),
        extent_mm=[*[min(row['extent_mm'][i] for row in rows) for i in range(3)],
                   *[max(row['extent_mm'][i+3] for row in rows) for i in range(3)]], instances=rows)
    (ROOT / 'reports/latest/assembly_verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print({key: value for key, value in report.items() if key != 'instances'})
    return report


def catalog():
    """Metadata-only coverage of every part script and assembly occurrence."""
    from tools.components import summary
    report = summary(ROOT)
    (ROOT / 'reports/latest/component_classification.json').write_text(json.dumps(report, indent=2) + '\n')
    print({key: value for key, value in report.items() if key != 'models'})
    return report


def battery_pod():
    """Zeee 2S holder at rest: printed solids, seated pack, gaps, static contacts, clamp and closed shoulders.

    Nominal CAD evidence. The pack is a published-dimension reference; leads are
    visual envelopes. No physical fit, print or strength test. Service and motion
    paths are in checks/battery_paths.py; the four screw joints in checks/part_quality.py.
    """
    import itertools
    import cadquery as cq
    from checks.common import read_scene, bounds, volume, pieces, pair_gap, bound_gap
    from lib import lipo_shoulders as ls, lipo_mount as lm, body_shell_fasteners as bf, stick_pod as sp

    tolerance = 1e-5  # mm3; coincident-face numerical tolerance
    scene = read_scene(ROOT / 'STEP/frankenduck.step')
    shapes = {part.label: cq.Shape.cast(part.shape().wrapped) for part in scene.roots[0].children}
    parts = {name: pieces(shape) for name, shape in shapes.items()}
    boxes = {name: bounds(shape) for name, shape in shapes.items()}
    shell_names = ('left_shell_native', 'right_shell_native')
    retired = {'stick_pod', 'parkside_battery', 'power_switch_kcd1_native', 'backpack_dock', 'parkside_pap_204_a1'}
    assert not retired & shapes.keys(), ('retired battery parts in the assembly', sorted(retired & shapes.keys()))

    prints = {}
    for name in ('trunk_base', 'lipo_pod', *shell_names):
        shape = shapes[name]
        prints[name] = dict(valid=shape.isValid(), solids=len(shape.Solids()), volume_mm3=volume(shape))
        assert shape.isValid() and len(shape.Solids()) == 1, name

    # The pack is seated at its published size, top 8 mm above the rear rim.
    variant = json.loads((ROOT / 'source/lipo_variant.json').read_text())
    # The pack's largest solid is its nominal body; 0.1 mm colour panels sit on its rear face.
    body = max(shapes['zeee_2s_battery'].Solids(), key=lambda q: q.Volume())
    pack = bounds(body)
    size = sorted(pack[i + 3] - pack[i] for i in range(3))
    seated = dict(body_bounds_mm=pack, size_mm=size, published_mm=sorted(variant['published_pack_dimensions_mm']),
                  with_panels_bounds_mm=boxes['zeee_2s_battery'],
                  center_x_mm=(pack[0] + pack[3]) / 2, center_y_mm=(pack[1] + pack[4]) / 2,
                  top_above_rim_mm=pack[5] - ls.MOUTH_Z)
    seated['passed'] = (max(abs(a - b) for a, b in zip(size, seated['published_mm'])) < 1e-3
                        and abs(seated['center_x_mm']) < 1e-4 and abs(seated['center_y_mm'] - variant['pack_center_y_mm']) < 1e-4
                        and abs(seated['top_above_rim_mm'] - variant['pack_exposed_above_rear_rim_mm']) < 1e-3)

    # Gaps between the changed parts; nominal values are in src/lib/lipo_*.py.
    gaps = []
    def gap(interface, a, b, minimum, a_pieces=None, b_pieces=None):
        result = pair_gap(a_pieces or parts[a], b_pieces or parts[b], cap=10.0)
        row = dict(interface=interface, minimum_mm=minimum, measured_mm=result['gap_mm'], overlap_mm3=result['overlap_mm3'],
                   passed=result['gap_mm'] >= minimum - 1e-5 and result['overlap_mm3'] < tolerance)
        gaps.append(row)
        return row
    for name in shell_names:
        gap(name + ' to holder', name, 'lipo_pod', ls.SHELL_GAP - 0.05)
        gap('pack to ' + name, 'zeee_2s_battery', name, 1.0)
    gap('pack in holder cavity', 'zeee_2s_battery', 'lipo_pod', 0.55)
    gap('pack to trunk', 'zeee_2s_battery', 'trunk_base', 1.0)

    # The shells meet only at their two internal ties; elsewhere the centre split stays open.
    ties = [cq.Solid.makeCylinder(bf.BOSS_RADIUS + bf.FRONT_ROOT_FILLET + 0.5, 4.0, cq.Vector(-2.0, y, z), cq.Vector(1, 0, 0))
            for y, z in bf.CENTERS]
    zone = ties[0].fuse(ties[1])
    left, right = (shapes[n] for n in shell_names)
    split = gap('shell centre split, outside the ties', *shell_names, 2 * sp.SHELL_SPLIT_HALF_GAP,
                pieces(left.cut(zone)), pieces(right.cut(zone)))
    tie_rows = []
    for (y, z), tool in zip(bf.CENTERS, ties):
        result = pair_gap(pieces(left.intersect(tool)), pieces(right.intersect(tool)), cap=1.0)
        tie_rows.append(dict(tie_center_yz_mm=[y, z], gap_mm=result['gap_mm'], overlap_mm3=result['overlap_mm3'],
                             passed=result['gap_mm'] < 1e-6 and result['overlap_mm3'] < tolerance))

    # Static overlaps of every changed part with every other occurrence. The new
    # LiPo parts may touch only at the declared interfaces; the heat-set inserts are
    # melted into their pilots and the modeled leads start inside the pack's top face.
    # The trunk and shells keep their existing mounting contacts, which are listed.
    contact = {frozenset(p): why for p, why in (
        (('lipo_pod', 'trunk_base'), 'arm support faces clamp the plate underside'),
        (('lipo_mount_screws', 'trunk_base'), 'screw heads seat in the plate recesses'),
        (('lipo_mount_screws', 'lipo_mount_inserts'), 'screw threads engage the inserts'),
        (('lipo_mount_screws', 'lipo_pod'), 'screw tips in the 3 mm passages below the inserts'),
        (('lipo_mount_inserts', 'trunk_base'), 'insert tops flush with the arm clamp face'),
        (('lipo_pad_and_leads', 'lipo_pod'), 'foam pad rests on the holder floor'))}
    pressed = {frozenset(('lipo_mount_inserts', 'lipo_pod')): 'heat-set inserts melted into the 4 mm pilots',
               frozenset(('lipo_pad_and_leads', 'zeee_2s_battery')): 'pack rests on the pad; lead ends start inside the pack top'}
    new_parts = {'lipo_pod', 'zeee_2s_battery', 'lipo_pad_and_leads', 'lipo_mount_screws', 'lipo_mount_inserts'}
    changed = ('lipo_pod', 'zeee_2s_battery', 'lipo_pad_and_leads', 'lipo_mount_screws', 'lipo_mount_inserts',
               'trunk_base', *shell_names)
    static = []; contacts = []
    for i, a in enumerate(changed):
        for b in shapes:
            if b == a or (b in changed and changed.index(b) < i) or bound_gap(boxes[a], boxes[b]) > 1e-7:
                continue
            result = pair_gap(parts[a], parts[b], cap=0.5)
            key = frozenset((a, b))
            row = dict(a=a, b=b, gap_lower_bound_mm=result['gap_mm'], overlap_mm3=result['overlap_mm3'],
                       interface=contact.get(key) or pressed.get(key))
            row['passed'] = key in pressed or result['overlap_mm3'] < tolerance
            declared = key in contact or key in pressed
            if 'lipo_pod' in key and not declared and result['gap_mm'] < 0.3:
                row['passed'] = False
            static.append(row)
            if result['gap_mm'] <= 1e-6:
                contacts.append(f'{a} / {b}')
                if key & new_parts and not declared:
                    row['passed'] = False
    insert_fit = []
    for x, y in lm.SCREWS:
        pilot = cq.Solid.makeCylinder(lm.INSERT_PILOT_D / 2 + 1.0, lm.INSERT_DEPTH + 0.2, cq.Vector(x, y, lm.ARM_TOP - lm.INSERT_DEPTH - 0.1))
        inside = volume(shapes['lipo_mount_inserts'].intersect(shapes['lipo_pod']).intersect(pilot))
        insert_fit.append(dict(x_mm=x, y_mm=y, melt_overlap_in_pilot_mm3=inside))
    melt_total = volume(shapes['lipo_mount_inserts'].intersect(shapes['lipo_pod']))
    melt = dict(per_insert=insert_fit, total_mm3=melt_total,
                passed=abs(sum(r['melt_overlap_in_pilot_mm3'] for r in insert_fit) - melt_total) < 1e-4)
    # The modeled leads overlap the pack only where they start at its top face; the pad only touches it.
    lead_overlap = shapes['lipo_pad_and_leads'].intersect(shapes['zeee_2s_battery'])
    lb = bounds(lead_overlap) if lead_overlap.Solids() else None
    leads = dict(overlap_mm3=volume(lead_overlap), bounds_mm=lb,
                 passed=lb is not None and lb[2] >= pack[5] - 1.5 and lb[5] <= pack[5] + 1e-4)

    # Arm clamp: each arm's top face bears on the plate underside wherever the plate
    # covers it. The plate ends at its rear edge; the arms continue to the front panel.
    plate_rear = boxes['trunk_base'][4]
    clamp = dict(plate_rear_edge_y_mm=plate_rear, arms=[])
    for sign in (-1, 1):
        x0, x1 = sorted((sign * lm.ARM_X0, sign * lm.ARM_X1))
        def slab(z0, z1, y1):
            return cq.Solid.makeBox(x1 - x0, y1 - lm.ARM_FRONT_Y, z1 - z0, cq.Vector(x0, lm.ARM_FRONT_Y, z0))
        arm_face = shapes['lipo_pod'].intersect(slab(lm.ARM_TOP - 0.05, lm.ARM_TOP, plate_rear))
        plate_face = shapes['trunk_base'].intersect(slab(lm.ARM_TOP, lm.ARM_TOP + 0.05, plate_rear))
        area = volume(arm_face) / 0.05
        bearing = volume(arm_face.translate(cq.Vector(0, 0, 0.05)).intersect(plate_face)) / 0.05
        clamp['arms'].append(dict(x_mm=[x0, x1], arm_top_under_plate_mm2=area, bearing_area_mm2=bearing,
                                  passed=area > 80 and bearing >= 0.99 * area))
    clamp['passed'] = all(r['passed'] for r in clamp['arms'])

    # Closed side shoulders: solid holder wall beside the access, from the rim to the roof.
    shoulders = []
    for x, y, z in itertools.product((-27.0, 27.0), (42.0, 48.0, 54.0), (158.0, 164.0, 168.0)):
        ray = cq.Edge.makeLine(cq.Vector(x - 1, y, z), cq.Vector(x + 1, y, z))
        depth = sum(e.Length() for e in ray.intersect(shapes['lipo_pod']).Edges())
        shoulders.append(dict(x_mm=x, y_mm=y, z_mm=z, solid_probe_mm=depth, passed=abs(depth - 2.0) < 0.01))

    checks = [seated, *gaps, *tie_rows, *static, melt, leads, clamp, *shoulders]
    report = dict(document_sha256=scene.document_hash, units='mm', pose='saved zero pose', overlap_tolerance_mm3=tolerance,
                  prints=prints, seated_pack=seated, gaps=gaps, shell_ties=tie_rows, contacts=contacts,
                  insert_melt=melt, lead_starts=leads, clamp=clamp, closed_shoulders=shoulders, static_pairs=static,
                  coverage='Valid single-solid trunk, holder and shells; pack seated at its published size and datum; '
                           'shell-holder, pack-cavity, pack-shell and pack-trunk gaps; shells touch only at their two ties '
                           'with the 0.6 mm split elsewhere; static overlaps of the changed parts with every occurrence '
                           '(the new parts touch only at declared interfaces, the holder keeps 0.3 mm from everything '
                           'else, the inserts overlap the holder only in their pilots, the leads overlap the pack only at '
                           'its top face); full arm-to-plate bearing under the plate; solid closed '
                           'shoulders. The pack, pad and leads are references. No slicing, physical fit or strength test.',
                  passed=all(row['passed'] for row in checks))
    (ROOT / 'reports/latest/battery_pod_verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print({key: value for key, value in report.items() if key != 'static_pairs'})
    assert report['passed'], [row for row in checks if not row['passed']]
    return report
