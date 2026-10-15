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
    """Stick pod at rest: printed solids, gaps, static overlaps, tongue screws and adapter spaces.

    Bounded CAD evidence for the APPROXIMATE photo-derived stick; no physical fit,
    print or strength test. Paths are in checks/battery_paths.py.
    """
    import cadquery as cq
    from checks.common import read_scene, bounds, volume, pieces, pair_gap, bound_gap
    from lib import stick_pod as sp

    tolerance = 1e-5  # mm3; coincident-face numerical tolerance
    scene = read_scene(ROOT / 'STEP/frankenduck.step')
    shapes = {part.label: cq.Shape.cast(part.shape().wrapped) for part in scene.roots[0].children}
    parts = {name: pieces(shape) for name, shape in shapes.items()}
    boxes = {name: bounds(shape) for name, shape in shapes.items()}
    shell_names = ('left_shell_native', 'right_shell_native')
    assert not {'backpack_dock', 'parkside_pap_204_a1'} & shapes.keys(), 'PAP 204 A1 parts must not be in this variant'

    prints = {}
    for name in ('trunk_base', 'stick_pod', *shell_names):
        shape = shapes[name]
        prints[name] = dict(valid=shape.isValid(), solids=len(shape.Solids()), volume_mm3=volume(shape))
        assert shape.isValid() and len(shape.Solids()) == 1, name
    battery_bounds = boxes['parkside_battery']
    expected = [-22.59, 32.27, 81.5, 22.59, 83.72, 163.5]
    assert max(abs(a - b) for a, b in zip(battery_bounds, expected)) < 0.01, battery_bounds

    # Gaps between the changed parts; nominal values are in src/lib/stick_pod.py.
    gaps = []
    def gap(interface, a, b, minimum):
        result = pair_gap(parts[a], parts[b], cap=10.0)
        row = dict(interface=interface, minimum_mm=minimum, measured_mm=result['gap_mm'],
                   passed=result['gap_mm'] >= minimum - 1e-5)
        gaps.append(row)
        return row
    gap('shell centre split', *shell_names, 2 * sp.SHELL_SPLIT_HALF_GAP)
    for name in shell_names:
        gap(name + ' to pod', name, 'stick_pod', sp.SHELL_GAP - 0.05)
        gap('battery to ' + name, 'parkside_battery', name, 1.0)
    gap('battery to pod (hook over lip)', 'parkside_battery', 'stick_pod', sp.LIP_GAP - 0.05)
    gap('battery to trunk', 'parkside_battery', 'trunk_base', 1.0)

    # Static overlaps of every changed part with every other occurrence. Face
    # contact is reported; only the trunk may touch the pod (its clamp face),
    # and the battery's gap is measured above.
    changed = ('stick_pod', 'parkside_battery', 'trunk_base', *shell_names)
    static = []; contacts = []
    for i, a in enumerate(changed):
        for b in shapes:
            if b == a or (b in changed and changed.index(b) < i) or bound_gap(boxes[a], boxes[b]) > 1e-7:
                continue
            result = pair_gap(parts[a], parts[b], cap=0.5)
            row = dict(a=a, b=b, gap_lower_bound_mm=result['gap_mm'], overlap_mm3=result['overlap_mm3'],
                       passed=result['overlap_mm3'] < tolerance)
            if a == 'stick_pod' and b not in ('trunk_base', 'parkside_battery') and result['gap_mm'] < 0.3:
                row['passed'] = False
            static.append(row)
            if result['gap_mm'] <= 1e-6:
                contacts.append(f'{a} / {b}')

    # Tongue clamp: the pod's top face meets the plate underside.
    t = sp.TONGUE
    def slab(z0, z1):
        return cq.Solid.makeBox(2 * t['x'], t['y1'] - t['y0'], z1 - z0, cq.Vector(-t['x'], t['y0'], z0))
    pod_face = shapes['stick_pod'].intersect(slab(sp.PLATE_BOTTOM_Z - 0.05, sp.PLATE_BOTTOM_Z))
    plate_face = shapes['trunk_base'].intersect(slab(sp.PLATE_BOTTOM_Z, sp.PLATE_BOTTOM_Z + 0.05))
    contact = volume(pod_face.translate(cq.Vector(0, 0, 0.05)).intersect(plate_face)) / 0.05
    clamp = dict(contact_area_mm2=contact, minimum_mm2=150.0, passed=contact >= 150.0)

    # Two M3 x 12 socket-head screws from above into heat-set inserts.
    screws = []
    def cylinder(x, y, r, z0, z1):
        return cq.Solid.makeCylinder(r, z1 - z0, cq.Vector(x, y, z0), cq.Vector(0, 0, 1))
    def ring(x, y, r0, r1, z0, z1):
        return cylinder(x, y, r1, z0, z1).cut(cylinder(x, y, r0, z0 - 1, z1 + 1))
    for x, y in sp.SCREWS:
        bore = cylinder(x, y, sp.SCREW_CLEARANCE_D / 2 - 0.05, sp.PLATE_BOTTOM_Z - 0.01, sp.PLATE_TOP_Z + 0.01)
        seat = ring(x, y, sp.SCREW_CLEARANCE_D / 2, 2.75, sp.PLATE_TOP_Z - 0.1, sp.PLATE_TOP_Z)
        pilot = cylinder(x, y, sp.INSERT_PILOT_D / 2 - 0.05, sp.PLATE_BOTTOM_Z - sp.INSERT_DEPTH, sp.PLATE_BOTTOM_Z - 0.01)
        wall = ring(x, y, sp.INSERT_PILOT_D / 2, sp.INSERT_PILOT_D / 2 + 1.5, sp.PLATE_BOTTOM_Z - sp.INSERT_DEPTH, sp.PLATE_BOTTOM_Z)
        shank = cylinder(x, y, 1.5, sp.PLATE_TOP_Z - 10.0, sp.PLATE_TOP_Z)
        driver = cylinder(x, y, 3.0, sp.PLATE_TOP_Z + 0.01, sp.PLATE_TOP_Z + 63.0)
        row = dict(x_mm=x, y_mm=y, nominal_screw='M3 x 12 socket head', nominal_insert='M3 x 5.7 heat-set',
                   plate_bore_overlap_mm3=volume(shapes['trunk_base'].intersect(bore)),
                   head_seat_fill=volume(shapes['trunk_base'].intersect(seat)) / volume(seat),
                   pilot_overlap_mm3=volume(shapes['stick_pod'].intersect(pilot)),
                   insert_wall_fill=volume(shapes['stick_pod'].intersect(wall)) / volume(wall), blocked_by={})
        for name, other in parts.items():
            for tool, label, allowed in ((shank, 'shank', ('trunk_base', 'stick_pod')), (driver, 'head and 3 x 60 driver', ())):
                if name in allowed or bound_gap(bounds(tool), boxes[name]) > 1e-7:
                    continue
                result = pair_gap(pieces(tool), other, cap=0.1)
                if result['overlap_mm3'] > tolerance:
                    row['blocked_by'][f'{label}: {name}'] = result['overlap_mm3']
        row['passed'] = (row['plate_bore_overlap_mm3'] < tolerance and row['pilot_overlap_mm3'] < tolerance and
                         row['head_seat_fill'] > 0.99 and row['insert_wall_fill'] > 0.99 and not row['blocked_by'])
        screws.append(row)

    # Review volumes for a later contact adapter stay free of every occurrence.
    allowances = []
    for key, solid in sp.adapter_allowances().items():
        hits = {}
        for name, other in parts.items():
            if bound_gap(bounds(solid), boxes[name]) > 1e-7:
                continue
            result = pair_gap(pieces(solid), other, cap=0.1)
            if result['overlap_mm3'] > tolerance:
                hits[name] = result['overlap_mm3']
        allowances.append(dict(volume=key, volume_mm3=volume(solid), extent_mm=bounds(solid), blocked_by=hits, passed=not hits))

    # Closed top: every vertical line above the stick meets the pod or a body shell.
    stick_top = sp.STICK_ORIGIN[2] + sp._stick().stick_top
    outline = sp.local_to_world(sp.prism('stick', -1.0, 0.0, 1.0))
    covers = [shapes[name] for name in ('stick_pod', *shell_names)]
    lines = 0; open_points = []
    ob = bounds(outline)
    x = ob[0]
    while x <= ob[3]:
        y = ob[1]
        while y <= ob[4]:
            if outline.isInside(cq.Vector(x, y, ob[2] + 0.5), 1e-6):
                lines += 1
                line = cq.Edge.makeLine(cq.Vector(x, y, stick_top + 0.1), cq.Vector(x, y, 200.0))
                if not any(cover.intersect(line).Edges() for cover in covers):
                    open_points.append([round(x, 2), round(y, 2)])
            y += 2.0
        x += 2.0
    closed = dict(vertical_lines=lines, grid_mm=2.0, open_at_xy_mm=open_points, passed=lines > 100 and not open_points)

    checks = [*gaps, *static, clamp, *screws, *allowances, closed]
    report = dict(document_sha256=scene.document_hash, units='mm', pose='saved zero pose', overlap_tolerance_mm3=tolerance,
                  prints=prints, battery_bounds_mm=battery_bounds, gaps=gaps, contacts=contacts, clamp=clamp,
                  screws=screws, adapter_allowances=allowances, closed_top=closed, static_pairs=static,
                  coverage='Valid single-solid trunk, pod and shells; seated battery bounds; shell split, shell-pod and '
                           'battery gaps; static overlaps of the changed parts with every occurrence (the pod may touch '
                           'only the trunk); tongue contact area; M3 bores, head seats, insert pilots and walls, shank and '
                           'driver paths; adapter and XT60 review volumes free; pod or shell over every 2 mm grid point of the '
                           'stick top. The stick, latch and contacts are APPROXIMATE. '
                           'No slicing, physical fit or strength test.',
                  passed=all(row['passed'] for row in checks))
    (ROOT / 'reports/latest/battery_pod_verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print({key: value for key, value in report.items() if key != 'static_pairs'})
    assert report['passed'], [row for row in checks if not row['passed']]
    return report
