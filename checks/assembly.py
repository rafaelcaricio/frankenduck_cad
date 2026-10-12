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
        if mate['kind'] == 'revolute':
            assert mate['axis']['origin'] == declared['origin'], name
            assert mate['axis']['dir'] == declared['direction'], name
            assert abs(math.sqrt(sum(v*v for v in declared['direction'])) - 1) < 1e-6, name
            assert mate['limits']['value'] == declared['limits'], name
    assert side['kinematics']['poses'] == data['poses'], 'Saved pose presets differ'

    source = (ROOT / 'src/animation.js').read_text()
    clips = json.loads(source.split('const DATA = ', 1)[1].split(';\nconst OPS = ', 1)[0])
    operations = json.loads(source.split('const OPS = ', 1)[1].split(';\n', 1)[0])
    assert side['animation']['source'] == source, 'Saved animation differs'
    joint_names = {name for name, mate in expected.items() if mate['kind'] == 'revolute'}
    operation_refs = [operation['target'] for operation in operations]
    assert len(operation_refs) == len(set(operation_refs)), 'Duplicate animation target'
    assert set(operation_refs) == {'#' + ref for ref in refs.values()}, 'Animation target inventory differs'
    for operation in operations:
        for joint in operation['joints']:
            declared = expected[joint['name']]
            assert joint['axis'] == declared['direction'], joint['name']
            assert joint['origin'] == declared['origin'], joint['name']
    for name, clip in clips.items():
        keys = clip['keys']
        assert keys and keys[0][1] == 0, name
        assert clip['duration'] == keys[-1][1], name
        assert all(a[1] < b[1] for a, b in zip(keys, keys[1:])), name
        for pose, timestamp in keys:
            assert pose.keys() <= joint_names, name
            assert math.isfinite(timestamp) and all(math.isfinite(v) for v in pose.values()), name
    board = next(operation for operation in operations if operation['target'] == '#o1.15')
    cooler = next(operation for operation in operations if operation['target'] == '#o1.74')
    assert board['joints'] == cooler['joints'], 'Cooler must follow the board'

    report = dict(passed=True, mates=len(actual), revolute=len(joint_names),
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
