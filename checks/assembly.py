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
    cooler = next(operation for operation in operations if operation['target'] == '#' + refs['radxa_6530b'])
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


def backpack():
    """Approved backpack datums, static clearance and 43 sampled leg poses.

    This is bounded CAD evidence, not a continuous sweep or physical release.
    """
    import bisect
    import itertools
    import numpy as np
    import cadquery as cq
    from checks.common import read_scene, bounds, intersection, volume
    from lib.backpack import thumb_approach, dock, incline, BATTERY_BOTTOM_Z, JOINT_X, JOINT_Z, HEAD_SEAT_Y
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape

    raw_intersection = intersection
    solid_cache = {}
    def solid_pieces(shape):
        key=id(shape)
        if key not in solid_cache:
            solid_cache[key]=(shape,[(q,bounds(q)) for q in shape.Solids()])
        return solid_cache[key][1]
    def intersection(a_name, a, b_name, b, tolerance_mm3):
        # Test actual solid pairs, avoiding a costly compound-wide distance
        # across all decorative battery bodies and spline shell faces.
        overlap=0.0; exact=0
        for qa,av in solid_pieces(a):
            for qb,bv in solid_pieces(b):
                if any(av[i]>bv[i+3]+1e-7 or bv[i]>av[i+3]+1e-7 for i in range(3)):
                    continue
                exact+=1
                distance=BRepExtrema_DistShapeShape(qa.wrapped,qb.wrapped)
                assert distance.IsDone(),(a_name,b_name)
                if distance.Value()<=1e-6:
                    overlap+=volume(qa.intersect(qb))
        return dict(a=a_name,b=b_name,method='solid pairs with conservative bounds',
                    exact_solid_pairs=exact,overlap_mm3=overlap,passed=overlap<tolerance_mm3)

    tolerance = 1e-5  # mm3; coincident-face numerical tolerance
    scene = read_scene(ROOT / 'STEP/frankenduck.step')
    occurrences = {part.ref: part for part in scene.roots[0].children}
    shapes = {part.label: cq.Shape.cast(part.shape().wrapped) for part in occurrences.values()}
    cached_bounds = {name: bounds(shape) for name,shape in shapes.items()}
    pack = shapes['parkside_battery']
    trunk = shapes['trunk_base']
    carrier = shapes['backpack_dock']
    shell_names = ('left_shell_native', 'right_shell_native')
    prints = {}
    for name in ('trunk_base', 'backpack_dock', *shell_names):
        shape = shapes[name]
        prints[name] = dict(valid=shape.isValid(), solids=len(shape.Solids()), volume_mm3=volume(shape))
        assert shape.isValid() and len(shape.Solids()) == 1, name
    gaps = []
    def gap(name, first, second, minimum):
        # Exact minimum over solid pairs; bounding separation only skips pairs
        # that provably cannot improve the current minimum.
        pairs=[]
        for qa,a in solid_pieces(first):
            for qb,b in solid_pieces(second):
                lower=math.sqrt(sum(max(a[i]-b[i+3],b[i]-a[i+3],0)**2 for i in range(3)))
                pairs.append((lower,qa,qb))
        measured=float('inf')
        for lower,qa,qb in sorted(pairs,key=lambda row:row[0]):
            if lower>=measured:continue
            distance=BRepExtrema_DistShapeShape(qa.wrapped,qb.wrapped)
            assert distance.IsDone(),name
            measured=min(measured,distance.Value())
        assert math.isfinite(measured),name
        row = dict(interface=name, minimum_mm=minimum, measured_mm=measured,
                   passed=measured >= minimum - 1e-5)
        gaps.append(row)
        assert row['passed'], row
    gap('shell split including rear patch', shapes[shell_names[0]], shapes[shell_names[1]], .6)
    mount = cq.Shape.cast(dock().wrapped)
    for name in shell_names:
        gap(name + ' to dock', shapes[name], mount, .8)
    gap('battery to trunk frame', pack, trunk, .5)
    gap('battery to separate dock', pack, carrier, .5)
    for name in shell_names:
        gap('battery to ' + name, pack, shapes[name], .8)
    # Necessary openings: actual fastener paths and nominal nut envelopes.
    joints = []
    for x in JOINT_X:
        for z in JOINT_Z:
            bore = cq.Solid.makeCylinder(1.65,17,cq.Vector(x,50,z),cq.Vector(0,1,0))
            driver = cq.Solid.makeCylinder(3.1,76.2,cq.Vector(x,HEAD_SEAT_Y,z),cq.Vector(0,1,0))
            radius = 5.5/math.sqrt(3)
            points = [cq.Vector(x+radius*math.cos(i*math.pi/3),53.2,z+radius*math.sin(i*math.pi/3)) for i in range(6)]
            nut = cq.Solid.extrudeLinear(cq.Face.makeFromWires(cq.Wire.makePolygon(points,close=True)),cq.Vector(0,2.4,0))
            # Four independent load-bearing patches meet directly at Y=60.
            front = cq.Solid.makeBox(7,.1,7,cq.Vector(x-3.5,59.9,z-3.5)).cut(bore)
            rear = cq.Solid.makeBox(7,.1,7,cq.Vector(x-3.5,60,z-3.5)).cut(bore)
            bore,driver,nut,front,rear = [incline(q) for q in (bore,driver,nut,front,rear)]
            row = dict(x_mm=x,z_mm=z,frame_bore_overlap_mm3=trunk.intersect(bore).Volume(),
                       dock_bore_overlap_mm3=carrier.intersect(bore).Volume(),nut_overlap_mm3=trunk.intersect(nut).Volume(),
                       frame_bearing_patch_mm3=trunk.intersect(front).Volume(),dock_bearing_patch_mm3=carrier.intersect(rear).Volume(),
                       nominal_screw='M3 x 12 socket head',nominal_nut='M3 hex, 5.5 mm AF, 2.4 mm thick')
            assert max(row[k] for k in ('frame_bore_overlap_mm3','dock_bore_overlap_mm3','nut_overlap_mm3')) < tolerance,row
            assert min(row['frame_bearing_patch_mm3'],row['dock_bearing_patch_mm3'])>3.9,row
            for other,shape in shapes.items():
                if other in ('parkside_battery',): continue
                result=intersection('screwdriver_access_battery_removed',driver,other,shape,tolerance)
                assert result['passed'],result
            joints.append(row)
    root_sections=[]
    for x in JOINT_X:
        line=cq.Edge.makeLine(cq.Vector(x,28,145),cq.Vector(x,28,166))
        lengths=[edge.Length() for edge in trunk.intersect(line).Edges()]
        assert len(lengths)==1 and lengths[0]>=17.9,(x,lengths)
        root_sections.append(dict(x_mm=x,y_mm=28,continuous_depth_mm=lengths[0]))
    gap('clamped frame/dock faces',trunk,carrier,0)
    assert BRepExtrema_DistShapeShape(trunk.wrapped,carrier.wrapped).Value()<1e-6
    sections=[]
    for name,sign in zip(shell_names,(1,-1)):
        for y,z in ((20,145),(30,160),(36,160)):
            line=cq.Edge.makeLine(cq.Vector(sign*35,y,z),cq.Vector(sign*46,y,z))
            lengths=[edge.Length() for edge in shapes[name].intersect(line).Edges()]
            row=dict(part=name,y_mm=y,z_mm=z,wall_chords_mm=lengths,minimum_mm=2.9)
            sections.append(row)
            assert len(lengths)==1 and lengths[0]>=2.9,row
        # The superseded central corner tabs are intentionally absent.
        opening=cq.Solid.makeBox(56,14,12,cq.Vector(-28,48,126))
        assert volume(shapes[name].intersect(opening))<tolerance,name
        # User-marked upper return is fully open, and the original roof height
        # is retained instead of raising a crown to cover the mount.
        upper_opening=cq.Solid.makeBox(48.8,60,80,cq.Vector(-24.4,28,164))
        assert volume(shapes[name].intersect(upper_opening))<tolerance,name
        assert bounds(shapes[name])[5]<=175.0,name
    print(dict(assembly_gaps=gaps, shell_wall_sections=sections), flush=True)
    pack_bounds = bounds(pack)
    expected_bounds = [-41.5, 39.116254574, BATTERY_BOTTOM_Z, 41.5, 137.919003710, 218.791029088]
    assert max(abs(a-b) for a, b in zip(pack_bounds, expected_bounds)) < 1e-4, pack_bounds
    assert not {'npf550_pack_native', 'battery_carrier_native', 'battery_donor_cover_native'} & shapes.keys()
    battery_occurrence = next(p for p in occurrences.values() if p.label == 'parkside_battery')
    leaves = []
    def collect(part):
        if part.children:
            for child in part.children:
                collect(child)
        else:
            leaves.append(part)
    collect(battery_occurrence)
    assert len(leaves) == 1, 'Battery must remain one selectable colored component'
    static = []
    for name, shape in shapes.items():
        if name != 'parkside_battery':
            static.append(intersection('battery', pack, name, shape, tolerance))
        if name not in ('trunk_base', 'parkside_battery'):
            static.append(intersection('trunk_frame', trunk, name, shape, tolerance))
        if name not in ('backpack_dock', 'parkside_battery'):
            static.append(intersection('separate_dock', carrier, name, shape, tolerance))
        if name != 'parkside_battery':
            static.append(intersection('thumb_approach', thumb_approach(), name, shape, tolerance))
    static.append(intersection('thumb_approach', thumb_approach(), 'trunk_base', trunk, tolerance))
    # The retained rear shell rims must also clear every saved hardware occurrence.
    corners = []
    for name, x0 in zip(shell_names, (20, -38)):
        clip = cq.Solid.makeBox(18, 14, 20, cq.Vector(x0, 47, 126.5))
        corner = shapes[name].intersect(clip)
        corners.append((name + '_rear_rim', corner))
        for other, shape in shapes.items():
            if other != name:
                static.append(intersection(name + '_rear_rim', corner, other, shape, tolerance))


    source = (ROOT / 'src/animation.js').read_text()
    clips = json.loads(source.split('const DATA = ', 1)[1].split(';\nconst OPS = ', 1)[0])
    operations = json.loads(source.split('const OPS = ', 1)[1].split(';\n', 1)[0])
    clip_name = 'walk_sit_stand_walk_sit'
    clip = clips[clip_name]
    times = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17.7,20.08,21.12,
             23.08,24.08,25.52,28.1,29.1,30.1,31.1,32.1,33.1,34.1,35.1,36.1,
             37.1,38.1,39.1,40.1,41.1,42.1,43.1,44.8,48.08,49.12,58.12]
    assert max(times) <= clip['duration'], 'Sample timeline no longer matches the clip'
    legs = [op for op in operations if any(j['name'].startswith(('left_', 'right_')) for j in op['joints'])]
    assert len(legs) == 30, 'Leg occurrence coverage changed'
    def pose_at(t):
        keys = clip['keys']; kt = [k[1] for k in keys]
        i = max(0, min(len(keys)-2, bisect.bisect_right(kt, t)-1))
        a, b = keys[i:i+2]
        u = max(0, min(1, (t-a[1])/(b[1]-a[1])))
        if clip.get('easing') == 'ease':
            u = u*u*(3-2*u)
        return {k: v + u*(b[0].get(k, v)-v) for k, v in a[0].items()}
    def transform(joints, pose):
        result = np.eye(4)
        for joint in joints:
            axis = np.array(joint['axis'], dtype=float)
            angle = math.radians(pose.get(joint['name'], 0))
            x,y,z = axis
            skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            rotation = np.eye(3)*math.cos(angle) + (1-math.cos(angle))*np.outer(axis,axis) + math.sin(angle)*skew
            origin = np.array(joint['origin'])
            matrix = np.eye(4); matrix[:3,:3] = rotation; matrix[:3,3] = origin-rotation@origin
            result = matrix @ result
        return result
    targets = [('battery', pack), ('trunk_frame', trunk), ('separate_dock',carrier), *[(n,shapes[n]) for n in shell_names]]
    target_bounds = {name: bounds(shape) for name,shape in targets}
    motion = []; rejected = 0
    corner_motion_gaps = []
    for t in times:
        pose = pose_at(t)
        for op in legs:
            part = occurrences[op['target']]
            shape = shapes[part.label]; box = cached_bounds[part.label]
            corners = np.array([(*p,1) for p in itertools.product(*[(box[i],box[i+3]) for i in range(3)])])
            moved = (transform(op['joints'], pose) @ corners.T).T[:,:3]
            lo,hi = moved.min(axis=0),moved.max(axis=0)
            for target, fixed in targets:
                fixed_box = target_bounds[target]
                corner_target = target in shell_names
                bound_gap = math.sqrt(sum(max(lo[i]-fixed_box[i+3], fixed_box[i]-hi[i], 0)**2 for i in range(3)))
                if corner_target and bound_gap >= 1.0:
                    corner_motion_gaps.append(dict(target=target, ref=part.ref, time_s=t,
                        lower_bound_mm=float(bound_gap), method='transformed bounding box', passed=True))
                if (not corner_target or bound_gap >= 1.0) and any(lo[i] > fixed_box[i+3]+1e-7 or hi[i] < fixed_box[i]-1e-7 for i in range(3)):
                    rejected += 1
                    continue
                candidate = shape
                for joint in op['joints']:
                    origin = joint['origin']; end = [a+b for a,b in zip(origin,joint['axis'])]
                    candidate = candidate.rotate(origin, end, pose.get(joint['name'], 0))
                if corner_target:
                    measured = BRepExtrema_DistShapeShape(fixed.wrapped, candidate.wrapped).Value()
                    corner_motion_gaps.append(dict(target=target, ref=part.ref, time_s=t,
                        lower_bound_mm=measured, method='exact solid distance', passed=measured >= 1.0-1e-5))
                row = intersection(target, fixed, part.label, candidate, tolerance)
                row.update(time_s=t, ref=part.ref)
                motion.append(row)
    report = dict(document_sha256=scene.document_hash, units='mm', overlap_tolerance_mm3=tolerance,
                  prints=prints, assembly_gaps=gaps, screwed_joints=joints, root_sections=root_sections, shell_wall_sections=sections, battery_bounds_mm=pack_bounds, battery_selectable_leaves=len(leaves),
                  static_pairs=static, clip=clip_name, sample_times_s=times, leg_occurrences=len(legs),
                  sampled_pair_checks=len(times)*len(legs)*len(targets), bounds_rejected=rejected,
                  motion_candidates=motion, corner_motion_clearances=corner_motion_gaps,
                  coverage='Saved rest and 43 discrete leg poses against battery, trunk frame, separate dock and complete shells. Measured dock, split seam and seated battery gaps; M3 passages, nut fit, bearing patches and battery-removed driver access. No continuous sweep, physical rail/latch verification or structural load test.',
                  passed=all(row['passed'] for row in static+motion+corner_motion_gaps))
    (ROOT / 'reports/latest/backpack_verification.json').write_text(json.dumps(report, indent=2)+'\n')
    print({key: value for key,value in report.items() if key not in ('static_pairs','motion_candidates','corner_motion_clearances')})
    assert report['passed'], [r for r in static+motion+corner_motion_gaps if not r['passed']]
    return report
