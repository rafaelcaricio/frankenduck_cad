"""Small shared primitives; importing this module does not load the CAD kernel."""
from functools import lru_cache
import math
import time


@lru_cache(maxsize=8)
def read_scene(path):
    """One saved scene per path per runner invocation."""
    from cadgen import read_scene as load
    return load(path)


def bounds(shape):
    b = shape.BoundingBox()
    return [b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax]


def volume(shape):
    return sum(s.Volume() for s in shape.Solids())


def intersection(a_name, a, b_name, b, tolerance_mm3):
    start = time.monotonic()
    av, bv = bounds(a), bounds(b)
    gaps = [max(bv[i] - av[i+3], av[i] - bv[i+3], 0) for i in range(3)]
    row = dict(a=a_name, b=b_name)
    if max(gaps) > 1e-7:
        row.update(method="disjoint exact bounds", overlap_mm3=0.0,
                   separation_lower_bound_mm=math.sqrt(sum(g*g for g in gaps)))
    else:
        row.update(method="B-rep intersection", overlap_mm3=volume(a.intersect(b)))
    row.update(passed=row["overlap_mm3"] <= tolerance_mm3,
               seconds=time.monotonic()-start)
    return row


def bound_gap(a, b):
    """Lower bound on the distance between two shapes from their exact boxes."""
    return math.sqrt(sum(max(a[i] - b[i+3], b[i] - a[i+3], 0) ** 2 for i in range(3)))


def distance(a, b):
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    d = BRepExtrema_DistShapeShape()
    d.SetMultiThread(True)
    d.LoadS1(a.wrapped)
    d.LoadS2(b.wrapped)
    d.Perform()
    assert d.IsDone(), 'distance computation failed'
    return d.Value()


def pieces(shape):
    """Solids of a shape with their exact boxes."""
    return [(solid, bounds(solid)) for solid in shape.Solids()]


def pair_gap(a, b, cap=math.inf):
    """Exact minimum gap and overlap volume over solid pairs.

    Pairs whose boxes are at least the current minimum (or cap) apart are
    skipped, so a result equal to cap is a lower bound.
    """
    rows = sorted(((bound_gap(ba, bb), qa, qb) for qa, ba in a for qb, bb in b), key=lambda row: row[0])
    best, overlap, exact = cap, 0.0, 0
    for lower, qa, qb in rows:
        if lower >= best and lower > 1e-7:
            continue
        exact += 1
        measured = distance(qa, qb)
        best = min(best, measured)
        if measured <= 1e-6:
            overlap += volume(qa.intersect(qb))
    return dict(gap_mm=best, overlap_mm3=overlap, exact_solid_pairs=exact)


def clip_data(root):
    """Animation clips and per-occurrence joint operations from src/animation.js."""
    import json
    source = (root / 'src/animation.js').read_text()
    clips = json.loads(source.split('const DATA = ', 1)[1].split(';\nconst OPS = ', 1)[0])
    operations = json.loads(source.split('const OPS = ', 1)[1].split(';\n', 1)[0])
    return clips, operations


def joint_matrix(joints, pose):
    """4x4 transform of an occurrence: its joints applied in order, as in the viewer."""
    import numpy as np
    result = np.eye(4)
    for joint in joints:
        value = pose.get(joint['name'], 0)
        matrix = np.eye(4)
        axis = np.array(joint['axis'], dtype=float)
        if joint.get('kind') == 'slider':
            matrix[:3, 3] = axis * value
        else:
            x, y, z = axis
            angle = math.radians(value)
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            rotation = np.eye(3) * math.cos(angle) + (1 - math.cos(angle)) * np.outer(axis, axis) + math.sin(angle) * skew
            origin = np.array(joint['origin'])
            matrix[:3, :3] = rotation
            matrix[:3, 3] = origin - rotation @ origin
        result = matrix @ result
    return result


def posed(shape, joints, pose):
    """The shape moved by its joints at pose."""
    import cadquery as cq
    for joint in joints:
        value = pose.get(joint['name'], 0)
        if joint.get('kind') == 'slider':
            shape = shape.translate(cq.Vector(*[a * value for a in joint['axis']]))
        elif value:
            origin = joint['origin']
            shape = shape.rotate(origin, [o + a for o, a in zip(origin, joint['axis'])], value)
    return shape
