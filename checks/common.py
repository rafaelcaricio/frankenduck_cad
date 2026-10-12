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
