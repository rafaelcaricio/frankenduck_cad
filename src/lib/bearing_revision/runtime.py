"""Pure geometry factories for the pinned CADGEN project runtime."""
from copy import deepcopy
from functools import lru_cache, wraps
from importlib import import_module
from pathlib import Path
from cadgen import build123d as bd, declare_input

def cached_shape(fn):
    cached = lru_cache(maxsize=None)(fn)
    @wraps(fn)
    def make(*args, **kwargs):
        return deepcopy(cached(*args, **kwargs))
    return make

@cached_shape
def read_step(filename):
    path = Path(filename)
    if path.parent.name == 'parts':
        path = path.parent.parent/'imported'/('bearing_baseline_'+path.name)
    declare_input(path)
    return bd.import_step(path)
