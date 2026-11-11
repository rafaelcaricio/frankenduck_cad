from cadgen import step
from lib.open_head import make_skeleton


def make():
    """Open development head skeleton; see docs/open-head.md. Part-local millimetres."""
    return make_skeleton()


@step(out="../../STEP/parts/open_head_skeleton_native.step")
def open_head_skeleton_native():
    return make()


if __name__ == "__main__":
    open_head_skeleton_native()
