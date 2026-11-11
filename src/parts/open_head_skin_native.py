from cadgen import step
from lib.open_head import make_skin


def make():
    """Open development head skin; see docs/open-head.md. Part-local millimetres."""
    return make_skin()


@step(out="../../STEP/parts/open_head_skin_native.step")
def open_head_skin_native():
    return make()


if __name__ == "__main__":
    open_head_skin_native()
