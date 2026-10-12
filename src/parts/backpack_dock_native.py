from __future__ import annotations
from cadgen import step


def make():
    from lib.backpack import carrier
    return carrier()


@step(out="../../STEP/parts/backpack_dock_native.step")
def backpack_dock_native():
    return make()


if __name__ == "__main__":
    backpack_dock_native()
