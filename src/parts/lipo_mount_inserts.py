from cadgen import step


def make():
    """Four M3 x 5.7 heat-set inserts in the holder arms. World millimetres."""
    from lib.lipo_shoulders import fasteners
    return fasteners()[1]


@step(out="../../STEP/parts/lipo_mount_inserts.step")
def lipo_mount_inserts():
    return make()


if __name__ == "__main__":
    lipo_mount_inserts()
