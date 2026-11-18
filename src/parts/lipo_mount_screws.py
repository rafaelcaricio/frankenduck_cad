from cadgen import step


def make():
    """Four M3 x 10 socket-head screws clamping the trunk plate onto the holder arms. World millimetres."""
    from lib.lipo_shoulders import fasteners
    return fasteners()[0]


@step(out="../../STEP/parts/lipo_mount_screws.step")
def lipo_mount_screws():
    return make()


if __name__ == "__main__":
    lipo_mount_screws()
