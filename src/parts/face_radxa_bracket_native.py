from cadgen import step
from lib.radxa_gpio_mount import make as support

def make():
    """Single open carrier; contact at four corner screw lands only."""
    return support()

@step(out='../../STEP/parts/face_radxa_bracket_native.step')
def face_radxa_bracket_native():return make()

if __name__=='__main__':face_radxa_bracket_native()
