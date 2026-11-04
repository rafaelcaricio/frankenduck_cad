from cadgen import step

def make():
    from lib.passive_leg_support import hardware_native
    return hardware_native('left','horn')

@step(out="../../STEP/parts/passive_horn_screw_heads_left.step")
def passive_horn_screw_heads_left():
    return make()

if __name__ == '__main__':
    passive_horn_screw_heads_left()
