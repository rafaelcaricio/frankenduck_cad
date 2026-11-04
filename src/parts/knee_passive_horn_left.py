from cadgen import step

def make():
    from lib.passive_leg_support import native_part
    return native_part('horn','left')

@step(out="../../STEP/parts/knee_passive_horn_left.step")
def knee_passive_horn_left():
    return make()

if __name__ == '__main__':
    knee_passive_horn_left()
