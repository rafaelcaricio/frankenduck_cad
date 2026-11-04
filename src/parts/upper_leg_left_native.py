from cadgen import step

def make():
    from lib.passive_leg_support import native_part
    return native_part('shell','left')

@step(out="../../STEP/parts/upper_leg_left_native.step")
def upper_leg_left_native():
    return make()

if __name__ == '__main__':
    upper_leg_left_native()
