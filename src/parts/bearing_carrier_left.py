from cadgen import step

def make():
    from lib.passive_leg_support import native_part
    return native_part('carrier','left')

@step(out="../../STEP/parts/bearing_carrier_left.step")
def bearing_carrier_left():
    return make()

if __name__ == '__main__':
    bearing_carrier_left()
