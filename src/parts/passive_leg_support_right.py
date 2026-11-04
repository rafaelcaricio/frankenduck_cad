from cadgen import step

def make():
    from lib.passive_leg_support import native_part
    return native_part('support','right')

@step(out="../../STEP/parts/passive_leg_support_right.step")
def passive_leg_support_right():
    return make()

if __name__ == '__main__':
    passive_leg_support_right()
