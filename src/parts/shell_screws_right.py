from cadgen import step

def make():
    from lib.passive_leg_support import hardware_native
    return hardware_native('right','shell')

@step(out="../../STEP/parts/shell_screws_right.step")
def shell_screws_right():
    return make()

if __name__ == '__main__':
    shell_screws_right()
