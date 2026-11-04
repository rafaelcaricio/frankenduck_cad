from cadgen import step

def make():
    from lib.bearing_revision.factories import added
    return added('bearing_roll_adapter_right')

@step(out="../../STEP/parts/bearing_roll_adapter_right.step")
def bearing_roll_adapter_right():
    return make()

if __name__ == "__main__":
    bearing_roll_adapter_right()
