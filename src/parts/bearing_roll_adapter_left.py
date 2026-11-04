from cadgen import step

def make():
    from lib.bearing_revision.factories import added
    return added('bearing_roll_adapter_left')

@step(out="../../STEP/parts/bearing_roll_adapter_left.step")
def bearing_roll_adapter_left():
    return make()

if __name__ == "__main__":
    bearing_roll_adapter_left()
