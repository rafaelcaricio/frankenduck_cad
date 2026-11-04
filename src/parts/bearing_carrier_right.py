from cadgen import step

def make():
    from lib.bearing_revision.factories import added
    return added('bearing_carrier_right')

@step(out="../../STEP/parts/bearing_carrier_right.step")
def bearing_carrier_right():
    return make()

if __name__ == "__main__":
    bearing_carrier_right()
