from cadgen import step

def make():
    from lib.bearing_revision.factories import servo
    return servo()

@step(out="../../STEP/parts/bearing_servo_reference.step")
def bearing_servo_reference():
    return make()

if __name__ == "__main__":bearing_servo_reference()
