from cadgen import step

def make():
    from lib.bearing_revision.factories import added
    return added('bearing_shared_yaw_adapter')

@step(out="../../STEP/parts/bearing_shared_yaw_adapter.step")
def bearing_shared_yaw_adapter():
    return make()

if __name__ == "__main__":
    bearing_shared_yaw_adapter()
