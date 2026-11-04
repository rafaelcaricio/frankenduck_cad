from cadgen import step

def make():
    from lib.bearing_revision.factories import added
    return added('bearing_ankle_adapter')

@step(out="../../STEP/parts/bearing_ankle_adapter.step")
def bearing_ankle_adapter():
    return make()

if __name__ == "__main__":
    bearing_ankle_adapter()
