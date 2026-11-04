from cadgen import step

def make():
    from lib.bearing_revision.factories import added
    return added('bearing_6704')

@step(out="../../STEP/parts/bearing_6704.step")
def bearing_6704():
    return make()

if __name__ == "__main__":
    bearing_6704()
