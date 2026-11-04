from cadgen import step

def make():
    from lib.bearing_revision.factories import added
    return added('bearing_screws_left')

@step(out="../../STEP/parts/bearing_screws_left.step")
def bearing_screws_left():
    return make()

if __name__ == "__main__":
    bearing_screws_left()
