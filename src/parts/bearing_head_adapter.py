from cadgen import step

def make():
    from lib.head_bearings import base_adapter
    return base_adapter()

@step(out="../../STEP/parts/bearing_head_adapter.step")
def bearing_head_adapter():
    return make()

if __name__ == "__main__":
    bearing_head_adapter()
