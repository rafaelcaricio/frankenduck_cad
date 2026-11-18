from cadgen import step
from lib.lipo_compact import battery

def make():
    return battery()

@step(out="../../STEP/parts/zeee_2s_2200_reference.step")
def zeee_2s_2200_reference():
    return make()

if __name__ == "__main__":
    zeee_2s_2200_reference()
