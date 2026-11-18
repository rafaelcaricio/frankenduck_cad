from cadgen import step
from lib.lipo_shoulders import housing

def make():
    return housing()

@step(out="../../STEP/parts/lipo_pod_native.step")
def lipo_pod_native():
    return make()

if __name__ == "__main__":
    lipo_pod_native()
