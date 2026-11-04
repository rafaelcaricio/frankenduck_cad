from cadgen import step

def make():
    from lib.neck_pitch import native
    from lib.viewer_palette import paint_original
    return paint_original(native(),"neck_pitch_001_native")

@step(out="../../STEP/parts/neck_pitch_001_native.step")
def neck_pitch_001_native():
    return make()

if __name__ == "__main__":
    neck_pitch_001_native()
