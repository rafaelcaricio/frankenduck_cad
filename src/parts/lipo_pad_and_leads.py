from cadgen import step


def make():
    """Foam pad under the Zeee 2S pack, its power and balance leads and plugs. World millimetres; visual envelopes."""
    from lib.lipo_shoulders import reference_hardware
    return reference_hardware()


@step(out="../../STEP/parts/lipo_pad_and_leads.step")
def lipo_pad_and_leads():
    return make()


if __name__ == "__main__":
    lipo_pad_and_leads()
