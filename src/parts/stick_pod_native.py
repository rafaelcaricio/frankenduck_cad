from __future__ import annotations
from cadgen import step


def make():
    from cadgen import build123d as bd
    from lib.stick_pod import pod, COLOR
    result = bd.Solid(pod().wrapped)
    result.label = 'stick_pod_native'
    result.color = bd.Color(COLOR)
    return result


@step(out="../../STEP/parts/stick_pod_native.step")
def stick_pod_native():
    return make()


if __name__ == "__main__":
    stick_pod_native()
