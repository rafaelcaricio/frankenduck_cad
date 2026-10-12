from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    from cadgen import declare_input
    from lib.steve_compat import ROOT
    from lib.import_paths import IMPORTS
    declare_input(ROOT / IMPORTS['f4484a3777bee86a3f443ff41f07cd8f1de65fdcbb7d7d6b99c878c8dadae972'])
    import cadquery as cq
    from lib.steve_compat import Model
    model = Model()
    model.import_step('import_1', 'f4484a3777bee86a3f443ff41f07cd8f1de65fdcbb7d7d6b99c878c8dadae972', name='RV1103 Camera')
    result = model.result()
    return to_build123d(model, 'mighty_camera')


@step(out="../../STEP/parts/mighty_camera.step")
def mighty_camera():
    return make()


if __name__ == "__main__":
    mighty_camera()
