from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    from cadgen import declare_input
    from lib.steve_compat import ROOT
    from lib.import_paths import IMPORTS
    declare_input(ROOT / IMPORTS['75f971949e6ba6f325cb7d5ecc90d49bc5bb5ac514bd25fa09f449cb9f956f95'])
    from lib.steve_compat import Component
    import_pcb_sph0645_mic = Component('import_pcb_sph0645_mic', name='pcb_sph0645_mic')
    import_pcb_sph0645_mic.import_step('import_1', '75f971949e6ba6f325cb7d5ecc90d49bc5bb5ac514bd25fa09f449cb9f956f95', name='pcb_sph0645_mic')
    model = import_pcb_sph0645_mic
    return to_build123d(model, 'import_pcb_sph0645_mic')


@step(out="../../STEP/parts/import_pcb_sph0645_mic.step")
def import_pcb_sph0645_mic():
    return make()


if __name__ == "__main__":
    import_pcb_sph0645_mic()
