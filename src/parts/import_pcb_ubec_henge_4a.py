from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    from cadgen import declare_input
    from lib.steve_compat import ROOT
    from lib.import_paths import IMPORTS
    declare_input(ROOT / IMPORTS['fa170ef193bf9d1cf7c17a2f571d3f9301bf10e4c5d694e7b63ce0fe2bf9731e'])
    from lib.steve_compat import Component
    import_pcb_ubec_henge_4a = Component('import_pcb_ubec_henge_4a', name='pcb_ubec_henge_4a')
    import_pcb_ubec_henge_4a.import_step('import_1', 'fa170ef193bf9d1cf7c17a2f571d3f9301bf10e4c5d694e7b63ce0fe2bf9731e', name='pcb_ubec_henge_4a')
    model = import_pcb_ubec_henge_4a
    return to_build123d(model, 'import_pcb_ubec_henge_4a')


@step(out="../../STEP/parts/import_pcb_ubec_henge_4a.step")
def import_pcb_ubec_henge_4a():
    return make()


if __name__ == "__main__":
    import_pcb_ubec_henge_4a()
