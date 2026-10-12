from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    from cadgen import declare_input
    from lib.cadquery_adapter import ROOT
    from lib.import_paths import IMPORTS
    declare_input(ROOT / IMPORTS['f0c6e7123818eee09d2da1c533a9ce122367e766d431a84f8bb91a18fa7d0517'])
    from lib.cadquery_adapter import Component
    import_pcb_waveshare_adapter = Component('import_pcb_waveshare_adapter', name='pcb_waveshare_adapter')
    import_pcb_waveshare_adapter.import_step('import_1', 'f0c6e7123818eee09d2da1c533a9ce122367e766d431a84f8bb91a18fa7d0517', name='pcb_waveshare_adapter')
    model = import_pcb_waveshare_adapter
    return to_build123d(model, 'import_pcb_waveshare_adapter')


@step(out="../../STEP/parts/import_pcb_waveshare_adapter.step")
def import_pcb_waveshare_adapter():
    return make()


if __name__ == "__main__":
    import_pcb_waveshare_adapter()
