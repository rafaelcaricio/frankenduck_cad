from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    from cadgen import declare_input
    from lib.cadquery_adapter import ROOT
    from lib.import_paths import IMPORTS
    declare_input(ROOT / IMPORTS['b46da23dcb87978bd4709aab0ad88485cdaee24505fb8829c6c455e55f58b09c'])
    from lib.cadquery_adapter import Component
    import_pcb_bno055 = Component('import_pcb_bno055', name='pcb_bno055')
    import_pcb_bno055.import_step('import_1', 'b46da23dcb87978bd4709aab0ad88485cdaee24505fb8829c6c455e55f58b09c', name='pcb_bno055')
    model = import_pcb_bno055
    return to_build123d(model, 'import_pcb_bno055')


@step(out="../../STEP/parts/import_pcb_bno055.step")
def import_pcb_bno055():
    return make()


if __name__ == "__main__":
    import_pcb_bno055()
