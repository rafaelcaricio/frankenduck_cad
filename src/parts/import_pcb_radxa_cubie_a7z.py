from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    from cadgen import declare_input
    from lib.cadquery_adapter import ROOT
    from lib.import_paths import IMPORTS
    declare_input(ROOT / IMPORTS['85222c1f307d75029e6f3d41720afb3b33e8cce6ec0f3d30b847365029e71b9a'])
    from lib.cadquery_adapter import Component
    import_pcb_radxa_cubie_a7z = Component('import_pcb_radxa_cubie_a7z', name='pcb_radxa_cubie_a7z')
    import_pcb_radxa_cubie_a7z.import_step('import_1', '85222c1f307d75029e6f3d41720afb3b33e8cce6ec0f3d30b847365029e71b9a', name='pcb_radxa_cubie_a7z')
    model = import_pcb_radxa_cubie_a7z
    return to_build123d(model, 'import_pcb_radxa_cubie_a7z')


@step(out="../../STEP/parts/import_pcb_radxa_cubie_a7z.step")
def import_pcb_radxa_cubie_a7z():
    return make()


if __name__ == "__main__":
    import_pcb_radxa_cubie_a7z()
