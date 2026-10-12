from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    from cadgen import declare_input
    from lib.cadquery_adapter import ROOT
    from lib.import_paths import IMPORTS
    declare_input(ROOT / IMPORTS['7e066f91dd35a33edaada3b157e6115f49971bb7b695cb23e3b939346be328e4'])
    from lib.cadquery_adapter import Component
    import_horn_upper_passive = Component('import_horn_upper_passive', name='horn_upper_passive')
    import_horn_upper_passive.import_step('import_1', '7e066f91dd35a33edaada3b157e6115f49971bb7b695cb23e3b939346be328e4', name='horn_upper_passive')
    model = import_horn_upper_passive
    return to_build123d(model, 'import_horn_upper_passive')


@step(out="../../STEP/parts/import_horn_upper_passive.step")
def import_horn_upper_passive():
    return make()


if __name__ == "__main__":
    import_horn_upper_passive()
