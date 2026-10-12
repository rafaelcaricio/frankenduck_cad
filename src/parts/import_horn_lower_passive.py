from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    from cadgen import declare_input
    from lib.steve_compat import ROOT
    from lib.import_paths import IMPORTS
    declare_input(ROOT / IMPORTS['505cee384bf73098c7b4af001ab8c7f7922ed29fbf5564fe68b8ce842551d0f5'])
    from lib.steve_compat import Component
    import_horn_lower_passive = Component('import_horn_lower_passive', name='horn_lower_passive')
    import_horn_lower_passive.import_step('import_1', '505cee384bf73098c7b4af001ab8c7f7922ed29fbf5564fe68b8ce842551d0f5', name='horn_lower_passive')
    model = import_horn_lower_passive
    return to_build123d(model, 'import_horn_lower_passive')


@step(out="../../STEP/parts/import_horn_lower_passive.step")
def import_horn_lower_passive():
    return make()


if __name__ == "__main__":
    import_horn_lower_passive()
