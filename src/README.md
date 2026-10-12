# Model catalog

| Entry | Purpose |
| --- | --- |
| `frankenduck.py` | Complete assembly: 73 occurrences, joints, poses and animation |
| `radxa_fit_review.py` | Mounting context with GPIO and cooler clearance volumes |
| `parts/*.py` | Individual component models and parameter variants |
| `parts/parkside_pap_204_a1.py` | Exposed backpack battery with per-face colors |
| `lib/backpack.py` | Two-part mount, shell reliefs, battery datum and access allowances |
| `lib/cadquery_adapter.py` | Shared feature construction and build123d conversion |
| `lib/import_paths.py` | Local purchased-component STEP inputs |
| `lib/radxa_clearance.py` | Jumper-plug and upward cable space |
| `lib/radxa_6530b_parameters.py` | Cooler dimensions, detailed assumptions and inlet space |
| `animation.js` | Joint timeline and world-space animation operations |

Build the complete assembly from the repository root with `./cad build`.
Each part has a `make()` geometry factory and a decorated entrypoint that
writes `STEP/parts/<model>.step`. Build a component independently with:

```sh
CADGEN_CACHE_DIR="$PWD/.cadgen" PYTHONPATH=src .venv/bin/python src/parts/trunk_base_native.py
```

`source/assembly_data.json` declares occurrences, mates and poses.
`source/variants.json` selects the assembly's component configurations.
`source/components.json` defines roles, print intent and material hints.
`./cad parts` lists quantities and `./cad export print` creates the printable
bundle. Rebuild the assembly after a component edit.

See [the project README](../README.md) for the workflow and
[design notes](../docs/design.md) for interface dimensions and assumptions.
