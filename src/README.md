# Model catalog

| Entry | Purpose |
| --- | --- |
| `frankenduck.py` | Complete assembly: 104 occurrences, joints, poses and animation |
| `radxa_fit_review.py` | Mounting context with GPIO and cooler clearance volumes |
| `parts/*.py` | Individual component models and parameter variants |
| `parts/lipo_pod_native.py` | Printed holder for the Zeee 2S LiPo pack |
| `parts/zeee_2s_2200_reference.py` | Zeee 2S 2200 mAh pack at its published size; reference |
| `parts/lipo_pad_and_leads.py`, `parts/lipo_mount_screws.py`, `parts/lipo_mount_inserts.py` | Pack pad and lead envelopes, and the holder's M3 screws and heat-set inserts |
| `lib/lipo_*.py` | Holder construction in stages, trunk and shell changes, fasteners; `lipo_shoulders.py` gives the final parts |
| `lib/body_profile.py`, `lib/body_shell_fasteners.py`, `lib/body_shell_finish.py` | Shared body silhouette, internal shell ties and the shells' rear tip round |
| `parts/parkside_pap_204_a1.py` | PAP 204 A1 backpack battery with per-face colors; not in the assembly |
| `parts/parkside_papk_12_a4.py`, `parts/stick_pod_native.py` | Retired PAPK 12 A4 stick battery and its pod; not in the assembly |
| `lib/stick_pod.py` | Stick pod geometry; the LiPo holder reuses its fairing, socket and shell-relief construction |
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
