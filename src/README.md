# Model catalog

| Entry | Output | Purpose |
| --- | --- | --- |
| `frankenduck.py` | `STEP/frankenduck.step` and sidecar | 73 original occurrences plus the cooler, mates, poses, animation |
| `radxa_fit_review.py` | `STEP/radxa_fit_review.step` | Saved mounting context with explicit plug and cable allowance volumes |
| `parts/*.py` | `STEP/parts/<same stem>.step` | 59 original local component variants plus the cooler; each file has one decorated model |
| `parts/parkside_pap_204_a1.py` | `STEP/parts/parkside_pap_204_a1.step` | Standalone PARKSIDE battery port; one selectable component with retained per-face STEP colours, outside the robot assembly |
| `lib/steve_compat.py` | — | Local CadQuery feature adapter and conversion to build123d |
| `lib/import_paths.py` | — | Frozen imported-geometry identifiers mapped to local STEP inputs |
| `lib/radxa_clearance.py` | — | Shared millimetre bounds for individual jumper housings and their upward cable path |
| `lib/radxa_6530b_parameters.py` | — | Official cooler outer size, explicit detailed-geometry assumptions and fan-inlet space |
| `animation.js` | Embedded by `frankenduck.py` | Original timeline translated to cadgen world-space operations |

Build the root with `../cad build` from this directory or `./cad build` from the
project root. Run `./cad check fast` during editing and `./cad check full`
before accepting a revision. The project README owns the check workflow.
To rebuild a single model from the project root:

```sh
CADGEN_CACHE_DIR="$PWD/.cadgen" PYTHONPATH=src .venv/bin/python src/parts/trunk_base_native.py
```

The source snapshot and parameter-to-variant map are under `../source/`.
The active bracket, speaker mount and upper-cover interior contain local edits documented in the
project README; their original factories are preserved in `../revisions/r91_import/`.
`tools/migration/prepare_port.py` regenerates the transformed source from that snapshot;
running it intentionally replaces local source edits, so use it only to recreate
the original port. It needs the embedded-input export ZIP in `scratch/` only to
re-extract the already-retained inputs; normal model builds use local STEP files.

Component roles and print intent are owned by `../source/components.json`;
`../cad parts` lists them and `../cad export print` creates the tracked 3MF bundle.

The standalone PARKSIDE battery and its source provenance are documented in the
project README. Build it with the same single-model command above, substituting
`parkside_pap_204_a1.py`. Its `make()` factory is available for later assembly use.
