# CAD tools

Use `./cad` from the repository root for ordinary work.

| File | Responsibility |
| --- | --- |
| `cad.py` | Build/check orchestration, freshness, status and review commands |
| `components.py` | Component classification, active variants and quantities |
| `print_export.py` | Validated, atomic 3MF print bundle, mesh reuse and parallel workers |
| `print_mesh.py` | Per-part meshing worker and its mesh, face and STL checks |
| `prepare_viewer.py` | Project-local viewer setup with component face colors |
| `viewer/face_colours.mjs` | Viewer face-color support |
| `tests/` | Workflow, classification, mesh and viewer regressions |

Checks are registered in `checks/registry.json`. Their assertions belong in
`checks/assembly.py` or `checks/radxa.py`; shared geometry operations belong in
`checks/common.py`. Each registry entry declares its purpose, dependencies,
suites, output and coverage. `fast` checks metadata, `radxa` adds local fit and
`full` runs all maintained checks.

The runner caches passing results by input contents and runtime versions.
Missing inputs, changed outputs and failed or interrupted work stay visible
in `./cad status`. Builds and expensive exports remain explicit.

Print export runs native meshing in bounded worker processes, checks the saved
3MF files against the current CAD geometry and records settings and warnings
in the manifest. A complete bundle replaces `printable/` only after validation.
Up to eight workers run at once, heaviest parts first. Each part tries the mesh
recipe that passed for it last time before the rest of the ladder, with a 600 s
limit instead of 180 s so that load from parallel workers cannot change which
recipe a part ends up with. `PART_RECIPES` in `print_export.py` gives the soles
their own first recipe (Watson 0.01 mm at 0.35 rad); their lofted walls have
over 800 spline knots, and the general ladder meshed them at 2 M triangles.

Each manifest part records the hash of its normalized saved STEP geometry
(`geometry_sha256`, taken before any rebuild of an invalid saved solid) and of everything else that can change its mesh
(`mesh_key`: the `print_mesh.py` and `gmsh_mesh.py` sources, runtime versions,
tolerances and recipes). When both match the previous intact bundle, the part's
verified 3MF and STL are copied unchanged and checked by hash. Any other change
meshes and checks that part again. Changes to `print_export.py`, the README
text or the license files do not re-mesh parts.
The viewer uses an ignored local client under `.cadgen/` for face-color support.

Run the relevant regressions after tool changes:

```sh
.venv/bin/python -m unittest discover -s tools/tests -v
node --test tools/tests/test_viewer_face_colours.mjs
```

Keep one-off diagnostics in ignored `scratch/`. Promote useful assertions into
an existing maintained check instead of accumulating standalone scripts.
