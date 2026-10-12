# CAD tools

Use `./cad` from the repository root for ordinary work.

| File | Responsibility |
| --- | --- |
| `cad.py` | Build/check orchestration, freshness, status and review commands |
| `components.py` | Component classification, active variants and quantities |
| `print_export.py` | Validated, atomic 3MF print bundle |
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
The viewer uses an ignored local client under `.cadgen/` for face-color support.

Run the relevant regressions after tool changes:

```sh
.venv/bin/python -m unittest discover -s tools/tests -v
node --test tools/tests/test_viewer_face_colours.mjs
```

Keep one-off diagnostics in ignored `scratch/`. Promote useful assertions into
an existing maintained check instead of accumulating standalone scripts.
