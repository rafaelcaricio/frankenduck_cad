# Working on Frankenduck CAD

This repository owns the standalone, editable Frankenduck Text to CAD model.
`src/` owns current geometry and assembly; `source/reference/` retains local
reference declarations and baseline data. Platform labels and external links in
those references are normalized. The project builds from local inputs.
Keep ordinary work within CAD. Robot calibration, firmware, deployment and
servo writes require a separate user request and authorization.

## Model and outputs

- Read `README.md` for the active design, dimensions, assumptions and check
  coverage, and `src/README.md` for the model catalog.
- Use the installed Text to CAD skill and the pinned local `.venv` interpreter.
  `./setup.sh` creates the environment; do not casually upgrade the CAD kernel.
- Edit parametric source in `src/parts/` and the assembly in `src/frankenduck.py`.
  Shared functional dimensions belong in `src/lib/`. Use millimetres and explicit
  datums. Inspect the requested saved part/reference before changing geometry.
- Keep standard parts as retained imported STEP inputs where possible. Record
  the source and mark approximate geometry clearly. The Radxa 6530B detailed
  model is approximate even though its outer dimensions are documented.
- Build the root after any child-part edit. Rebuilding a child alone does not
  update the full assembly. Keep the STEP and its sidecar together to retain
  joints, presets and animation; preserve linked component structure.
- Preserve existing occurrence IDs, names and mate bindings where possible.
  New geometry should be appended rather than silently renumbering existing
  references. Update local additions and checks when the assembly changes.
- Preserve Radxa USB-C-up orientation, GPIO jumper/cable allowances and cooler
  space unless the requested revision changes them. Do not treat modeled cable
  allowances or assumed screw dimensions as physical measurements.

## Component classification and print exports

`source/components.json` owns explicit roles, geometry fidelity, print intent,
material hints and print-release status for every part script. Every assembly
variant must map to one active catalog entry. Do not infer print intent from
filenames, imports or palette materials. The inherited aluminium appearance
on the inner bearing caps is incorrect; cataloged print intent is authoritative.
Unused source variants remain classified but are excluded from the print set.

Use `./cad parts` to inspect classification and quantities. After a current
build, `./cad export print` creates the tracked `printable/` bundle of individual
millimetre 3MFs, parts list and manifest. Keep exports explicit to preserve the
fast edit loop. Only active `printed_part` entries export; purchased hardware
and review volumes must never enter the print bundle. Export the saved assembly
geometry and retain stable occurrence references in its receipt. Native/mesh
validation failure must preserve the previous complete bundle.

Use the pinned native exporter. For STEP serialization damage, rebuilding the
current factory at its source-defined assembly placement is permitted only with
the bounds/volume comparison and explicit provenance recorded in the manifest.
Thin planar-face triangulation must preserve its sampled boundary, validate
polygon area and pass the final closed-mesh checks. Record all mesh tolerances;
never silently repair source geometry or accept a malformed print package.
Native conversions run in isolated processes. Small-face cleanup may operate
only on export copies at the documented precision, with valid-solid and
bounds/volume comparisons recorded in the manifest. Native mass-property
integration disagreement may use bidirectional solid differences instead;
never confuse differing integral estimates with an actual geometry change.
Keep native meshing attempts bounded and record their selected profiles.
Never relax those gates to make an export succeed.
The user explicitly requested that rounding artifacts not block export.
Balanced internal seam overlaps may therefore warn when there are no open
edges, winding balances and geometry checks pass. Keep their counts and native
library flags in the manifest; do not report those meshes as two-manifold.
Actual open boundaries, winding errors and geometry changes still fail.

Do not automatically mark a part released because export validation passes.
Print readiness, physical fit and motion clearance need their own evidence.
No print orientation, supports, material or slicer profile is selected by the
export command. Commit generated printable files with their matching catalog
and manifest when the user requests a commit or publication of a revision.

## Normal workflow

Run from the repository root:

```sh
./cad build
./cad check fast
```

Use `./cad check radxa` after Radxa-area edits, and `./cad check full` before
accepting a completed revision. `./cad status` shows current/stale/failed/missing
results. `--force` on a check bypasses passing cache entries. Checks deliberately
require a current build and never silently rebuild or bake animation exports.

`./cad review radxa` prepares the colored clearance model;
`./cad review assembly` identifies the full assembly. `./cad view` starts/reuses
CAD Viewer bound to `0.0.0.0` for LAN access. Generate and inspect appropriate
saved-artifact snapshots after visible geometry changes. Report source save,
build, measured checks, visual review and physical readiness separately.

## Keep checks bounded and traceable

- `checks/registry.json` owns each check's purpose, inputs, suites, output and
  coverage. Assertions live in `checks/assembly.py` and `checks/radxa.py`.
  Extend the suite that already owns a requirement before adding a new script.
- `checks/common.py` owns shared scene loading and intersection primitives;
  `tools/cad.py` owns orchestration, freshness, timing and reporting. Fast checks
  must avoid geometry loading and broad intersections.
- Register all relevant file dependencies, including shared dimensions and
  baselines. Changed inputs, check code and runtime versions must invalidate
  cached results. Missing data or failed computation must never count as a pass.
- Every geometry assertion needs units, tolerance, selected parts and pose or
  range. Current fit checks cover the saved rest pose; passing `full` does not
  establish motion clearance, thermal performance or manufacturing readiness.
- Use the existing bounds rejection before exact intersections. Reuse loaded
  geometry within a run. Run expensive geometry checks when a change warrants
  them; keep meshes, baked animation and screenshot batches explicit.
- Temporary probes belong in ignored `scratch/`. Discard them when finished or
  promote the useful assertion into a maintained suite. Avoid accumulating
  duplicate investigations in the maintained check directory.
- Runner regression tests: `.venv/bin/python -m unittest discover -s tools/tests -v`.
  Run relevant tests after changing runner/cache/report behavior; do not add
  tests that merely repeat low-impact implementation details.

## Evidence, history and Git

Generated artifacts, environment, caches, scratch and latest reports are
ignored. Source, irreplaceable imported inputs and frozen references are kept.
`reports/latest/` is replaceable current evidence; preserve required evidence
under `revisions/` when deliberately accepting a revision. Historical reports
in `reports/archive/` are historical, not current passing check receipts.

`tools/migration/` contains original-port utilities outside the normal workflow.
`prepare_port.py --replace-local-source` overwrites local CAD source edits; use
it only for a deliberate request to recreate the original port. Do not run it
as a fix for a build or fit failure. Preserve the frozen source and revisions.

Keep changes within the requested CAD scope. Never commit credentials, private
keys, environments, caches or scratch. Inventory large binary inputs and review
Git status before a requested commit. Do not automatically print, publish or
push changes. The repository is local unless the user requests a remote.
