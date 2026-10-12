# Working on Frankenduck CAD

This repository owns the editable Frankenduck Text to CAD model. Keep work
within this CAD project; calibration, firmware, servo writes and deployment
require a separate user request.

## Model ownership

- Use the installed Text to CAD skill and pinned `.venv` interpreter.
  `./setup.sh` creates the environment. Keep dependency upgrades deliberate.
- Edit geometry in `src/parts/`, placements in `src/frankenduck.py` and shared
  functional dimensions in `src/lib/`. Use millimetres and explicit datums.
- `source/assembly_data.json` owns the occurrence inventory, mates and poses;
  `src/animation.js` owns the timeline. Update these with assembly changes.
  Preserve stable occurrence names and references where possible.
- Use local purchased-component inputs in `STEP/imported/`. Keep their origin
  and any dimensional assumptions documented. Approximate hardware must stay
  marked approximate.
- Read [docs/design.md](docs/design.md) before Radxa-area edits. Preserve USB-C
  facing up, jumper-plug/cable space and cooler allowances unless the requested
  change explicitly revises them.

## Build, check and review

```sh
./cad build
./cad check fast
```

Rebuild the root assembly after editing a child part. Run `./cad check radxa`
for board, bracket, speaker or cooler changes, and `./cad check full` before
accepting a completed revision. `./cad status` distinguishes current results
from stale, missing or failed checks. Checks never silently rebuild.

`./cad review radxa` creates the clearance model. `./cad view` starts or reuses
the viewer on `0.0.0.0` for LAN access. Review the saved artifact with suitable
snapshots after visible geometry changes. Keep the STEP and sidecar together.
Report build, CAD checks, visual review and physical readiness accurately;
the maintained fit checks cover the rest pose.

## Components and printable exports

`source/components.json` owns roles, geometry fidelity, print intent, material
hints and print status. Every part script must be classified. The inner bearing
caps are printed parts. Purchased hardware and review volumes never export.

Use `./cad parts` to inspect selection and `./cad export print` to update the
tracked `printable/` bundle. Exports remain explicit so they do not slow ordinary
editing. Commit matching models, catalog and printable files together for print
revisions. Do not edit generated meshes by hand.

Preserve the exporter's units, valid-solid, geometry and mesh checks. Balanced
numerical seam overlaps may warn when there are no open edges and winding is
consistent. Open boundaries, winding errors and geometry changes still fail.
Record conversion settings and warnings in the manifest; failed conversion
must preserve the previous complete bundle. Successful export does not mark a
part physically released or choose its print orientation and slicer settings.

## Keep the workflow small

- `checks/registry.json` declares check purposes, inputs, suites and coverage.
  Extend an existing check before adding another script. Keep fast checks free
  of native geometry loading and broad intersections.
- Include every relevant dependency in the registry. State units, tolerances,
  selected parts and pose for geometry assertions. Missing data or failed
  computation is a failure, never a pass.
- Reuse loaded geometry and bounds rejection. Run expensive checks when a
  change warrants them; keep mesh exports and screenshot batches explicit.
- Temporary probes belong in ignored `scratch/`. Remove them when finished or
  promote a useful assertion into the maintained suite.
- After changing workflow, catalog, exporter or viewer behavior, run the relevant
  regressions: `.venv/bin/python -m unittest discover -s tools/tests -v` and,
  for viewer colors, `node --test tools/tests/test_viewer_face_colours.mjs`.

Git owns design history. Keep current source, required component inputs and
printable files; ignore environments, caches, working outputs and temporary
reports. Review scope before committing. Never commit credentials or private
keys. Push or publish only with user authorization.
