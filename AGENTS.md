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

## User preferences for mechanical revisions

These preferences come from the neck, head and bearing redesign reviews. Apply
them throughout this project unless the user explicitly changes the requirement.

### Design intent

- Preserve the original external appearance of modified Frankenduck parts where
  possible. Prefer internal reinforcement over external bulk. Use the Microduck /
  DuckBuddy reference for mechanical layout, compact proportions and integrated
  supports, adapted to the actual STS3215 hardware.
- Preserve existing joint spacing and usable movement. A static assembly that
  fits is insufficient evidence of preserved motion. Check the affected travel
  and state the limits of sampled or local checks; do not claim full robot motion
  from them. Discuss any required reduction in travel or change of joint axes.
- Prefer continuous, integrated carriers and reinforcement. Remove obsolete stock
  when rebuilding a region instead of layering patches over it. Do not leave
  thin tips, slivers, fragile lips, projecting support ends, buried remnants or
  material crossing openings after Boolean operations or smoothing.
- Retain MuJoCo colors on existing parts, including parts that were modified.
  Only entirely new printed parts are black. Servos are black; bearings are
  aluminium/gray. A modified part is not a new part for coloring purposes.

### Fits, hardware and assembly

- Use authoritative local hardware models and the user's measurements. Keep
  measured dimensions, vendor CAD dimensions and assumptions explicitly distinct.
  Preserve the measurement datum: total horn height, plate thickness and installed
  projection are different quantities. Do not silently substitute one for another.
- Include explicit assembly and running clearances where appropriate. Zero
  overlap does not establish clearance or print fit. Measure the actual geometry,
  not just the parameter used to construct it. Distinguish intentional contact at
  screw-clamped mounting faces from clearance around horns, casings and adapters.
  A clearance chosen for one interface is not a universal fit allowance.
- Preserve screw holes, recesses, fastener lands, tool access and cable openings.
  Check how the servo, horn, bearing and adapter are inserted and removed, not
  only whether they fit in the final pose. Keep removable adapters separate when
  assembly access requires it.

### Complete the revision before handing it back

- Apply feedback to the latest accepted design even when an annotation references
  an older viewer file. Carry all accepted corrections forward. Keep explicitly
  reverted experiments reverted; an old snapshot is not a rollback instruction.
- Inspect the marked view plus the inner, opposite, underside or sectional views
  needed to expose hidden leftovers and depth. Review the whole affected feature
  and its counterpart, not just the side most recently edited. The user should
  not have to discover each remaining defect in successive reviews.
- Treat visual intent, valid topology, clearances and motion as separate checks.
  One valid connected solid is not proof of a smooth, finished or printable part.
  A passing collision check does not excuse a thin remnant or an obstructed hole.
- Rebuild the relevant assembly and show it in the built-in CAD viewer with the
  actual servos, active/passive horns, bearings, adapters and surrounding parts
  needed to inspect the change. Identify the revision being shown.
- When intent is clear, complete authorized edits, inspection and verification
  without repeated permission questions. Ask only when unresolved information
  materially changes the result. Report what changed, what was checked and what
  remains unverified; do not present nominal CAD fit as physical validation.
- When printable exports are requested, export the correct current revisions and
  inspect the resulting meshes for deformation, missing features and material
  crossing openings. Successful file generation alone is insufficient. Give the
  exact local folder and filenames, and distinguish review candidates from the
  canonical printable release. Follow the existing promotion and export workflow.

## Build, check and review

```sh
./cad build
./cad check fast
```

Rebuild the root assembly after editing a child part. Run `./cad check radxa`
for board, bracket, speaker or cooler changes, and `./cad check full` before
accepting a completed revision. `./cad status` distinguishes current results
from stale, missing or failed checks. Checks never silently rebuild.

`./cad review radxa` creates the clearance model. `./cad review pod` renders the
stick pod's views and sections; examine every picture after a pod change.
`./cad view` starts or reuses the viewer on `0.0.0.0` for LAN access. Review the saved artifact with suitable
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
