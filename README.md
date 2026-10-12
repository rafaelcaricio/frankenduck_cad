# Frankenduck CAD

Standalone parametric Text to CAD project originally ported from **The Frankenduck, STEVE revision 91**,
captured on 2026-10-12. The active model includes the local Radxa revision below.
The original import reference is [The Frankenduck in STEVE](https://stevecad.studio/?document=ddc406fe-ad83-4176-be52-cf163852c1ac).
This repository owns the current editable CAD in `src/`, its interpreter, cache, imported inputs,
exports, and review evidence. It has no dependency on robotd, hardware calibration,
or microduck_rl, and building it does not contact STEVE or write to a robot.

Repository: `~/development/frankenduck_cad`. Working guidance is in [AGENTS.md](AGENTS.md).

## Local Radxa and jumper-wire revision

The active `STEP/frankenduck.step` changes five original occurrences: the Radxa bracket
`#o1.6`, the Radxa board `#o1.15`, the speaker `#o1.16`, and its bulkhead mount
`#o1.3`, and a small internal heatsink clearance in the upper shell `#o1.10`.
It adds the cooler as `#o1.74`. The other 68 original occurrences retain their placed geometry. The
frozen source and original STEP remain in `source/steve/` and `revisions/r91_import/`.
This revision is local to the project and makes no STEVE edits.

- Flip the bracket front/back about vertical Z; retain USB-C facing up.
- Seat the PCB on the support and align all four M2 mounting holes. Move the
  board 12 mm sideways to give straight jumper plugs access past the jaw mount.
- Remove the three long tubes. The bracket is one solid with **3.2 mm thick
  rails and mounting pads**, four M2 pan-head wells with 1.6 mm screw grip, and
  three flush 90-degree M3 flat-head seats. It uses the face's spare lower-left
  attachment instead of its upper-left attachment, which the shifted PCB covers.
  The M3 seats allow nominal 5.6 mm screw heads; pan-head M3 screws would protrude.
- Reserve **53 × 23 × 5.5 mm** behind the PCB for the GPIO header, individual
  jumper housings and their cable turn, followed by a **53 × 6 × 18 mm** upward
  cable path. The 23 mm depth is measured from the PCB seating plane and includes
  the existing header. It extends 12.75 mm past the modeled header's rear tip.
- Shift the speaker 5 mm rearward and shorten its two mounting posts to the
  new seating plane. Its magnet receives a shallow recess leaving 5.25 mm of
  plate thickness. The original bearing seat and flange geometry remain intact.

[Pololu's housing drawing](https://www.pololu.com/product/1900) specifies
14.00 ± 0.25 mm housing length. It is a dimension reference for the allowance,
not an identification of the user's jumper leads. Exact housings, screw heads,
wire diameter and physical bending remain unmeasured.

Open the [plug-and-cable clearance review](http://192.168.1.88:3245/?file=STEP/radxa_fit_review.step).
Yellow marks the plug/turn bay and purple marks the upward cable path. These
are review volumes and are absent from the complete robot assembly. Context
parts are translucent; use the full assembly link below for the physical model.

`checks/radxa.py (mount)` reopens the saved STEP and checks modified parts
and both allowance volumes against every other occurrence in the stored rest
pose, with a positive-overlap threshold of 1e-5 mm³. It also checks PCB seating,
four M2 axes, three M3 axes, screw passages/head seats, the two speaker pilot
axes, preserved bearing geometry and the other 68 original placed parts. Results are
saved in `reports/latest/radxa_mount_fit.json`. Reviewed images are in `review/radxa_fit/`.
This is a CAD fit revision for review, not a manufacturing or hardware release.

## Official Radxa heatsink representation

The user's official Cubie A7Z cooler is represented as **Radxa 6530B**. The
original imported PCB had GPIO, SoC and port geometry but no cooler. Radxa's
[6530B specifications and installation guide](https://docs.radxa.com/en/accessories/heatsink-case/heatsink-6530b)
give **65 × 30 × 11.5 mm**, mounted on the component/GPIO side. Product photos
and catalog-search provenance are retained in `source/vendor/radxa_6530b/`.
The searched STEP Parts catalog had no matching 6530B, Radxa heatsink or Cubie
record; the checked Radxa accessory downloads did not offer a 6530B STEP.

`src/parts/radxa_6530b.py` is a dimensioned representation, **not a vendor-exact
STEP**. It has a finned metal body, fan casing/rotor and one 16 × 16 × 1.5 mm
thermal pad. Fin spacing, 21.5 mm fan casing, 4.8 mm feet, GPIO relief and the
5 mm base standoff are explicit modeling assumptions. The standoff follows
the existing coarse model's 3.5 mm SoC height plus the uncompressed pad.
These details need checking against the physical cooler before a print release.

`checks/radxa.py (heatsink)` checks the **full official outer envelope**
against other robot parts, checks a **2 mm clear approach to the modeled fan
inlet**, and measures the remaining upper-cover wall at the internal relief.
It excludes the parent PCB from the full-box test because real cooler cutouts
leave the SoC, GPIO and ports accessible; the actual representation is checked
against the PCB and both plug/cable allowances by `radxa_mount_fit.py`.
The inlet allowance is a spatial margin, not a thermal performance result.
Blue in the clearance review marks this inlet space.

Radxa's mechanical hole grid is 57.8 × 22.9 mm, while the imported PCB uses a
coarser 58 × 23 mm grid. The nominal axis offset is 0.112 mm and remains within
the bracket's 0.2 mm radial M2 clearance. The supplied 4 mm screws need reviewing
with the bracket in the stack: the model's 1.6 mm bracket grip plus 1.75 mm PCB
leaves only 0.65 mm of thread engagement. Longer M2 hardware may be needed;
actual PCB thickness and cooler thread depth have not been measured.

The cooler has one new rigid attachment and follows the board's original
animation transforms. All original occurrence references, mates and keyframes
are retained. The pre-cooler STEP and source are saved under
`revisions/radxa_jumper_bay_before_heatsink/`.

## Standalone PARKSIDE battery

[`src/parts/parkside_pap_204_a1.py`](src/parts/parkside_pap_204_a1.py) ports the
[PARKSIDE PAP 204 A1 — 20 V 4 Ah battery in STEVE](https://stevecad.studio/?document=d08bce05-3ec5-4847-9ca4-395f8f05d915)
at **revision 6** as one reusable local component. Its case, slide rail, contacts,
locking tooth, release paddle, charge button, lens and side artwork remain grouped
inside one selectable battery component containing 66 solids. The 30 source
geometry groups are internal construction details, not separate viewer parts.
Colours are retained on all 1,628 STEP faces. The project-local viewer patch
displays the original seven-colour palette while keeping this as one component.
The component is standalone; the robot assembly still uses its existing NP-F550.
The component catalog classifies the PARKSIDE as purchased hardware, inactive in
the assembly and excluded from print exports.

Build it independently from the project root:

```sh
CADGEN_CACHE_DIR="$PWD/.cadgen" PYTHONPATH=src .venv/bin/python src/parts/parkside_pap_204_a1.py
```

This produces `STEP/parts/parkside_pap_204_a1.step`. Other models
can reuse `parts.parkside_pap_204_a1.make()`. The build uses the existing pinned
local CAD runtime and needs no STEVE connection. The original source, exported
reference STEP and baseline measurements are frozen under
`source/steve/parts/parkside_pap_204_a1/`. The DejaVu Sans Bold font and its license
are retained under `source/fonts/`; the model declares the font as a build input
and uses its explicit path to reproduce the lettering across machines.

Units are millimetres. The origin is the centre of the bottom face; +X points
toward the red release paddle, -X toward the contact nose, and +Z points up.
The modeled envelope is **123 × 83 × 66.4 mm**. The STEVE source cites a listed
product height of **70 mm** and product mass of **711 g**, while following the
photos for the 66.4 mm paddle height. Those product claims are preserved from
the source. Other dimensions, including the rail and latch, are photo-derived:
this is an exterior/packaging reference, not a measured mating drawing. Solid
CAD volume and appearance materials do not reproduce the battery's internal
construction or product mass.

Port verification is recorded in `source/port_audit/parkside_pap_204_a1.json`:
saved local geometry is checked against the retained STEVE export and baseline,
including one leaf component, retained face colours, original solid ordering,
positive valid solids, bounds, faces, volume and
bidirectional geometric differences. Isometric and opposing views are reviewed
from the generated STEP. These checks establish port fidelity; the battery's
physical interface and installation in the robot remain untested.

`./view.sh` and `./cad view` automatically prepare a local copy of cadgen 0.6.5's
viewer client in the ignored `.cadgen/viewer-dist/` directory. The patch in
`tools/prepare_viewer.py` and `tools/viewer/face_colours.mjs` fixes both the direct
mesh converter and its background-worker copy: they now populate vertex colours
from the tessellator's face ranges. Previously, both discarded those colours for
single components. Uniform and uncoloured parts retain their existing path.
The installed package and pinned CAD kernel remain unchanged. Generated asset
names include the patch hash, so an existing tab loads the corrected code after
refresh. An incompatible future runtime stops preparation for review.

Run the colour regressions with
`node --test tools/tests/test_viewer_face_colours.mjs`. Browser verification is
saved in `review/parkside_pap_204_a1/single_component_colours_viewer.png`; the
ordinary bundled CLI snapshot client is separate from this viewer patch.

## Components and printable files

[`source/components.json`](source/components.json) explicitly classifies every
part model and the review volumes. Roles, geometry fidelity and print-release
status are separate fields. Source names, imports and appearance materials do
not decide print intent. `./cad parts` lists roles, quantities and print selection;
`./cad parts --json` provides the same catalog summary for tooling.

The active assembly uses 45 printed components and 29 purchased-component
occurrences. Eleven retained source variants are inactive. Purchased hardware
includes the electronics, servo horns, servos, battery, switch and heatsink.
The inner bearing caps are printed parts: their inherited aluminium appearance
label is incorrect metadata, and their print material remains unspecified.

For printing without running CAD, open [`printable/README.md`](printable/README.md).
The tracked `printable/3mf/` contains individual millimetre 3MFs, with quantities
and material hints in `printable/PARTS.csv` and provenance/mesh measurements in
`printable/manifest.json`. All current print-release statuses are `unreviewed`.

To regenerate the complete bundle from the saved assembly:

```sh
./cad build
./cad export print
```

Exports are explicit and never added to the normal build/edit loop. The command
runs fast metadata/catalog checks, extracts only active printed occurrences,
normalizes translation to centre XY and set min Z to zero, and validates each
3MF's units, closed mesh, winding, positive volume and bounds/volume agreement
with native geometry. Rest-pose axes are preserved; material, orientation,
supports and slicer profiles remain the user's choice. These conversion checks
do not establish printability or physical readiness.

The native OCCT/Lib3MF exporter first tries Watson at 0.01 mm absolute
deflection with surface refinement disabled. A controlled Watson profile for
curved lofts uses 0.05 mm deflection, 0.2 rad angular deflection and 0.01 mm
minimum mesh size. Further attempts use Delabella at 0.0005 mm and Watson at
0.00001 mm, with 0.1 rad angular deflection. Each attempt has a 180-second limit.
Boundary seams are welded within 0.0001 mm. Omitted planar mesh regions may
use constrained triangulation only when their sampled boundary is valid and
triangle samples lie within 0.001 mm of the native surface. Actual mesh
topology and geometry are checked after serialization; per-part settings,
recoveries and prior failures are recorded in the manifest.
Balanced internal seam overlaps with no open edges and consistent winding
are warnings rather than export failures. They can arise when rounding
collapses opposing internal surfaces. The manifest records their count and
Lib3MF's validity/manifold flags; inspect affected parts in the slicer. Open
edges, unbalanced winding and non-positive volume still fail. Print-release
status remains unreviewed.
The exporter rebuilds each current source factory with its exact assembly
placement to avoid STEP serialization damage. Each result must match saved
bounds within 0.00001 mm. Volume must agree within `max(0.02 mm³, 1e-6
relative)`; when native mass integration disagrees, bidirectional native
solid differences must instead total at most 0.02 mm³. The comparison and
geometry source are recorded explicitly. It does not edit or heal CAD source.
Conversions run in isolated worker processes because native geometry can share
kernel data across occurrences. If triangulation still fails, OCCT small-face
cleanup at 0.0001 mm precision may be applied to an export copy. The cleaned
copy must remain one valid solid and match the same bounds/volume tolerances;
the manifest records the cleanup and its measured effect. Failed checks prevent
publication of the entire bundle.

An unchanged, hash-verified bundle is reused. `./cad export print --force`
regenerates it. Publishing is atomic: an incomplete or failed export never
replaces the previous bundle. Edit source/catalog and regenerate, rather than
editing generated print files. Commit updated print files and their manifest
alongside a completed CAD revision. Review the export manifest before committing.

`assembly.catalog` belongs to the fast suite and rejects unclassified source
models, ambiguous/missing assembly mappings, and non-print export selections.
Geometry fidelity distinguishes design geometry, imported references,
dimensioned approximations and vendor CAD whose accuracy has actually been
verified; imported models are not automatically vendor-exact.

## Setup

From a fresh checkout, install `uv` and run `./setup.sh` from the repository root.
It creates a local Python 3.12 `.venv` and installs the exact dependency versions
in `requirements.txt`. The VTK-enabled OCP wheel is installed last because it
shares an import namespace with cadgen's no-VTK dependency. Environment and
cache directories are ignored by Git; recreate them for each checkout rather
than copying them. Tracked scripts resolve paths relative to this repository.

The package pins describe the validated runtime. Upgrade them deliberately,
then rebuild and validate before recording new pins. Build, check and review
commands below use the local interpreter without requiring shell activation.

## Open and build

### Maintained workflow

Use `./cad` from this project directory. The normal edit loop is:

```sh
./cad build
./cad check fast
./cad review radxa
./cad view
```

After editing the Radxa area, run `./cad check radxa`. Before accepting a
completed revision, run `./cad check full`. Use `./cad status` at any time to
see current, stale, failed and missing results. `./cad check full --force`
reruns passing checks; `./cad build --force` forces a CAD rebuild. Running
checks never silently rebuilds the model or generates animation exports.

| Suite | Coverage | When to run |
| --- | --- | --- |
| `fast` | Build freshness, component classifications and joint/pose/animation metadata; no CAD intersections | Ordinary edits |
| `radxa` | Metadata, mount/GPIO/cable allowance checks, heatsink envelope/inlet/cover checks | Radxa-area edits |
| `full` | All five maintained checks, including saved assembly inventory; provenance receipt on success | Completed revision |

Passing `full` means all **registered checks** passed. Their current coverage
is the stored rest pose, assembly declarations and the specific dimensions
documented below. Motion sweeps, tool/connector insertion paths, physical cable
bending and cooling performance remain untested. Historical import audits do
not run in `full`, because local geometry changes deliberately differ from r91.

`checks/registry.json` owns each maintained check's ID, engineering purpose,
explicit file dependencies, suite membership, report path and coverage.
Assertions live in `checks/assembly.py` and `checks/radxa.py`; the shared
`checks/common.py` handles scene reuse, bounds rejection and intersection timing.
`tools/cad.py` owns orchestration and reporting. Fast checks do not import the
CAD kernel. Geometry suites share saved scenes within a single invocation.

Passing results are reused only when the declared file contents, check code,
registry and relevant Python/CAD package versions match. Build receipts also
track all model Python/animation sources, assembly data, imported STEP inputs,
requirements and the build script. A changed saved STEP or sidecar invalidates
the receipt. New files matching an input pattern also invalidate it. Missing
inputs, changed report contents and interrupted runs cannot count as a pass.
Commands acquire a nonblocking workflow lock to prevent overlapping builds or
report writers. The legacy `build.sh` still works, but run `./cad build` afterward
to establish its receipt before using the check runner.

Generated results and logs live in ignored `reports/latest/`, with the overall
status in `summary.json` and per-check fingerprints/times in `*.receipt.json`.
`provenance_manifest.json` is generated after a successful full suite.
Its validation summary is embedded as a snapshot; refreshable status and viewer
reports are excluded from its file hashes so opening status cannot invalidate it.
Historical reports retained during this restructuring live under `reports/archive/`;
they have no current workflow receipts. Deliberate revision evidence belongs
under `revisions/`. Preserve required latest reports there when accepting a
revision; overwriting `latest` does not create an ever-growing history.

Review job configurations live in `review/jobs/`. `./cad review radxa` rebuilds
the separate colored allowance model; `./cad review assembly` identifies the
saved full assembly. Both require a current build. The viewer remains bound to
`0.0.0.0`; opening it does not run checks. Screenshots and mesh/animated exports
are explicit tasks, outside the normal edit loop.

Temporary probes belong in ignored `scratch/` (the former `tmp/` directory).
After an investigation, discard them or move the useful assertion into an
existing suite. Add a new maintained check only for a new engineering
requirement, with its inputs, tolerance, pose/coverage and run suite registered.
Avoid a new standalone script when an existing suite already owns the question.
Original reconstruction utilities are isolated under `tools/migration/`; see
[their usage and overwrite boundary](tools/migration/README.md).

Runner regression tests use temporary fixtures and do not load CAD geometry:

```sh
.venv/bin/python -m unittest discover -s tools/tests -v
```

- [Complete interactive STEP assembly](http://192.168.1.88:3245/?file=STEP/frankenduck.step)
- [Animated GLB](http://192.168.1.88:3245/?file=GLB/frankenduck_animated.glb)
- `./build.sh` rebuilds `STEP/frankenduck.step` and its sidecar.
- `./view.sh` starts or reuses a viewer bound to `0.0.0.0` for this directory and prints its actual URL.
  The current LAN address is `192.168.1.88`; other computers can use the links above.
- `./setup.sh` recreates the isolated Python environment when needed; it requires `uv`.

Use Text to CAD's **Kinematics** tab for the joint sliders and `zero` / `home`
presets. Use **Animation** for `walk_sit_stand_walk_sit`: 58.12 seconds, 1,702
original keyframes, linear interpolation, one playback ending in the recorded
seated pose. The trunk remains grounded, as in the canonical STEVE design.
The animated GLB contains 1,744 samples at 30 fps; the cooler adds four moving
body occurrences to the original 296.

## What was ported

The frozen assembly contains 41 pinned linked documents plus seven embedded
imported components, placed as 73 named occurrences. The local cooler makes 74.
The component library has the original 59 entrypoints (51 editable geometry
variants and eight imported STEP models), plus the dimensioned cooler.
It preserves the mirrored leg variants, the ToF parameter override, the original
placements except for the local revision above, 15 revolute joints, the original
57 rigid attachments plus one cooler attachment, the two
pose presets, body colors, and the active recorded animation.

The geometry source retains its CadQuery/OCP construction through a small local
adapter, then returns build123d shapes to cadgen. It is executable Text to CAD
source with independent per-model STEP outputs. It is not a reconstruction of
STEVE's graphical feature editor. Original comments and source files are retained
byte-for-byte under `source/steve/`; transformed, editable factories are under
`src/parts/`. All imported geometry is stored locally under `STEP/imported/`.

STEVE's historical simulation mass overrides and lumped masses are preserved in
`source/assembly_data.json` and the original `mass_record.py`. They are not
physical mass measurements and are not a cadgen dynamics model. Source comments
identify that frozen training reference as predating the updated camera, ToF,
speaker, and Radxa orientation.

## Format adaptation

STEVE declares no measured jaw range. cadgen requires finite limits for every
revolute slider, so this viewer uses **jaw = -180 to +180 degrees solely as a
presentation range**. The unmodified source record retains the absence of a
measured limit. The other 14 revolute limits and all pivots/axis signs match the
frozen canonical source. The model contains 15 STS3215 CAD instances, including
the jaw; that count is preserved from the CAD rather than inferred from runtime
hardware configuration.

## Import verification and current outputs

The complete assembly built and was reopened with cadgen's saved STEP reader:
**74 named occurrences, 314 solids**, in millimetres. Its rest envelope is
approximately **190.350 × 194.580 × 366.733 mm**.

- `revisions/r91_import/checks/geometry_audit.json`: the original 59 reconstructions are valid; 48
  baseline comparisons pass the volume tolerance `max(0.02 mm³, 1e-6 relative)`.
  The face plate is verified separately, and ten variants have no directly
  applicable current standalone baseline (stale previews, embedded imports, or
  an assembly parameter override).
- `reports/archive/import/face_equivalence.json`: exact B-rep subtraction is empty in both
  directions. Adaptive volume integration at tolerance 1e-9 agrees within
  1e-5 mm³. The ordinary kernel volume estimator reported a 0.737 mm³ difference
  for equivalent boundaries; that is not treated as a geometric difference.
- `reports/latest/assembly_verification.json`: saved artifact SHA-256, all occurrence
  names, references, solid counts, volumes, and placed bounds.
- `reports/latest/metadata_verification.json`: all 72 original mate bindings plus the cooler,
  original pivots, axes,
  limits, presets, original keyframes, and non-looping duration checked.
- `review/`: reviewed rest isometric/front, HOME isometric, and seated-end PNGs.
  Live viewer playback was also exercised and the final frame was checked.

The separate [STEVE geometry reference](https://stevecad.studio/?document=c3b740ee-146e-4b23-a9bc-9ff920903616)
was created only for export and comparison; the canonical document was not edited.
Its optional full comparison export was still processing at handoff;
`reports/archive/import/steve_reference_status.json` records that boundary. A build or visual
review is not a new collision study, manufacturing release, or hardware test.

The initial source-equivalence and broad review reports describe the frozen
import. The current Radxa changes are covered by the separate mount fit report;
the original reconstruction audit is not a source-equivalence claim for the three
locally edited part factories; the cooler is a new local model.

The primary deliverables are `STEP/frankenduck.step` plus
`STEP/frankenduck.step.json`, `GLB/frankenduck.glb`, and
`GLB/frankenduck_animated.glb`. Keep the STEP sidecar alongside the STEP to retain
kinematics and procedural animation. Static GLB alone carries geometry; the
animated GLB carries the baked clip.
