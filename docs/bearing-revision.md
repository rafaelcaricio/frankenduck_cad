# Frankenduck horn-bearing revision

This promotes the reviewed two-leg design into the canonical CAD assembly. It is a **test-print revision, not a physical or full-motion release**. Original native factories remain as `make_original()` where revised geometry derives from them. `src/lib/bearing_revision/` contains the editable Boolean, reinforcement, adapter and cradle construction. Its baseline placements are captured in `source/bearing_revision_baseline.json`; baseline geometry is frozen in `STEP/imported/bearing_baseline_*.step` with hashes in `source/bearing_revision_inputs.json`, preserving the exact topology used in the reviewed Boolean design. These inputs do not read this revision's generated output. Original native factories remain editable as provenance; changing a baseline requires explicitly rebuilding and revalidating the bearing revision.

The current canonical revision uses the approved V9 shallow carriers, continuous passive-horn/ankle supports and deeper original-shape upper-leg shells, alongside the reinforced hip links, cleaned knee/ankle bearing reliefs, yaw-to-roll cradles, roll adapters and shared yaw adapter. The two standalone ankle adapters are superseded by the integrated supports. All twelve active-horn bearing interfaces now use the measured 22 mm inner-ring contact correction. [Passive support and print requirements](passive-leg-support.md) owns the V9 construction, shell screws and bearing-contact details. Joint spacing and recorded trajectories are unchanged.

## Dimensions and assumptions

- Purchased bearing envelope: 20 mm bore, 27 mm OD, 4 mm width (6704-ZZ).
- User-measured metal horn: 19.85 mm OD, 5.8 mm total height including hub, approximately 2.1 mm mounting-plate thickness.
- Installed horn datum remains the review model's provisional seating datum; physical fit must confirm it.
- Adapter recessed side clearance: 0.2 mm per side; mounting pads remain flush.
- Cradle servo side and roof allowance: 0.2 mm; roof lowered 3.363296 mm.
- Four yaw-horn screw counterbores per cradle: 6.2 mm diameter, 3.363296 mm depth, retaining 3.4 mm through holes.
- Two adapter-to-cradle screws per side: nominal 2 x 8 mm countersunk **plastic thread-forming screws**, exact SKU unselected. Cradle pilot diameter 1.6 mm, entry diameter 2.2 mm, entry depth 0.3 mm. These are printed pilots, not modeled helical threads. Test the material-specific screw fit before assembling the robot.
- Vendor STS3215 input: `STEP/imported/sts3215_vendor_bearing_reference.step`, copied from the reference used in the reviewed prototype. Rear casing uses a repaired outer envelope for visual context. Horn and bearing envelopes are dimensional references, not certified purchased-part CAD.

## Remaining limitations

Full combined hip yaw/roll motion is **not validated**. Previous checks found upper yaw-seat interference and broad shared-bridge interference at combined extreme poses; subsequent local cradle improvements do not resolve or revalidate that whole-motion problem. Do not treat a successful maintained rest-pose suite as evidence that the original full motion envelope is preserved.

The reviewed geometry also has legacy yaw-bracket/rear-case overlap inherited from the original model. Material-strength, retention, screw selection, print orientation, supports and real-world fit remain unverified. Print-release status stays `unreviewed`.

## Appearance and exports

`source/viewer_palette.json` captures visual colors from `microduck_rl/simulation/geometry/robot.xml`. Existing parts, including the modified hip links, cradles, lower legs and ankle brackets, retain those colors. Only entirely added carriers and adapters are black. Servo housings are nearly black and bearings/screws metallic gray. Viewer colors are appearance choices, not material/strength certification. No MuJoCo model or hardware calibration is modified.

Regenerate with `./cad build`, `./cad check full` and `./cad export print`. The print bundle includes all active printed parts, with individual 3MF and binary STL copies, quantities, hashes, topology checks, dimensions and mesh-to-CAD comparisons. Purchased hardware is excluded. STL coordinates are millimetres by convention and STL has no color metadata. Files are centred in X/Y with minimum Z zero; this is not an optimized print orientation. Failed conversion preserves the previous complete bundle.

## Promotion verification

The promoted pieces were compared with the reviewed two-leg assembly. Fourteen of the fifteen placed printed pieces have zero bidirectional Boolean difference. The left carrier has identical bounds and a volume difference below 0.000001 mm³; it has different face splitting after the runtime conversion, and its bilateral Boolean comparison is inconsistent. An independent comparison of 7,578 boundary samples in both directions found a maximum surface distance of 0.0000001 mm. This is a bounded sampled comparison, not a proof over every surface point. Full values and artifact hashes are recorded in `reports/bearing_revision_promotion.json`.

The pinned runtime conversion explicitly extracts placed solids before Boolean operations and constructs raw common results as compounds. Regression tests cover both behaviors so a displaced compound or empty cast cannot silently pass an interference assertion.

For a fresh built-in static viewer document with the saved STEP occurrence colors, run `~/.local/bin/cad run review/canonical_assembly_view.py`. The canonical STEP and its sidecar retain the assembly/animation data. The subsequent color-only correction restores the original MuJoCo colors of modified parts; it does not change the geometry comparison above.

## Print export correction (2026-11-04)

The shared yaw adapter bridge previously crossed both circular horn openings. The bridge now has full-depth 20.25 mm horn-clearance openings and the existing 25.4 mm race relief, while retaining the bearing seats and mounting pads. The built part asserts one valid solid, clear horn passages and preserved seats. The corrected actual STL was visually reviewed with both openings clear.

The previous STL release also flattened some curved surfaces. The exporter now clears cached triangulations, enables surface deflection control and internal vertices, and samples triangle centroids and edge midpoints against the exact trimmed CAD surfaces. The fixed sampled deviation limit is 0.02 mm. Exact surface projection with face-wire classification handles periodic-face distance-query failures without accepting material across holes. These are sampled checks, not a continuous error proof.

Difficult surfaces can use an isolated, pinned Gmsh fallback. Each mesh still passes bounds, volume, open-edge and winding checks, and each binary STL is reread to verify the float32 conversion. Balanced seam overlaps are permitted by the repository rules and are explicitly recorded in the manifest; inspect those parts in the slicer.

The canonical build and all nine implemented full checks passed after the opening correction. The recorded-pose obstacle check covers 46 articulated occurrences over 1,702 poses; it is not a continuous or complete self-collision sweep. Thirty-six Python regression tests passed. Full combined hip yaw/roll movement remains unverified as described above.

The current export inventory is 52 unique printable models (53 pieces), each supplied as 3MF and STL. The active catalog excludes the superseded standalone ankle adapters. Files retain rest-pose axes and are centred in X/Y with the lowest Z at zero. No automatic print orientation, supports, material or slicing settings are selected, and physical fit and strength remain untested. Use the current `printable/manifest.json` for per-part conversion receipts and warnings.
