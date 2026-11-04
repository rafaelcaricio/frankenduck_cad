# Head and neck bearing design

The canonical neck bracket, head frame and yaw-to-roll link replace the reviewed
V15 assembly without changing joint axes or spacing. `src/lib/neck_pitch.py` and
`src/lib/head_bearings.py` own the final geometry; the original head/link stock
remains as `baseline_make` inside their part modules, not as exported variants.

The existing STS3215 adapter is used at head yaw and head roll, each with a
20 x 27 x 4 mm 6704 bearing. The neck pitch bracket uses the active metal horn
and supplied passive horn. Active horn OD is user-measured 19.85 mm, total height
5.8 mm, screw plate approximately 2.1 mm, and installed front projection 5.8 mm.
Passive OD 19.2 mm and its chamfer come from vendor CAD, not measurement.
Both neck upper horn seats have 0.20 mm radial clearance; flat screw mounting
faces remain in contact. These nominal allowances still require a physical fit.

The neck has four 6.8 mm screw wells with 4 mm grip, continuous exterior and
trimmed internal arch ends. The head frame has a 12.5 x 19 mm rounded plug window
and a removable roll adapter with a vertical insertion path. The yaw-to-roll
link retains a 9.5 x 19 mm cable opening and omits the fragile front lip above
the bearing relief. Modified parts keep MuJoCo colors; added adapters are black.

`./cad check full` includes `head_neck.fit`: nominal fits, screw openings,
vertical insertion, pitch +/-90 and yaw +/-170 in 10 degree increments, and
roll +/-25 in 5 degree increments. These are local sampled checks, not full
combined travel, physical strength, print calibration or a walking test.
Run `.venv/bin/python review/head_assembly.py` for the canonical head-only view.
Printable deliverables are generated through `./cad export print`.
