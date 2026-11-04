# Hip links with exposed horn bearings

The canonical left and right hip links use the approved V2 design: 5.4 mm flat
walls, the original rounded mounting-end profiles, and a continuous raised
reinforcing bar around the roll-side arm and elbow. The straight bar is 2.7 mm
proud of the wall (8.1 mm total), with smoothly blended shoulders and ends.
`src/lib/hip_link.py` owns the construction; the existing part names and assembly
placements are retained. No joint spacing, axes, limits or animation changed.

The 20 x 27 x 4 mm bearings sit outside the wall. A 21.6 mm lip contacts the
user-measured 22 mm inner ring, with 0.2 mm clearance to the shield. The main
walls sit 3.225 mm farther from the servos than the original walls; raised horn
lands preserve the mating planes. The inside elbow radius is 7 mm.

**Horn screws need 3.225 mm more grip length.** The printed grip is now 5.925 mm.
Choose screw length for the actual horn and thread engagement; the installed
screw length has not been measured. All eight mounting holes per link and their
counterbore access paths remain open.

## Reused approval evidence

The user approved the complete V2 assembly and explicitly requested that the
same checks not be run again for promotion. `reports/hip_link_v2_promotion.json`
retains the completed V2 measurements and source hashes: 32 wall sections,
16 counterbore access paths, valid single solids, STEP round-trip preservation,
zero bearing/horn overlaps, local hip-roll samples every 5 degrees plus both
31 degree end stops, and exact pitch-axis separation for the adjacent carrier,
shell and servo casing. The smallest conservative axial gap is 0.105 mm.

The motion evidence checks the complete links directly against the surrounding
hardware. An inconsistent old/new Boolean subtraction was discarded. These
checks do not validate combined whole-robot motion, strength or physical print
fit. The earlier full-project suite was not rerun during this promotion, and
its workflow receipts are not relabeled as a fresh pass.

The approved full assembly geometry is promoted directly, retaining the existing
kinematic/animation sidecar and normalizing only STEP occurrence labels. Only
the two changed hip print meshes are regenerated; unchanged print files retain
their prior receipts. New mesh-conversion results are recorded in the printable
manifest. This does not select material, orientation, supports or slicing.
