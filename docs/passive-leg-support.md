# Passive leg supports and bearing inner-ring contact

The approved V9 design is canonical. Each leg uses one continuous black support
combining the ankle servo adapter and the supplied STS3215 rear/passive horn mount.
Do not also print or install the superseded `bearing_ankle_adapter`.

The support has a 19.2 mm upper web and a 2.6 mm normal wall through its curved
transition. The 19.2 mm horn seating pad is integral, 1 mm behind the main web;
the four original horn holes remain. Joint axes and spacing are unchanged.
The passive side does not use another bearing.

Each upper-leg shell retains the rounded exterior, extended 3 mm outward, with
the approved inner tail removed. An integrated internal bridge supports the two
lower shell screw positions behind the moving arm. All eight original servo
screw positions per shell are retained.

## Printing and hardware

The canonical bundle is `printable/3mf/` with matching `printable/stl/` files.
Print one of each new support, upper shell and shallow bearing carrier, left and
right. The complete bundle also includes the updated bearing-contact parts:
hip links, yaw/roll cradles, lower legs, ankles, neck pitch and head yaw/roll link.
Their 21.6 mm rotating contact lips bear on the user's measured 22 mm inner ring;
0.4 mm axial shield relief does not enlarge the existing bearing seat.

The two near-horn shell fasteners are nominal **M2 x 5 mm countersunk screws per
leg (four total)**. Their modeled 2.6 mm printed grip leaves 2.4 mm nominal servo
engagement. Verify actual thread depth and head seating before tightening.
The six other shell fasteners per leg retain the M2 x 6 mm nominal envelopes.
Horn screw heads are clearance envelopes, not an exact purchased fastener model.

The mesh files are unsliced. Preserve 100 percent scale and millimetres. Orient
the broad supports on a broad face with appropriate support under the offset
regions; do not assume the exported rest-pose orientation is a print orientation.
Keep support scars out of bearing seats and screw lands. Printer, polymer,
perimeters and support settings remain a slicer choice. Export validation does
not replace a physical assembly, retention or strength test.

## Reproducibility and validation

`src/lib/passive_leg_support.py` and `src/lib/passive_leg_shell.py` build the
canonical parts. The three named `STEP/imported/passive_*_stock.step` inputs capture
the prior canonical carrier, front adapter and approved lower ankle mounting stock; they are required source
inputs, not scratch outputs. No canonical factory reads a review directory.

The promotion is checked against the approved V9 parts by bidirectional solid
difference. `passive_leg.fit` checks valid solids, 2.6 mm bend wall, bores, eight
shell mounts, sampled original knee and ankle travel, and shell movement past
the support and passive horn. `bearing.inner_ring` checks all twelve contacts.
The normal full check suite also covers assembly declarations and the recorded
battery-area motion. These are sampled CAD checks, not physical validation.
