# Design notes

All dimensions are in millimetres. Clearances below describe the modeled rest
pose; check physical components and fasteners before manufacturing release.

## Radxa mount and GPIO wiring

The Cubie A7Z is seated on four aligned M2 mounting holes with USB-C facing up.
The face bracket is a single solid with 3.2 mm rails and flush mounting pads.
It uses three M3 face attachments, four M2 pan-head wells and 1.6 mm M2 screw
grip. The M3 seats accept nominal 5.6 mm, 90-degree flat-head screws.

The GPIO allowance is 53 × 23 × 5.5 mm behind the PCB, followed by a
53 × 6 × 18 mm upward cable path. The 23 mm depth starts at the PCB seating
plane and includes the header. It extends 12.75 mm past the modeled header tip.
[Pololu's housing drawing](https://www.pololu.com/product/1900) gives a nominal
14 mm jumper housing length; the installed leads and cable bend remain
unmeasured.

The speaker sits on two aligned pilot holes at Y=-60. Its magnet recess leaves
5.25 mm of bulkhead thickness. Fit checks include the F688 bearing passages,
PCB seating, screw passages and head seats. The bracket, board, speaker,
bulkhead, cooler and both GPIO allowances are checked against surrounding
assembly parts with a positive-overlap threshold of 1e-5 mm³.

Run `./cad review radxa` to view the clearances: yellow is the plug/turn bay,
purple is the upward cable path and blue is the fan-inlet approach. These
volumes are excluded from the robot assembly and print set.

## Radxa 6530B cooler

[Radxa's dimensions and installation guide](https://docs.radxa.com/en/accessories/heatsink-case/heatsink-6530b)
specify a 65 × 30 × 11.5 mm cooler, a 57.8 × 22.9 mm mounting grid and a
16 × 16 × 1.5 mm thermal pad. The cooler mounts on the component/GPIO side.
Component documentation and images are in `source/vendor/radxa_6530b/`.

`src/parts/radxa_6530b.py` models the body, fins, fan and pad. Fin spacing,
21.5 mm fan casing, 4.8 mm feet, GPIO relief and 5 mm base standoff are modeling
assumptions. The standoff follows the modeled 3.5 mm SoC plus thermal pad.
The model remains a dimensioned approximation.

The fit suite checks the outer envelope, a 2 mm approach to the fan inlet and
three upper-cover wall samples of at least 2 mm. Inlet space is a clearance
allowance, not a thermal result. The cooler is rigidly attached to the board
and follows it in animation.

The PCB model uses a 58 × 23 mm hole grid. Its 0.112 mm nominal axis offset
from the cooler grid fits inside the bracket's 0.2 mm radial M2 allowance.
The supplied 4 mm screws leave only 0.65 mm nominal engagement through the
1.6 mm bracket grip and 1.75 mm PCB. Review screw length against the actual
PCB thickness and heatsink thread depth.

## PARKSIDE battery component

`src/parts/parkside_pap_204_a1.py` provides the PARKSIDE PAP 204 A1, 20 V 4 Ah,
as one selectable component with 66 solids and seven face colors. It is a
standalone library model; the robot assembly uses an NP-F550 battery.

The origin is the center of the bottom face. +X points toward the release
paddle, -X toward the contact nose and +Z points up. Its modeled envelope is
123 × 83 × 66.4 mm. Length and width follow product dimensions; the exterior,
rail and latch are photo-derived. This is a packaging model, not a measured
mating interface. CAD solid volume does not represent battery internals.
The lettering uses the bundled DejaVu Sans Bold font and license in
`source/fonts/`.

## Joints and poses

The assembly declares 15 revolute joints, 58 rigid attachments, `zero` and
`home` pose presets, and a non-looping walking/seated animation. The jaw's
±180-degree viewer range is a display range; its physical travel has not been
measured. The model's 15 servo representations describe the CAD assembly.
Hardware calibration belongs to the robot project.
