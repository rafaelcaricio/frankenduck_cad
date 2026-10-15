# Design notes

All dimensions are in millimetres. Clearances below describe the modeled rest
pose; check physical components and fasteners before manufacturing release.

## Radxa mount and GPIO wiring

The Cubie A7Z is supported at all four M2 corner screws, with USB-C facing up.
One continuous open carrier uses the three existing M3 face attachments. The board
seats at world Y=-89.015 mm. The 4 mm carrier depth provides 2 mm webs plus
2 mm raised screw seats, within the original outer head profile.
The M2 pan-head wells are 1.6 mm deep and the remaining M2 screw grip is 2.4 mm.
The M3 seats accept nominal 5.6 mm, 90-degree flat-head screws.

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

The printed U-shaped carrier has a 7 mm tall lower rail. The PCB contacts
four radius 3.2 mm annular screw lands only. All other support material has a measured minimum 2 mm clearance
from the modeled Radxa, including the imported microSD housing. The screw
islands are the local exception: the two right islands clear the microSD
housing by at least 1.3 mm. There is no rail or bridge across the button area.
The carrier is one connected solid with a flat mounting/printing face;
the main recessed webs are 2 mm thick and the screw seats rise another 2 mm.

The imported board omits the physical button, so a small photo-derived
envelope alone is insufficient. `radxa.mount` checks both
an approximate button allowance from Radxa's
[underside layout](https://docs.radxa.com/en/cubie/a7z/hardware-use/hardware-info)
and a broad open strip across the button area between the four screw islands.
It separately checks the PCB-contact islands, the 2 mm non-island gap and each
island's distance from the modeled microSD housing. The installed button and
solder joints remain unmeasured; the orange review volume is an allowance.
The original 23 mm GPIO-plug depth and full cable allowance remain reserved,
translated with the board. Their clearance is rechecked against the speaker.

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
five upper-cover wall samples of at least 2 mm. Inlet space is a clearance
allowance, not a thermal result. The cooler is rigidly attached to the board
and follows it in animation. The 0.8 mm board shift extends the existing
internal cover pocket and adds a shallow circular frame-edge relief. The
external head profile is unchanged; all sampled cover sections retain 2 mm.

The PCB model uses a 58 × 23 mm hole grid. Its 0.112 mm nominal axis offset
from the cooler grid fits inside the bracket's 0.2 mm radial M2 allowance.
The 4 mm kit screws are too short for the new 2.4 mm bracket grip plus
1.75 mm PCB. Four M2 x 6 screws give 1.85 mm nominal thread engagement.
Check the actual heatsink thread depth before choosing the final screws.

## PARKSIDE battery components

`src/parts/parkside_pap_204_a1.py` provides the PARKSIDE PAP 204 A1, 20 V 4 Ah,
as one selectable component with 66 solids and seven face colors. It is a
purchased component and is excluded from print exports. This variant does not
use it; the screwed backpack revision is commit `a94355e`.

The origin is the center of the bottom face. +X points toward the release
paddle, -X toward the contact nose and +Z points up. Its modeled envelope is
123 × 83 × 66.4 mm. Length and width follow product dimensions; the exterior,
rail and latch are photo-derived. This is a packaging model, not a measured
mating interface. CAD solid volume does not represent battery internals.
The lettering uses the bundled DejaVu Sans Bold font and license in
`source/fonts/`.

`src/parts/parkside_papk_12_a4.py` provides the PARKSIDE PAPK 12 A4, 12 V 2 Ah
X12V stick battery (Lidl NL 100406214), as one selectable component with 71
solids and nine face colors. It stands in the rump pod described below. It is
cataloged as a purchased component and is excluded from print exports.

The origin is the center of the underside of the red cap. +Z points up along
the stick, -Y is the front with the X12V TEAM nameplate, the release buttons
face +X and -X, and the T contact is on the flat back of the stick at +Y. The
modeled envelope is 45.2 × 51.5 × 82.0 mm, including the 1.5 mm feet. The
82 mm height and the 185 g mass are manufacturer values. Everything else is
fitted to six public product photos: about 0.5 mm on the cap, collar and stick
outlines and about 1 mm elsewhere. The photo-derived plan differs from the
listed 52 × 48 mm; the model follows the photos. The nameplate height varies
by about 1.5 mm between the photos, and the model follows its gap to the red
cap. The nameplate and underside artwork is traced from the photos; the text
uses the same bundled DejaVu Sans Bold font. The faint brand line on the small
underside plate is unreadable in the photo and is drawn as /// PARKSIDE. The
contacts and latches are drawn as they look, not to the X12V interface
tolerances. The latch teeth are hooks: a 1 mm notch under the catch at z 39.5
takes the tool's lip, and a lead-in ramp runs from z 40.6 to the top at z 43.
CAD solid volume does not represent the battery mass.

Three third-party X12 prints from Thingiverse served as fit references. They
are not stored in the repository:

- BabaYog's PAPK 12 A1 replica shell
  ([thing:4230897](https://www.thingiverse.com/thing:4230897))
- GLoL's double 12 V adapter
  ([thing:4445736](https://www.thingiverse.com/thing:4445736))
- GLoL's x12 DIY cell case
  ([thing:4646324](https://www.thingiverse.com/thing:4646324)). It is a plain
  envelope for 18650 cells, so it is used only for interface heights.

The modeled stick fits the adapter socket line to line (-0.03 mm), as does the
A1 replica's stick (-0.11 mm). The model follows the references on three
points:

- The stick is centred in the collar. The photos put it about 1.2 mm toward -X.
  All three references centre it, and it brings the teeth closer to the centres
  of the adapter's latch windows.
- The back wall is 17.0 mm below the rim, at z 19.5, as in all three
  references. The photos give about 16 mm.
- The latch catch is 3.0 mm above the rim, as in both replicas and the side
  photo. The adapter's lip needs at least 2.7 mm.

The collar step-down behind the buttons follows both side photos, from z 36.5
at y 7.0 down to the back wall at y 13.0. The adapter's -X cradle clears it with
no overlap. Check the stick position and the back wall with a caliper before
using either for a mating part. Two differences remain, and the model keeps the
photos:

- The adapter's +X cradle starts its slope about 3 mm further forward than its
  -X cradle. The modeled battery therefore stops about 8.1 mm short of the
  adapter rim. Its +X latch lip also reaches 0.7 mm into the +X tooth. Both
  photographed sides match the -X cradle.
- The x12 case has a straight slot behind each latch arm instead of the sloped
  step, and is 1 to 3 mm wider than the photographed battery.

## PARKSIDE stick pod

This variant replaces the PAP 204 A1 backpack with the PAPK 12 A4 X12V stick
battery. The backpack revision stays at commit `a94355e` for comparison.

The stick stands cap down behind the trunk plate in one printed pod. The pod's
fairing continues the body's back line and closes in a low rounded dome over the
stick. The red cap, the collar and both release buttons stay below the body.
World +Y is rearward and +Z upward. `src/lib/stick_pod.py` owns the geometry;
`source/assembly_data.json` records the placement and the `battery_release`
slider. The placement is a pure translation: stick-local x, y and z are world X,
Y − 58 and Z − 83.

| Feature | Modeled dimension |
| --- | --- |
| Battery world bounds X / Y / Z | ±22.59 / 32.27–83.72 / 81.5–163.5 mm |
| Pod world bounds X / Y / Z | ±31.1 / 21.0–87.3 / 119.8–171.0 mm |
| Pod and socket walls | 2.5 mm |
| Socket clearance round the stick | 0.6 mm; socket top 0.5 mm above the stick |
| Pod seat over the collar rim | 0.3 mm; the pod's lower edge is flat at this level |
| Latch lips | Top 0.2 mm under the hook catch; 0.8 mm engagement |
| Hook flex room | 1.5 mm behind each hook |
| + and − contact slots | 10 mm wide, stick-local z 60–82, open to the body |
| T contact window | 22 × 8 mm on the back, stick-local z 58–80 |
| Wire passes | 10 × 12 × 10 mm, one each side |
| Hidden top bay | 42 × 18 × 4 mm over the stick's front half, under the body top skin |
| Front tongue | 19 × 21 × 8 mm under the trunk plate, R2 corners |
| Screws | Two M3 × 10 socket head at X = ±5, Y = 25, from above |
| Plate holes / insert pilots | Ø3.4 / Ø4.0 × 6 mm for M3 × 5.7 heat-set inserts |
| Tongue clamp contact | 239 mm² |
| Shell-to-pod gap / shell split | 0.395 mm (0.4 nominal) / 0.6 mm |
| Battery to pod / trunk / shells | 0.2 (hook over lip) / 3.29 / 3.65 mm |
| XT60 bay | 8.6 × 16 × 24 mm at X 22.5–31.1, Y 28.6–44.6, Z 130–154 |
| Release | Press both buttons, pull 95 mm straight down |

The fairing is a loft of superellipse sections from the cap outline to a
4 × 1.6 mm oval just under the apex. Its closed face is split into four
patches. OCCT's point classifier misjudges about 1 % of outside points against
one closed loft face or a single apex point, which made distance checks report
false contacts.

The tongue's top face clamps flat against the trunk plate's underside. Fit the
two heat-set inserts in the tongue, hold the pod under the plate and drive both
screws from above through the shells' top service opening; a 3 mm driver has
60 mm of free length above each head. The trunk keeps its original geometry,
with the four obsolete carrier bores filled and the two screw holes added. The
shells keep the original outline and closed rear top return. They are cut round
the pod envelope with a 0.4 mm gap and stop 0.3 mm short of the centre plane.

The stick's + and − contacts face the front sides. Their slots run through the
socket wall and the fairing's front wall into the body, so a later adapter can
reach the contact plates and its leads can leave forward. The T contact window
opens on the back, inside the fairing. The wire passes join the pod's side
spaces to the body. The XT60 bay is on the +X side between the hip-yaw servo
and the pod, beside the − contact slot. It holds a mated XT60 pair (nominally
8.2 × 15.5 × 23.5 mm) with its axis vertical. Lead room continues 9 mm above
it beside the trunk plate's rear edge, and a 5.4 × 5 mm channel runs under it
toward the centre. No adapter, terminal or wiring is designed. The review
volumes `x12_plus_contact_allowance`, `x12_minus_contact_allowance`,
`x12_t_contact_allowance`, `x12_top_bay` and `x12_xt60_bay` keep these spaces
free.

The stick, its latch hooks, buttons and contacts come from the photo-derived
stick model and are approximate. The socket clearance, lips, flex room and
contact openings are nominal. Check them against a real battery and a test print
before relying on the fit. Printed strength is untested.

### Mass

The stick variant weighs 1.80 kg, 543 g less than the PAP 204 A1 revision
(2.344 kg). The whole-robot centre of mass at the home pose moves 21.8 mm
forward and 3.5 mm down, to (−0.9, 0.4, 142.2) mm. The basis is the
microduck_rl HOME receipt for CAD `94c002d` with the battery-area rows
replaced: printed parts as solid PLA at 1310 kg/m³, the battery at its listed
185 g at the stick's mid-height, and 3 g for the screws and inserts. Real
infill, wiring and the cells' mass distribution will differ. The walking policy
in microduck_rl is not updated.

### Checks

`assembly.battery_pod` checks the saved trunk, pod and shells as valid single
solids; the seated battery bounds; the shell split, shell-to-pod and battery
gaps; static overlaps of the five changed parts with every occurrence, where the
pod may touch only the trunk; the tongue contact area; both screw bores, head
seats, insert pilots, 1.5 mm insert walls, shanks and driver paths; the five
review volumes; and pod or shell cover over every 2 mm grid point of the stick
top. Numerical positive-overlap tolerance is 1e-5 mm³.

`battery_pod.motion` checks all 1,702 keyframes of `walk_sit_stand_walk_sit`:
30 leg occurrences against the battery, pod, trunk and both shells, totaling
255,300 pairs. Conservative bounding volumes reject pairs separated by at least
2 mm; nearby pairs use exact CAD distances. No pair comes within 2 mm. This
covers the recorded keyframes, not continuous interpolation or every
combination of the declared joint ranges.

`battery_pod.removal` checks the full 95 mm straight-down release at the home
pose. Battery material inside the stick outline grown 0.3 mm stays in that
column on the way down. The column meets no part; it stays 0.3 mm from the pod,
2.99 mm from the trunk and 3.29 mm from the shells. Of the 23 battery pieces
outside the column, 22 have swept boxes that meet no part. The last, the body
round the latch hooks, is checked against the pod with exact booleans every
0.25 mm. The hooks ride over the lips from 0.25 to 4.75 mm of travel. All of
that overlap lies inside the 0.8 mm lip zone, so each hook flexes inward at most
0.84 mm into a 1.5 mm flex room that holds no pod material. A 20 mm finger space
outside each button is free. The hook travel is sampled; arm bending, button
force and physical fit are not modelled.

## Joints and poses

The assembly declares 15 revolute joints, 57 rigid attachments, `zero` and
`home` pose presets, and a non-looping walking/seated animation. The jaw's
±180-degree viewer range is a display range; its physical travel has not been
measured. The model's 15 servo representations describe the CAD assembly.
Hardware calibration belongs to the robot project.
