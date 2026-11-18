# Zeee 2S LiPo battery holder

The Zeee 2S 2200 mAh LiPo holder is the default battery mount. It replaced the
PARKSIDE stick pod, which stays as an inactive reference (last default in commit
`e883d4e`). `src/frankenduck.py` places the holder, pack, pad and leads, and
mount hardware; all are modeled in world coordinates and placed without
rotation. The trunk and both body shells are built for the holder.

`src/lib/lipo_pod.py` owns the base dimensions and socket;
`src/lib/lipo_compact.py` owns the inward placement and fitted shell relief;
`src/lib/lipo_profile.py` owns the rear cover that follows the body silhouette;
`src/lib/lipo_mount.py` owns the two arms, corresponding trunk changes and fasteners;
`src/lib/lipo_shoulders.py` owns the integrated upper shoulders, matching shell
relief and the final holder. The part scripts are `lipo_pod_native`,
`zeee_2s_2200_reference`, `lipo_pad_and_leads`, `lipo_mount_screws` and
`lipo_mount_inserts`; `trunk_base_native`, `left_shell_native` and
`right_shell_native` apply the holder's trunk and shell changes.
`source/lipo_variant.json` records the main datums.

## Pack reference

The user supplied a photo of a Zeee 2S 2200 mAh 50C pack and asked to use online
measurements. [Zeee's product specification](https://zeeebattery.com/products/zeee-2s-lipo-battery-2200mah-7-4v-50c-xt60)
lists 73 × 34 × 18.5 mm, 7.4 V, 2200 mAh, approximately 98 g, and an XT60 plug.
These are published dimensions, not measurements of the user's pack. The battery
CAD is a dimensioned reference with simplified wrapping. Leads, the XT60 and
balance plug are visual envelopes, not vendor geometry or verified cable lengths.

## Mechanical arrangement — V9 enclosed side shoulders

The battery slides down through an uncovered mouth. Removal lifts it clear
of the rear rim and then takes it away toward the rear.
The pack is at X = 0, Y = 48.5 mm, with its bottom at Z = 92 mm.
This moves it 19.5 mm toward the trunk from V5. V8 uses the four original battery mount positions described below.
The reduced rearward offset is established by CAD; the full robot
center of mass and physical balance have not been measured. Its upper
face is at Z = 165 mm; the rounded holder rim is at Z = 157 mm, leaving 8 mm
of pack exposed above the rear rim for a finger grip. The body-height front closing panel stands higher than the pack at the front. The nominal cavity retains 0.6 mm clearance
per side across X and 0.75 mm front/back. The entrance has a 0.5 mm lead-in
chamfer, and the outer rim is rounded by 0.5 mm.

The holder is one continuous printed part, including its 2.4 mm floor. A
separate 1.2 mm foam pad supports the pack. The former bottom cap, cap screws,
pilots and lower cable opening are removed. There is no top cover, latch or
crossbar. The pack rests on the padded floor and is guided by the cavity walls;
it is not latched.

Two integral arms replace the central mounting tongue. Their 17 mm widths span
X = 12–29 mm and −29–−12 mm; the flat forward portions are 8 mm deep, from
Z = 150.45–158.45 mm. The arms begin at Y = 27.7 mm and join the holder's front
wall with tangent 3 mm root curves and 0.5 mm exposed edge rounds. The leading
plan corners have 2 mm radii. The old raised pads are removed from the trunk.
The trunk's original 6 mm plate remains, with its central two holes filled.

Each arm uses two original carrier hole positions: X = ±18 and ±25 mm,
Y = 31.7 mm. Four M3 × 10 socket screws pass down through 3.4 mm trunk holes
into M3 × 5.7 heat-set inserts. The holder pilots are 4 mm diameter and 6 mm deep,
with a 3 mm tip passage below. Paired open recesses lower the head seats to Z = 162.45 mm, retaining 4 mm
of plate beneath each seat. The arm support faces contact its underside at Z = 158.45 mm. The paired recesses open at the rear edge instead of leaving thin strips
around separate counterbores. The two arms remain separate across the front,
while both are part of the single printed holder.

Fit the inserts before assembly, fasten the holder to the trunk from above,
then fit the cosmetic body shells. The mounting screws are assembly fasteners;
battery insertion/removal does not require touching them. Fasteners are modeled
from the installed bd_warehouse library; they are purchased parts, not printables.

The body color and rounded side outline are retained;
a continuous 1.8 mm front panel conceals the internals behind the body opening.
It joins the full depth of the shortened holder, from the floor to the body roof.
The intentional weight-relief space between the socket and outer side fairing
is closed by this panel; it is not a second battery pocket. The panel
upper surface follows the exact longitudinal body-shell spline from
`src/lib/body_profile.py`, shared with both shells. The raised rear tip is
removed and its exposed crest has a 0.5 mm radius. This preserves the body
silhouette while leaving the full battery opening uncovered.
V7 brings the rear cover inward while preserving the V6 battery placement,
shell geometry and front panel. V8 subsequently replaces only the mounting
interface. The upper rear follows the actual body
spline from Z = 140 mm to the rounded rear lip at Z = 157 mm. A tangent curved
transition from Z = 128–140 mm meets the straight lower cover at Y = 60.3 mm.
That lower section retains a continuous 1.8 mm rear wall behind the socket's
Y = 58.5 mm inner face. Following the body's original lower belly curve farther
inward would cut into the battery cavity. The lower rear projection reduces by
3.7 mm and the upper rear projection by about 5.3 mm. Both exposed rear edges
have 0.5 mm rounds, and the rear entry bevel remains. Boolean seam fragments
are removed before export; the holder remains one valid solid.

V9 closes only the openings beside the battery holder. The neck/service opening
is unchanged. The upper holder is rebuilt from the exact straight section below
the former dome, extended to the original body roof. Both shoulders now meet the
shell roof along a nominal 0.4 mm seam; the obsolete inward-curling dome ends and
short ledges are removed. Each shoulder is part of the same holder solid.
The central rear access is 49 mm wide, with 4 mm tangent runouts into the low rim,
0.5 mm exposed-edge rounds and a 0.5 mm tapered battery lead-in. The pack's full
35.2 × 20 mm insertion cavity remains open. There is no lid or crossbar.

The body shells keep their 1 mm skins, outside profile, screw interfaces and
neck opening. Their upper rear relief now follows the holder's straight upper
section instead of removing a full-width rectangular entry slot. The previously
accepted lower seams and 1.5 mm corner rounds are preserved outside the upper rear region (Y ≥ 36.3 mm, Z ≥ 154 mm);
they are not re-solved from the longer upper edges. The measured minimum seam
gap remains 0.367 mm. `src/lib/lipo_shoulders.py` owns the new upper construction
and combines it with the original lower finishing operation.

The pack's wire end faces upward. The modeled power leads, balance leads and
both connectors all remain above the mouth. Their dimensions and bends are
reference envelopes, not verified vendor CAD. Service is to disconnect the
battery, lift it 68 mm, then withdraw it 35 mm rearward. The pack bottom
is then 3 mm above the rear rim; the taller front panel remains beside
the pack and does not cover this exit. Lifting the loose leads 85 mm straight
up instead would bring their modeled route into the underside of the head. No holder screw or printed cover is removed.

In the assembly the pack and holder take the stick battery's and stick pod's
places in the occurrence list, and the KCD1 power switch is removed with the
stick pod's switch opening. Occurrences after the switch therefore move up one
place; the Radxa checks find the cooler by its label. The pad and leads, screws
and inserts are appended. The holder is fastened
to the trunk (`lipo_pod_mount`); the pack (`fix_zeee_battery`), pad and leads and
mount hardware are fastened to the holder. The stick's `battery_release` slider
and the `battery_removal` animation clip are removed; joint axes, spacing and
recorded poses are unchanged.

## Checks

`./cad check full` runs four maintained holder checks against the saved
assembly. Their reports are in `reports/latest/`. All distances are in mm and
apply to the CAD model only.

`assembly.battery_pod` (`battery_pod_verification.json`) checks the rest pose:

- The trunk, holder and both shells are each one valid solid.
- The pack body measures 18.5 × 34 × 73 mm at X = 0, Y = 48.5 mm and stands
  8 mm above the rim. Its 0.1 mm rear colour panels are reported separately.
- The pack keeps 0.600 mm in the cavity, 5.14 mm to each shell and 4.25 mm to
  the trunk. Each shell keeps 0.367 mm to the holder.
- The shells touch only at their two ties; elsewhere the centre split is 0.6 mm.
- Of the 41 static pairs, the new parts touch only at declared interfaces:
  holder to trunk, screws to trunk, inserts and holder, inserts flush with the
  arm clamp face, and the pad on the holder floor. The inserts overlap the
  holder only inside the four pilots (16.23 mm³ of modeled melt). The lead ends
  overlap the pack only in the top 1.5 mm.
- Each arm bears 97.25 mm² against the underside of the trunk plate, which is all
  of the arm face under the plate.
- 18 probes in the closed shoulders find solid material at least 2 mm deep.

`battery_pod.removal` (`battery_removal_verification.json`) sweeps the pack at
the home pose: 68 mm up, then 35 mm rearward. The pack keeps 0.6 mm to the
holder while lifting and 0.75 mm while moving rearward, and at least 2 mm to the
shells. The five lead and plug envelopes keep at least 2 mm to the holder and
shells.

`battery_pod.motion` (`battery_motion_verification.json`) checks all 1,702
recorded walking and sitting keyframes. 87 moving occurrences are checked
against the trunk, holder, pack, pad and leads, screws, inserts and both shells
(1,184,592 pairs). Nothing comes closer than the 1 mm limit; the proven lower
bound is 1.51 mm. Keyframes only: motion between them is not checked.

`battery_pod.quality` (`lipo_pod_quality.json`, limits in
`source/lipo_pod_quality.json`) checks the printed holder alone. Each checker
first finds the error in a known bad fixture; all 16 fixtures pass. The limits
are a 0.9 mm wall (two 0.45 mm lines), 0.5 mm edge radius, 0.05 mm short edge,
0.05 mm² small face and 0.1 mm narrow face. Symmetry about X = 0 has no
findings. The 26 designed sections measure 1.8 mm across the front panel (17),
2.4 mm across the floor (3) and 8 mm across the arms (6). All eight holes belong
to the four M3 × 10 screws. Each screw has a full head seat, a 3.4 mm trunk hole
with 0.2 mm radial clearance, a 4 × 6 mm pilot with a 1.6 mm insert wall,
5.7 mm of insert engagement, no tip contact 0.3 mm deeper, and a clear driver
path with the shells removed.

The remaining 95 quality findings come from the reviewed V9 shape, which the user
chose to keep. Each one lies in one of 31 named accepted areas with its reason:

- At the four lower corners (X = ±22.6 mm, Z = 88.4–88.9 mm), the side fairing's
  bottom round ends against the front panel and rear wall. Each end leaves a
  0.063 mm wide face (4 geometry findings) and a 0.004 mm sliver over 0.06 mm²,
  where one ray also grazes the surface (8 wall findings).
- The other 2 geometry findings are 0.044 mm edges where the 49 mm rear access
  opening meets the front panel at the roof.
- 79 edges are square or below the 0.5 mm radius: the arm clamp faces and insert
  tip passages (square by design), the pack mouth lead-in chamfer, and V9 edges on
  the front panel, shoulder roof, rear cover, rear lower corners and fairing round
  ends. The lower holder block hangs below the shells, so its square front, side
  and rear corners are exposed.
- 2 closed voids are the designed weight-relief chambers beside the socket.

The 0.9 mm wall limit is below the V9 upper side walls, which thin to 0.95 mm at
their curved corners.

These checks do not slice the part or test strength, physical fit or cable
bending.

## Review history

The V9 review output was `scratch/lipo-battery-v9/assembly.step`; the default
build reproduces its eight changed parts exactly (zero Boolean difference in
both directions and identical bounds). The receipts below are local review
records in ignored `scratch/` folders, made by local review scripts that are not
in the repository; the maintained checks are listed above. The V9 review checked
the three modified parts and reused the passed V8 receipt for the unchanged
mounting hardware, trunk, battery, wiring and lower geometry. Its receipt is
`scratch/lipo-battery-v9/validation.json`, tied to the saved assembly. It
checked valid native and STEP solids, face meshing,
new topology remnants, both seam gaps, material across the closed shoulder regions,
static hardware clearance, continuous battery/lead translation envelopes and the
recorded robot-motion samples. It does not certify physical fit or arbitrary
joint combinations. The saved V9 receipt passes 1,702 recorded keyframes over
87 moving occurrences (444,222 conservative pair checks), with a 2 mm clearance
lower bound for the changed regions. Both complete shell-to-holder seams have
0.367 mm minimum clearance and no overlap. All three saved parts are valid single
solids and every exported face meshes. The V8 mount receipt remains in
`scratch/lipo-battery-v8/validation.json`.
The preceding V7 profile receipt remains in `scratch/lipo-battery-v7/validation.json`.
The V8 check verifies unchanged parts and joint operations, holder/trunk topology
and meshes, removal of the central tongue and old pads, four screw joints and
insert support, assembly access, nearby installed parts and recorded robot motion.
Unchanged body-profile and battery service checks are inherited by artifact hash.

V8 passes the affected checks. Both revised printed parts are valid single
solids; every saved STEP face meshes, with no new small topology remnants.
The twelve sampled arm sections measure 8 mm. The center tongue and old trunk
pads are fully removed, and the old center holes are filled. All four screw
joints have complete head seating, 5.7 mm of insert engagement, at least the
library's 1.6 mm surrounding insert wall, and no screw-tip bottoming. Driver
access passes with the body shells removed. The screw heads clear both shells
by 1.25 mm; the new arms clear the nearest servo casings by 0.788 mm.

All 1,702 recorded walk/sit keyframes pass for 87 moving occurrences against
new arms, trunk fill, screws and inserts: 592,296 checked pairs, with a 2 mm
sampled clearance lower bound. The body-profile and open-top service envelopes
are unchanged. The annotated underside, both arms, their root sections, the
cleaned trunk underside, paired recesses and installed screw seats were reviewed.
Physical fit and strength remain untested.

The inherited V7 profile receipt passes: native and saved STEP are one valid solid, every holder STEP face
meshes, and no small Boolean seam faces remain. Twenty-one rear wall sections
retain at least 1.8 mm, and seven upper stations match the original body spline
within 0.005 mm. The added rear stock clears both shells by 1.574 mm; the overall
minimum holder-to-shell gap remains the inherited 0.367 mm. Pack clearance
remains 0.600 mm. The home-pose 68 mm upward removal plus 35 mm rearward exit
passes for the new stock; unchanged pack and wire paths retain the V6 receipt.
The marked side view, rear view, isolated holder and interior cutaway were
reviewed after the final lip rounding.

All 1,702 recorded walk/sit keyframes pass for 87 moving occurrences against
new rear material, with a sampled clearance lower bound of 2 mm. Unchanged
V6 geometry retains its prior recorded-motion validation. These checks cover
recorded samples, not continuous motion or arbitrary joint combinations.

The inherited V6 checks include twenty-two 1.8 mm front-panel sections, both
3 mm screwdriver paths, 1.864 mm nearest hip-servo clearance, and the complete
pack and modeled wire/connector removal paths. V7 changes only the rear region
at Y > 58.4 mm below Z = 157 mm. Short crest arcs and small fillet termination
faces are separately bounded; they do not represent thin material.

The variant STEP writer preserves XCAF colors/hierarchy and exports exact 3D
curves and surfaces without redundant PCurves. This avoids the pinned kernel's
unorientable bottom-face issue with the default writer. The builder asserts
that the saved holder reimports as one valid solid before replacing its file.
The canonical exporter is unchanged.

Physical pack fit, cable bending, retention and printing remain unverified.
The holder, trunk and both shells are in the printable bundle as CAD-checked
parts; that does not release them physically.

## Electrical scope

The holder change is mechanical. The two-cell LiPo reaches 8.4 V
when fully charged; changing the holder does not qualify the existing power
path, regulator behavior over the discharge range, or low-voltage protection.
The separate [power study](power.md) describes existing electrical assumptions;
it was written for the PARKSIDE stick, and its regulator proposal is not a
wiring instruction for this 2S pack.
No firmware, calibration, wiring or servo settings are changed.
