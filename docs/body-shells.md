# Body shell connections

The two body shells keep their original exterior envelope with the skin reduced from
3 mm to 1 mm. Existing mounting features and the M2 x 8 mm tie seats retain
their local reinforcement. Two integrated internal ties connect the shells near the upper rear rim
and lower front curve. The upper tie has moved 7.3 mm rearward from the
initial layout, placing its entire 7.2 mm diameter behind the top service
opening edge at world Y = 28.5 mm; its centre is Y = 32.3, Z = 170.3 mm.
The additional 0.3 mm upward shift merges the tube with the inside of the roof,
eliminating the 0.11 mm slit that the purely rearward move would leave.
The lower tie remains at Y = -24, Z = 140 mm. Their layout follows the reference Microduck approach,
adapted to clear the Frankenduck IMU support, neck yoke and hip servos.
The right shell's obsolete switch opening and guard collar remain removed.

`src/lib/body_shell_fasteners.py` owns the shared dimensions. Both shell
factories trim the added supports to the original unbroken exterior envelope.
The ties meet at the centre plane; the rest of the shell seam retains its
0.6 mm total gap. No separate connector is printed.

## Internal support fillets

Both shells have continuous concave fillets at the tube-to-shell junctions:
1.5 mm radius around the lower/front root and 0.8 mm radius around the upper
root. The smaller upper radius preserves the nearby top service opening.
The blends run along the internal attachment and around the return into the
side panel. Broad panels remain 1 mm; screw seats, bore sizes, mating ends,
and installed screw locations retain the V3 dimensions.

## Hardware and assembly

Use two M2 x 8 mm self-forming screws, inserted from the right shell. Each screw
and its driver pass through a 4.6 mm tunnel to a recessed 2 mm thick seat, then
through a 2.2 mm shank passage into the left shell's blind 1.6 mm pilot.
Nominal engagement is 6 mm, including the 0.3 mm pilot lead-in. The pilot is
7.5 mm deep, leaving 1.5 mm beyond the nominal screw tip. Supports behind the
pilot are cored without opening the left outer skin.

The 1.6 mm pilot follows the existing project's M2 printed-plastic interface;
it is not a measured fit for a particular screw or filament. Verify the first
printed connection before tightening both. The viewer screws are approximate
hardware envelopes with 3.8 mm heads, not printable parts or detailed threads.
Use a driver whose shaft fits the 4.6 mm tunnel. Existing shell mounting screws
remain in place.

## Tie review evidence

`scratch/body-shell-fasteners-v4/assembly.step` shows the 1 mm shells with the
upper connection beneath the closed rim and internal fillets on both shells.
The front root radius is 1.5 mm and the upper root radius is 0.8 mm. Both shells
are valid single solids, ten broad-panel sections measure 1.0 mm, and the exact
rounded service-opening volume remains clear. Direct comparison against the
original exterior envelope finds zero outside volume. The screw passages,
head seats and blind pilot floors remain intact.

The fillets add approximately 193 mm3 on the left and 191 mm3 on the right.
Whole-shell subtraction between the coincident before/after surfaces failed
in the CAD kernel, so clearance validation uses the complete modified root
regions rather than an unreliable difference shape. Those regions include
existing skin and conservatively cover the full fillet reach. Their minimum
fixed-part clearance is 0.45 mm to the trunk base.

The local neck-pitch sweep passed at 29 angles from -90 to +46.84 degrees.
The 66 nearby root-region comparisons had a minimum gap of 4.01 mm.
Other head joints were held at neutral; this does not validate combined
head-joint motion or the full robot workspace. Review images include the marked
camera angle, both interiors, and sections normal to the tube axes.

The fillets widen and smooth the attachment to the shell. Physical strength,
print stiffness and screw fit remain unverified. These shells are now the
default, relieved round the LiPo holder as described below, and they are in the
printable bundle.

## Relief round the LiPo holder

Both default shells are relieved round the Zeee 2S holder by
`lib.lipo_shoulders.body_shell`. Each shell is built twice: once with the lower
holder relief, which then gets a 1.5 mm tangent round and curved runout across
the full wall at its lower rear tip (`src/lib/body_shell_finish.py`), and once
with the upper relief that follows the holder's straight upper section and
closed side shoulders. Above world Y = 36.3 mm and Z = 154 mm the second shell
is used; below, the first. The 1 mm skins, screw interfaces, ties, neck opening
and outer shape are unchanged. See [lipo-battery.md](lipo-battery.md) for the
holder and the maintained checks that cover the shells.
