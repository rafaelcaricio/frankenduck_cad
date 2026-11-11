# Open development head (review candidate v1)

An open head for development work. A skeleton takes the place of the top head shell and
leaves the head modules reachable. A slim skin clips onto the skeleton, without screws,
and closes the head to its original outline. This is a review alternative, like the LiPo
battery: both parts are cataloged but inactive, `source/open_head_variant.json` records
the alternative, and the default assembly and the printable bundle are unchanged.

All dimensions are millimetres. Part-local axes are those of the top head shell: +X
forward, +Y to the robot's left, +Z up. `src/lib/open_head.py` owns the geometry;
`src/parts/open_head_skeleton_native.py` and `src/parts/open_head_skin_native.py` are the
part scripts; `review/open_head.py` builds and checks the head assembly and
`review/export_open_head.py` writes test-print meshes.

## What holds what

No module is mounted on the top head shell. The head frame, the electronics deck, the
bulkhead and the Radxa bracket carry them, and stay as they are. The face plate is the
exception: it carries the Radxa bracket (with the board and cooler), the camera, the ToF
board and the eye, and it is held only by the bottom head shell and by the slot in the
top head shell's front edge. The skeleton therefore keeps that front edge whole.

## Skeleton

The skeleton is the original top head shell with material removed, and nothing added:

- **Front band.** Everything in front of local X 96 (X 101 at the top) is kept: the face
  slot (rib behind the plate, band in front), both front screw bosses and the original
  front edge. The face plate is held exactly as before.
- **Belt.** A strip along the seam with the bottom head shell, from local Z 274.5 up to
  Z 285. It keeps the seam lip and the rear and side screw bosses. It rises to Z 307.5
  over each jaw-pivot cutout, to Z 291 at the side boss and to Z 290 round the rear.
- **Fixing.** The same four M3 screws from below, into the same self-forming pilots.
- The shell's two inner ribs remain as short stiffeners where they cross the belt.
- **Lip.** The inner 1.25 of the wall stands 2 past the seam, all along the belt and the
  band's rear edge, behind the skin's edge. It locates the skin and stops it being pushed
  in. It is cut away at the two tongues and the three tab recesses.

Because the skeleton lies inside the volume of the accepted top head shell, it is no
closer to any other part than that shell, at any pose. It is one new printed part,
shown black. Solid PLA mass 38 g (29 325 mm³); the top head shell is 73 278 mm³.

## Skin

The skin is the outer 0.8 mm of the original shell (two 0.4 mm lines), above the belt
and behind the band, with a 0.3 mm seam gap all round. It is thin so it prints quickly
and suits translucent PLA. Its outside is the original surface, so the closed head keeps
its outline. Solid PLA mass 21.5 g (16 448 mm³).

| Feature | Modeled dimension |
| --- | --- |
| Skin wall / gap to the lip behind it | 0.8 / 0.25 |
| Front tongues | Two, 10 wide at local Y ±14 to ±24, 1.2 thick, reaching 5 under the band, 0.3 below its inner wall, 1.0 short of the face rib |
| Snap tabs | Three, 1.2 thick and 10 wide: one each side at local X −14 and one at the rear on the centre plane; each in a 1.45 deep outside recess of the belt with 0.3 side and 0.25 face clearance |
| Snap bumps | Half-round, radius 0.9, 5 long, at local Z 281.2, into 6.4 × 3.2 windows in the belt |
| Putting on | Hook both tongues under the band, swing the rear down on the skin's top front edge, press the three tabs until they click |
| Taking off | Pull the rear tab and both side tabs outward, lift the rear about 3.5°, draw the skin back |

The tongues with their pads are the only skin material inside the original shell wall.

## Checks

`review/open_head.py` writes `scratch/open-head-v1/validation.json`. At the rest pose:

- skeleton and skin are each one valid solid;
- the skeleton has no material outside the original top head shell (0 mm³);
- the original shell is unchanged in front of local X 101.5, and all four screw bosses
  are whole;
- no loose fragments were left by the cuts in either part;
- seated, the skin body, tabs, bumps and tongues do not overlap the skeleton; nearest
  distances for tabs, bumps and tongues are 0.25, 0.50 and 0.29;
- swinging on the skin's top front edge (local X 100.85, Z 335.5), the tabs and tongues
  stay clear through 6° and the skin body through 4°; the bumps ride over the belt wall
  in the first 2° (2.36 mm³ at 1°, 0.04 mm³ at 2°), which is the snap;
- the tongues are 2.3 from the face plate and more than 10 from the Radxa bracket,
  board, cooler and camera.

## Test-print files

`review/export_open_head.py` writes review-candidate meshes, not a canonical release, to
`scratch/open-head-v1/print-candidate/`: `open_head_skeleton_native.3mf` / `.stl` and
`open_head_skin_native.3mf` / `.stl`, with `manifest.json`. Parts keep their modeled
orientation with the lowest point on Z 0; print orientation and supports are not chosen.

Both meshes have no open, non-manifold or wrongly wound edges, and their bounds match
the CAD within 0.05. The native mesher exceeded its 0.02 face tolerance by up to 0.005,
so the Gmsh mesher (0.35 maximum edge) was used, as in the other review exports. Mesh
volume differs from the kernel's default CAD volume by 0.24 % (skeleton) and 0.30 %
(skin), within a 0.5 % limit set for these thin walls, where 0.2 % is about 2 µm of
surface offset. The adaptive volume integral used by the canonical exporter does not
converge on these parts and is recorded in the manifest only. The canonical
`./cad export print` bundle is unchanged.

## Not done, not verified

- No head motion sweep was run. The skeleton inherits the accepted shell's clearances;
  the skin's tongues were checked only at the rest pose.
- The cut edges are square. The kernel's edge rounding failed on them. With the lip, the
  exposed top edge is a 1.25 wall, which a 0.4 nozzle prints as rounded lines.
- Tab stiffness, snap force, the 0.85 belt wall behind each recess and the stiffness of a
  0.8 skin in translucent PLA are untested.
- With the top shell cut open, the front band is tied to the rest only by the belt and
  the two front screws. Stiffness with the Radxa and cooler hanging on the face plate is
  not analysed.
- Nothing is printed. Print orientation and supports are not chosen.
