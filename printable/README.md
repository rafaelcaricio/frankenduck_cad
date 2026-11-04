# Frankenduck printable parts

This directory contains **50 printable models, each supplied as 3MF and STL** for the printed components used in the current assembly. Purchased electronics, servos, horns, batteries, the heatsink, unused variants and review volumes are excluded.

Open the files in `3mf/` directly in your slicer. Each file represents one native CAD solid and explicitly uses millimetres. Any numerical seam-overlap warnings are listed in the manifest; inspect affected meshes in your slicer. Left/right variants have separate filenames. Quantities and material hints are in [PARTS.csv](PARTS.csv).

Print-release status is **unreviewed**. The exports have passed mesh integrity and geometry-conversion checks; these do not establish printability or physical fit. Choose material, orientation, supports and slicer settings for your printer. Material hints are listed in PARTS.csv; choose the final material and slicer settings for your printer.

Geometry keeps its saved rest-pose axes, translated to centre X/Y and put its lowest Z at zero. No automatic print orientation is selected.

The [manifest](manifest.json) records source/catalog hashes, dimensions, volumes, mesh checks and export settings. Regenerate from the repository root with `./cad build` followed by `./cad export print`. An unchanged bundle is reused; `--force` regenerates it. This directory is generated and committed. Edit the model/catalog and regenerate rather than editing these files.

| File | Quantity | Material hint |
| --- | ---: | --- |
| [ankle_bearing_inner_cap_left_native.3mf](3mf/ankle_bearing_inner_cap_left_native.3mf) | 1 | unspecified |
| [ankle_bearing_inner_cap_left_native_right.3mf](3mf/ankle_bearing_inner_cap_left_native_right.3mf) | 1 | unspecified |
| [ankle_bearing_outer_keeper_left_native.3mf](3mf/ankle_bearing_outer_keeper_left_native.3mf) | 1 | pla |
| [ankle_bearing_outer_keeper_left_native_right.3mf](3mf/ankle_bearing_outer_keeper_left_native_right.3mf) | 1 | pla |
| [ankle_eye_left_native.3mf](3mf/ankle_eye_left_native.3mf) | 1 | pla |
| [ankle_eye_left_native_right.3mf](3mf/ankle_eye_left_native_right.3mf) | 1 | pla |
| [ankle_left_native.3mf](3mf/ankle_left_native.3mf) | 1 | printed polymer; original MuJoCo color; material and slicer profile unselected |
| [ankle_left_native_right.3mf](3mf/ankle_left_native_right.3mf) | 1 | printed polymer; original MuJoCo color; material and slicer profile unselected |
| [bearing_ankle_adapter.3mf](3mf/bearing_ankle_adapter.3mf) | 2 | black printed polymer; unselected |
| [bearing_carrier_left.3mf](3mf/bearing_carrier_left.3mf) | 1 | black printed polymer; unselected |
| [bearing_carrier_right.3mf](3mf/bearing_carrier_right.3mf) | 1 | black printed polymer; unselected |
| [bearing_roll_adapter_left.3mf](3mf/bearing_roll_adapter_left.3mf) | 1 | black printed polymer; unselected |
| [bearing_roll_adapter_right.3mf](3mf/bearing_roll_adapter_right.3mf) | 1 | black printed polymer; unselected |
| [bearing_shared_yaw_adapter.3mf](3mf/bearing_shared_yaw_adapter.3mf) | 1 | black printed polymer; unselected |
| [bottom_head_shell_native.3mf](3mf/bottom_head_shell_native.3mf) | 1 | pla |
| [electronics_deck_native.3mf](3mf/electronics_deck_native.3mf) | 1 | pla |
| [face_part_native.3mf](3mf/face_part_native.3mf) | 1 | pla |
| [face_radxa_bracket_native.3mf](3mf/face_radxa_bracket_native.3mf) | 1 | pla |
| [foot_left_native_a.3mf](3mf/foot_left_native_a.3mf) | 1 | pla |
| [foot_left_native_a_right.3mf](3mf/foot_left_native_a_right.3mf) | 1 | pla |
| [foot_left_native_b.3mf](3mf/foot_left_native_b.3mf) | 1 | pla |
| [foot_left_native_b_right.3mf](3mf/foot_left_native_b_right.3mf) | 1 | pla |
| [head_frame_native.3mf](3mf/head_frame_native.3mf) | 1 | pla |
| [hip_left_native.3mf](3mf/hip_left_native.3mf) | 1 | printed polymer; original MuJoCo color; material and slicer profile unselected |
| [hip_left_native_right.3mf](3mf/hip_left_native_right.3mf) | 1 | printed polymer; original MuJoCo color; material and slicer profile unselected |
| [imu_yoke_bridge_native.3mf](3mf/imu_yoke_bridge_native.3mf) | 1 | pla |
| [jaw_native.3mf](3mf/jaw_native.3mf) | 1 | pla |
| [jaw_soft_001_native.3mf](3mf/jaw_soft_001_native.3mf) | 1 | fdm-tpu |
| [left_shell_native.3mf](3mf/left_shell_native.3mf) | 1 | pla |
| [leg_left_native.3mf](3mf/leg_left_native.3mf) | 1 | printed polymer; original MuJoCo color; material and slicer profile unselected |
| [leg_left_native_right.3mf](3mf/leg_left_native_right.3mf) | 1 | printed polymer; original MuJoCo color; material and slicer profile unselected |
| [neck_001_native.3mf](3mf/neck_001_native.3mf) | 1 | pla |
| [neck_002_native.3mf](3mf/neck_002_native.3mf) | 1 | pla |
| [neck_internal_yoke_native.3mf](3mf/neck_internal_yoke_native.3mf) | 1 | pla |
| [neck_pitch_001_native.3mf](3mf/neck_pitch_001_native.3mf) | 1 | pla |
| [noenoeil_native.3mf](3mf/noenoeil_native.3mf) | 1 | pla |
| [right_shell_native.3mf](3mf/right_shell_native.3mf) | 1 | pla |
| [roll_front_bulkhead_native.3mf](3mf/roll_front_bulkhead_native.3mf) | 1 | pla |
| [soft_mouth_top_native.3mf](3mf/soft_mouth_top_native.3mf) | 1 | fdm-tpu |
| [sole_left_native.3mf](3mf/sole_left_native.3mf) | 1 | fdm-tpu |
| [sole_left_native_right.3mf](3mf/sole_left_native_right.3mf) | 1 | fdm-tpu |
| [stick_pod_native.3mf](3mf/stick_pod_native.3mf) | 1 | pla |
| [top_head_shell_native.3mf](3mf/top_head_shell_native.3mf) | 1 | pla |
| [trunk_base_native.3mf](3mf/trunk_base_native.3mf) | 1 | pla |
| [ubec_clamp_native.3mf](3mf/ubec_clamp_native.3mf) | 1 | pla |
| [upper_leg_left_native.3mf](3mf/upper_leg_left_native.3mf) | 1 | pla |
| [upper_leg_left_native_right.3mf](3mf/upper_leg_left_native_right.3mf) | 1 | pla |
| [yaw2roll_left_native.3mf](3mf/yaw2roll_left_native.3mf) | 1 | printed polymer; original MuJoCo color; material and slicer profile unselected |
| [yaw2roll_left_native_right.3mf](3mf/yaw2roll_left_native_right.3mf) | 1 | printed polymer; original MuJoCo color; material and slicer profile unselected |
| [yaw_roll_link_native.3mf](3mf/yaw_roll_link_native.3mf) | 1 | pla |

## STL copies

Matching binary STL files are in `stl/`. Import them as millimetres: STL cannot encode units or colors. They contain the same validated triangles as the 3MF files, checked again after float32 conversion. The CAD assembly carries the viewer colors. See `docs/bearing-revision.md` in the project for the bearing revision's unresolved movement and fit limitations.
