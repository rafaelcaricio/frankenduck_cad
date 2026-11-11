# Jigs

Printed aids for measuring the robot. They are not part of the assembly or of the printable bundle.

## Hip roll foot board

[hip_roll_foot_board.py](hip_roll_foot_board.py) is a plate that both soles are strapped flat
onto, for measuring the hip roll zeros with the servos limp. Print file:
[print/hip_roll_foot_board.3mf](print/hip_roll_foot_board.3mf) (80 × 186 × 5.5 mm, 42 cm³).

The soles are wedges: the bottom face of `sole_left_native` is tilted 4.84° about the foot's long
axis, so a sole lies flat when its leg is rolled 4.84° inward. With both soles flat on the plate,
each hip roll joint is at 4.84° inward minus the trunk's roll.

Build, check and export:

```sh
cad run jigs/hip_roll_foot_board.py          # prints the model's id and its viewer page
cad run jigs/check_hip_roll_foot_board.py    # fit against the feet of the robot model
cad model export <id> jigs/print/hip_roll_foot_board.3mf jigs/print/hip_roll_foot_board.stl
```

`cad` here is the installed Text to CAD command, not this repository's `./cad` script.

The foot outline comes from the robot's MuJoCo model, posed with the soles flat:
[reference/feet_flat_pose.py](reference/feet_flat_pose.py) writes `reference/feet_flat_pose.stl`
from microduck_rl's `simulation/geometry/scene.xml`. The check reads that file: the feet rest on
the plate, each has 1.5 mm of room inside its fence, the strap passes over a toe at most 25 mm
high, and nothing of the leg is above the strap's path.

Print flat as modelled (fences up), no supports; the two grooves on the underside are 16 mm
bridges. Any rigid filament.

Use:

1. Robot off or not initialised, so the servos are limp. Board on a flat table, arrow pointing to
   the robot's front.
2. Put each foot inside its fence, heel against the back fence.
3. One strap per foot over the toe, ahead of the ankle servo: a rubber band, a cable tie or a
   hook-and-loop tie. It goes down through the slot between the feet and the notch in the outer
   edge and runs under the plate in its groove. Pull it tight enough that the sole cannot rock.
4. Hold the trunk upright with one hand, or lean its back on a box. The trunk's roll is read from
   the IMU, so it need not be exactly level.
5. Read the hip roll counts. The script for it is `hip_roll_zero.py` in the study folder
   `microduck-training-sts3215/walk_robot_20261110/`. Turn the board half a turn on the table and
   read again; the mean of the two readings cancels a table that is not level.

Not verified: the fit on the real feet (the outline is the model's), and that the printed soles
have the CAD's 4.84° wedge.
