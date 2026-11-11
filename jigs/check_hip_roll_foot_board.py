"""Check the foot board against the robot's feet in the flat-sole pose.

    cad run jigs/check_hip_roll_foot_board.py

Reads `reference/feet_flat_pose.stl` (the robot model's feet and lower legs, soles on z = 0) and
the board's own dimensions. Prints each measure and stops on a failure.
"""
import struct
from pathlib import Path

import numpy as np

import hip_roll_foot_board as fb

raw = (Path(__file__).parent / "reference/feet_flat_pose.stl").read_bytes()
count = struct.unpack_from("<I", raw, 80)[0]
rec = np.frombuffer(raw, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=count, offset=84)
pts = rec["v"].reshape(-1, 3).astype(float)

heel = fb.SOLE_X[0] - fb.ROOM
toe = fb.SOLE_X[1] + fb.ROOM
inner = fb.SOLE_Y[0] - fb.ROOM
outer = fb.SOLE_Y[1] + fb.ROOM
ok = True


def report(what, value, good):
    global ok
    ok &= bool(good)
    print(f"{'ok  ' if good else 'FAIL'} {what}: {value}")


report("feet rest on the plate (lowest point of the feet, mm)", f"{pts[:, 2].min():.2f}", abs(pts[:, 2].min()) < 0.01)
for side, name in ((1, "left"), (-1, "right")):
    foot = pts[pts[:, 1] * side > 0].copy()
    foot[:, 1] *= side
    low = foot[foot[:, 2] <= 3.0]
    room = min(low[:, 0].min() - heel, toe - low[:, 0].max(), low[:, 1].min() - inner, outer - low[:, 1].max())
    report(f"{name} foot: least room to its fence (mm)", f"{room:.2f}", 1.4 <= room <= 2.0)
    band = foot[(foot[:, 0] >= fb.STRAP_X[0]) & (foot[:, 0] <= fb.STRAP_X[1])]
    report(f"{name} foot: height under the strap (mm)", f"{band[:, 2].max():.1f}", band[:, 2].max() < 27)
    report(f"{name} foot: width under the strap (mm)", f"{band[:, 1].min():.1f}..{band[:, 1].max():.1f}",
           band[:, 1].min() > fb.SLOT_Y[1] and band[:, 1].max() < fb.NOTCH_Y)
    ahead = foot[(foot[:, 0] > fb.STRAP_X[0] - 2) & (foot[:, 2] > 27)]
    report(f"{name} leg: nothing above 27 mm from 2 mm behind the strap forward", f"{len(ahead)} points", len(ahead) == 0)
    report(f"{name} foot: sole edge to the plate's side edge (mm)", f"{fb.HALF_WIDTH - foot[:, 1].max():.1f}", fb.HALF_WIDTH - foot[:, 1].max() > 5)
report("fence top below the level the outline was taken at (mm)", f"{fb.FENCE_HEIGHT:.1f}", fb.FENCE_HEIGHT < 3.0)
report("web left under the strap groove (mm)", f"{fb.THICKNESS - fb.GROOVE_DEPTH:.1f}", fb.THICKNESS - fb.GROOVE_DEPTH >= 1.5)

board = fb.hip_roll_foot_board().shape
report("board volume (cm3)", f"{board.volume / 1000:.1f}", 35 < board.volume / 1000 < 50)
if not ok:
    raise SystemExit("a check failed")
