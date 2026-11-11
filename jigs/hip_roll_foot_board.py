"""Foot board for measuring the hip roll zeros: both soles strapped flat on one plate.

    cad run jigs/hip_roll_foot_board.py

The soles are wedges: the bottom face of `sole_left_native` is tilted 4.84 degrees about the foot's
long axis, so a sole lies flat when its leg is rolled 4.84 degrees inward (the home pose has 5).
With both soles held flat on this plate and the servos limp, each hip roll joint is at 4.84 degrees
inward minus the trunk's roll, which the trunk IMU gives. The plate does not hold the trunk.

Frame, mm: x forward, y to the robot's left, z up; z = 0 is the plate's top face, where the soles
rest; the origin is under the trunk between the feet. The foot outline comes from the robot model
in that pose (`reference/feet_flat_pose.py`); `check_hip_roll_foot_board.py` compares the two.

Each foot sits inside a low fence (heel, both sides, a toe stop) with 1.5 mm of room, so the fence
places the foot without pushing the leg. One strap per foot (rubber band, cable tie or hook-and-loop
tie, up to 14 mm wide) goes over the toe ahead of the ankle servo, down through the slot between
the feet and the notch in the outer edge, and under the plate in a groove, so the plate still lies
flat on the table. Print flat as modelled, no supports.
"""
import build123d as bd
from cad import model

THICKNESS = 3.0
X_BACK, X_FRONT = -26.0, 54.0
HALF_WIDTH = 93.0
CORNER = 5.0

# A sole in the flat pose: everything of either foot below 3 mm, just above the fence's top
# (the model; the sole's edge is rounded, so the foot is wider further up).
SOLE_X = (-19.2, 46.9)
SOLE_Y = (30.9, 81.2)
ROOM = 1.5
FENCE_HEIGHT = 2.5
FENCE_WIDTH = 2.0
FENCE_X_END = 27.0  # the side fences stop before the strap
TOE_STOP_Y = (46.0, 66.0)

# Strap over the toe: the foot is at most 25 mm high there and 28.3..83.9 mm wide.
STRAP_X = (30.0, 46.0)
SLOT_Y = (22.5, 27.5)
NOTCH_Y = 85.0
GROOVE_DEPTH = 1.4

ARROW_DEPTH = 0.6
COLOR = "#222426"


def box(x0, x1, y0, y1, z0, z1):
    y0, y1 = sorted((y0, y1))
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def foot_board_body():
    plate = box(X_BACK, X_FRONT, -HALF_WIDTH, HALF_WIDTH, -THICKNESS, 0)
    plate = bd.fillet(plate.edges().filter_by(bd.Axis.Z), CORNER)

    heel = SOLE_X[0] - ROOM
    toe = SOLE_X[1] + ROOM
    inner = SOLE_Y[0] - ROOM
    outer = SOLE_Y[1] + ROOM
    for side in (1, -1):
        y = lambda v: side * v
        fences = [
            box(heel - FENCE_WIDTH, heel, y(inner - FENCE_WIDTH), y(outer + FENCE_WIDTH), 0, FENCE_HEIGHT),
            box(heel, FENCE_X_END, y(inner - FENCE_WIDTH), y(inner), 0, FENCE_HEIGHT),
            box(heel, FENCE_X_END, y(outer), y(outer + FENCE_WIDTH), 0, FENCE_HEIGHT),
            box(toe, toe + FENCE_WIDTH, y(TOE_STOP_Y[0]), y(TOE_STOP_Y[1]), 0, FENCE_HEIGHT),
        ]
        for fence in fences:
            plate = plate + fence
        # Strap path: slot between the feet, notch in the outer edge, groove under the plate.
        plate = plate - box(*STRAP_X, y(SLOT_Y[0]), y(SLOT_Y[1]), -THICKNESS, 0)
        plate = plate - box(*STRAP_X, y(NOTCH_Y), y(HALF_WIDTH + 1), -THICKNESS, 0)
        plate = plate - box(*STRAP_X, y(SLOT_Y[1]), y(NOTCH_Y), -THICKNESS, -THICKNESS + GROOVE_DEPTH)

    # An arrow sunk into the top between the feet: it points to the robot's front.
    arrow = bd.Polygon((2, -9), (16, 0), (2, 9), align=None)
    plate = plate - bd.Pos(0, 0, -ARROW_DEPTH) * bd.extrude(arrow, ARROW_DEPTH, dir=(0, 0, 1))

    size = plate.bounding_box().size
    assert abs(size.X - (X_FRONT - X_BACK)) < 1e-6 and abs(size.Y - 2 * HALF_WIDTH) < 1e-6, size
    assert abs(size.Z - (THICKNESS + FENCE_HEIGHT)) < 1e-6, size
    assert len(plate.solids()) == 1, "the board is one solid"
    assert len(plate.solids()[0].shells()) == 1, "no closed hollow inside the board"
    plate.label = "hip_roll_foot_board"
    plate.color = COLOR
    return plate


@model
def hip_roll_foot_board():
    return foot_board_body()


if __name__ == "__main__":
    hip_roll_foot_board()
