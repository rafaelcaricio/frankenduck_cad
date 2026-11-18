"""Local, subtractive finishing for the body-shell tips beside the battery.

Input is an installed world-coordinate shell. `lib.lipo_shoulders.body_shell`
applies it to both default body shells.
"""
import math
import numpy as np
import cadquery as cq
from scipy.optimize import brentq
from cadgen import build123d as b

CORNER_RADIUS = 1.5


def rear_corner_cutter(shell, side):
    """Tangent round plus a smooth runout, spanning the entire 1 mm wall.

    Solve the round against the existing outer relief curve, not a traced
    outline. A curved runout clears the inner edge without the tiny transverse
    end face left by stopping the cut at the outer-face tangency.
    """
    assert side in ('left', 'right')
    source = cq.Shape.cast(shell.wrapped)
    if side == 'left':
        source = source.mirror('YZ')
    tips = [e for e in source.Edges()
            if 24 < abs(e.Center().x) < 29.5 and 55 < e.Center().y < 58
            and 127.4 < e.BoundingBox().zmin < 127.6
            and 128 < e.BoundingBox().zmax < 128.3 and e.Length() < 2.0]
    assert len(tips) == 1, ('lower rear tip', side, len(tips))
    tip = min((tips[0].startPoint(), tips[0].endPoint()), key=lambda p: p.z)
    borders = [e for e in source.Edges()
               if ((e.startPoint()-tip).Length < 1e-4
                   or (e.endPoint()-tip).Length < 1e-4)
               and e.BoundingBox().zmax > 130]
    assert len(borders) == 1
    edge = borders[0]
    reverse = (edge.endPoint()-tip).Length < 1e-4

    def point(t):
        p = edge.positionAt(1-t if reverse else t)
        return np.array([p.x, p.z])

    def tangent(t):
        v = point(min(1, t+1e-5))-point(max(0, t-1e-5))
        return v/np.linalg.norm(v)

    radius, z0 = CORNER_RADIUS, tip.z
    u = brentq(lambda t: point(t)[1]+radius*tangent(t)[0]-(z0+radius), 0, 1)
    end_arc, direction = point(u), tangent(u)
    inward = np.array([-direction[1], direction[0]])
    center = end_arc+radius*inward
    start = np.array([center[0], z0])
    angle = math.atan2(*(end_arc-center)[::-1])
    mid_angle = (-math.pi/2+angle)/2
    mid = center+radius*np.array([math.cos(mid_angle), math.sin(mid_angle)])
    end = end_arc+2*direction-2*inward
    far = end-5*inward
    plane = cq.Plane(origin=(0, 65, 0), xDir=(1, 0, 0), normal=(0, -1, 0))
    tool = (cq.Workplane(plane).moveTo(*start)
            .threePointArc(tuple(mid), tuple(end_arc))
            .bezier([tuple(end_arc+.7*direction),
                     tuple(end_arc+1.4*direction-.6*inward), tuple(end)],
                    includeCurrent=True)
            .lineTo(*far).lineTo(far[0], z0-5).lineTo(start[0], z0-5)
            .close().extrude(15).val())
    return tool.mirror('YZ') if side == 'left' else tool


def round_rear_corner(shell, side):
    source = cq.Shape.cast(shell.wrapped)
    result = source.cut(rear_corner_cutter(shell, side)).clean()
    assert result.isValid() and len(result.Solids()) == 1
    part = b.Solid(result.Solids()[0].wrapped)
    part.label, part.color = shell.label, shell.color
    return part
