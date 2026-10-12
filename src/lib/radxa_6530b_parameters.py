"""Official outer size with explicit approximations for the 6530B cooler.

Radxa publishes 65 x 30 x 11.5 mm and a 16 x 16 x 1.5 mm thermal pad.
The detailed fan/fin/notch/foot dimensions are a review representation.
The mounting plane is the component-side PCB plane in the imported model.
"""
from cadgen import build123d as bd

WIDTH = 65.0
SHORT_SIDE = 30.0
HEIGHT = 11.5
HOLE_GRID = (57.8, 22.9)
HEATSINK_AT = ((-12.13, -88.065, 302.5), ("X", -90.0))
# Imported SoC top is 3.5 mm above this PCB plane. Add the uncompressed
# 1.5 mm pad; the base thickness and fin/fan divisions are approximate.
BASE_BOTTOM = 5.0
BASE_THICKNESS = 0.8
FAN_CENTRE = (20.0, -2.0)
FAN_OUTER = 21.5
FAN_INLET_RADIUS = 8.5


def outer_envelope():
    shape = bd.Box(WIDTH, HEIGHT, SHORT_SIDE).moved(
        bd.Location((-12.13, -88.065 + HEIGHT/2, 302.5)))
    shape.label = "6530b_official_outer_envelope"
    shape.color = bd.Color(0.15, 0.65, 0.8, 0.12)
    return shape


def fan_inlet_allowance():
    # A 2 mm clear approach to the inlet; this is a spatial review margin,
    # not a thermal simulation or a manufacturer airflow requirement.
    shape = bd.Cylinder(FAN_INLET_RADIUS, 2.0, align=(bd.Align.CENTER,
        bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location(
        (7.87, -76.565, 304.5), (-90, 0, 0)))
    shape.label = "6530b_fan_inlet_clearance_2mm"
    shape.color = bd.Color(0.15, 0.65, 0.8, 0.25)
    return shape
