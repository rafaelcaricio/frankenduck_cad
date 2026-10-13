"""Jumper-plug and wire allowances in the assembly's millimetre world frame.

These are review and fit-check volumes, not manufactured parts. Connector
depth starts at the bare PCB seating plane and includes the existing header.
Wires turn upwards into the second volume; exact jumper leads are not measured.
"""
from cadgen import build123d as bd

PCB_SEAT_Y = -89.015
PLUG_SIZE = (53.0, 23.0, 5.5)
PLUG_CENTRE = (-12.13, -77.515, 290.75)
CABLE_SIZE = (53.0, 6.0, 18.0)
CABLE_CENTRE = (-12.13, -69.015, 302.5)

# The imported PCB does not include the underside button. Position is inferred
# from Radxa's underside layout, not measured from this board. Reserve a broad
# 6 x 6 x 2 mm button envelope and at least 1 mm outward free space around it.
# https://docs.radxa.com/en/cubie/a7z/hardware-use/hardware-info
BUTTON_CENTRE = (-26.75, PCB_SEAT_Y-1.0, 315.5)
BUTTON_SIZE = (6.0, 2.0, 6.0)
BUTTON_CLEARANCE_SIZE = (8.0, 3.0, 8.0)
BUTTON_CLEARANCE_CENTRE = (-26.75, PCB_SEAT_Y-1.5, 315.5)

def button_clearance():
    shape=bd.Box(*BUTTON_CLEARANCE_SIZE).moved(bd.Location(BUTTON_CLEARANCE_CENTRE))
    shape.label='underside_button_clearance_approximate'
    shape.color=bd.Color(1.0,.55,.08,.5)
    return shape


def clearance_shapes():
    plug = bd.Box(*PLUG_SIZE).moved(bd.Location(PLUG_CENTRE))
    plug.label = "gpio_jumper_plug_allowance"
    plug.color = bd.Color(1.0, 0.60, 0.08, 0.35)
    cable = bd.Box(*CABLE_SIZE).moved(bd.Location(CABLE_CENTRE))
    cable.label = "gpio_upward_cable_allowance"
    cable.color = bd.Color(0.65, 0.25, 0.95, 0.35)
    return plug, cable
