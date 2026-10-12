"""Jumper-plug and wire allowances in the assembly's millimetre world frame.

These are review and fit-check volumes, not manufactured parts. Connector
depth starts at the bare PCB seating plane and includes the existing header.
Wires turn upwards into the second volume; exact jumper leads are not measured.
"""
from cadgen import build123d as bd

PCB_SEAT_Y = -89.815
PLUG_SIZE = (53.0, 23.0, 5.5)
PLUG_CENTRE = (-12.13, -78.315, 290.75)
CABLE_SIZE = (53.0, 6.0, 18.0)
CABLE_CENTRE = (-12.13, -69.815, 302.5)


def clearance_shapes():
    plug = bd.Box(*PLUG_SIZE).moved(bd.Location(PLUG_CENTRE))
    plug.label = "gpio_jumper_plug_allowance"
    plug.color = bd.Color(1.0, 0.60, 0.08, 0.35)
    cable = bd.Box(*CABLE_SIZE).moved(bd.Location(CABLE_CENTRE))
    cable.label = "gpio_upward_cable_allowance"
    cable.color = bd.Color(0.65, 0.25, 0.95, 0.35)
    return plug, cable
