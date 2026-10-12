# yaw2roll_native: bracket from the hip-yaw servo horn (top plate) to the hip-roll servo body
# (vertical plate) of the Frankenduck leg (yaw2roll.001 left / yaw2roll.002 right).
# Native rebuild of the Blender mesh, robot world frame, mm, Z up. Dimensions measured from
# the mesh (sections, circle fits). The right part is the exact mirror about Y = 0.
import cadquery as cq

SIDE = 1  # +1 left (yaw2roll.001, +Y), -1 right (yaw2roll.002, mirror about Y = 0)
x0 = -7.825  # mm Vertical plate outer face (-X)
x1 = -4.825  # mm Vertical plate inner face
x2 = 20.875  # mm Top plate +X end
y0 = 8.1  # mm Part Y minimum
y1 = 39.15  # mm Part Y maximum
z0 = 63.175  # mm Vertical plate bottom
zt0 = 116.25  # mm Top plate underside
zt1 = 120.25  # mm Top plate top face (yaw horn face)
plate_cr = 4.05  # mm Vertical plate bottom corner radius
wall_t = 1.82  # mm Side wall thickness
wall_z0 = 65.62  # mm Side wall slant start Z at the vertical plate
wall_z1 = 113.97  # mm Side wall slant end Z at the top plate end
top_r = 0.6  # mm Top plate edge round
# yaw horn (top plate, axis Z)
yaw_x = 8.1  # mm Yaw horn axis X
axis_y = 23.625  # mm Horn axis Y (both servos)
bolt_r = 7.0  # mm Horn screw bolt-circle radius
hole_r = 1.7  # mm M3 clearance radius
bore_r = 3.2  # mm Top plate centre bore radius
# roll servo body (vertical plate, axis X)
roll_z = 100.375  # mm Roll horn axis Z
win_r = 9.65  # mm Horn window radius
m2_r = 1.1  # mm Servo body screw radius
m2_dy = 10.25  # mm Servo screw half spacing (Y)
m2_z = (92.075, 67.625)  # mm Servo screw rows Z
slot_y = (13.125, 34.125)  # mm Slot Y extent
slot_z = (81.375, 89.625)  # mm Slot Z extent
slot_r = 2.5  # mm Slot corner radius
pk_y = (14.57, 32.68)  # mm Pocket Y extent
pk_z = (70.87, 81.5)  # mm Pocket Z extent (runs into the slot)
pk_x = -6.72  # mm Pocket floor
isl_y = (17.65, 29.6)  # mm Pocket floor island Y extent
isl_z = (72.9, 78.875)  # mm Pocket floor island Z extent
isl_x = -6.57  # mm Island top

top = cq.Workplane("XY").box(x2 - x0, y1 - y0, zt1 - zt0, centered=False).translate((x0, y0, zt0))
plate = (cq.Workplane("YZ", origin=(x0, 0, 0)).center((y0 + y1) / 2, (z0 + zt1) / 2)
         .rect(y1 - y0, zt1 - z0).extrude(x1 - x0).edges("|X and <Z").fillet(plate_cr))
walls = None
for wy in (y0, y1 - wall_t):
    w = (cq.Workplane("XZ", origin=(0, wy + wall_t, 0)).polyline([(x1, wall_z0), (x2, wall_z1), (x2, zt1), (x1, zt1)])
         .close().extrude(wall_t))
    walls = w if walls is None else walls.union(w)
body = top.union(plate).union(walls)
body = body.edges(cq.selectors.BoxSelector((x2 - 0.1, y0 - 1, wall_z1 + 0.1), (x2 + 0.1, y1 + 1, zt1 - 0.5))).fillet(top_r)
body = body.faces(">Z").edges().fillet(top_r)

# top plate: yaw horn bore and screws
pts = [(yaw_x + bolt_r, axis_y), (yaw_x - bolt_r, axis_y), (yaw_x, axis_y + bolt_r), (yaw_x, axis_y - bolt_r)]
body = body.cut(cq.Workplane("XY", origin=(0, 0, zt0 - 1)).center(yaw_x, axis_y).circle(bore_r).extrude(6))
body = body.cut(cq.Workplane("XY", origin=(0, 0, zt0 - 1)).pushPoints(pts).circle(hole_r).extrude(6))

# vertical plate: horn window, slot, servo screws, pocket with floor island
def yz(origin_x):
    return cq.Workplane("YZ", origin=(origin_x, 0, 0))


body = body.cut(yz(x0 - 1).center(axis_y, roll_z).circle(win_r).extrude(5))
body = body.cut(yz(x0 - 1).center(sum(slot_y) / 2, sum(slot_z) / 2)
                .rect(slot_y[1] - slot_y[0], slot_z[1] - slot_z[0]).extrude(5).edges("|X").fillet(slot_r))
body = body.cut(yz(x0 - 1).pushPoints([(axis_y + s * m2_dy, z) for s in (-1, 1) for z in m2_z]).circle(m2_r).extrude(5))
body = body.cut(yz(pk_x).center(sum(pk_y) / 2, sum(pk_z) / 2).rect(pk_y[1] - pk_y[0], pk_z[1] - pk_z[0]).extrude(x1 - pk_x + 1))
body = body.union(yz(pk_x).center(sum(isl_y) / 2, sum(isl_z) / 2).rect(isl_y[1] - isl_y[0], isl_z[1] - isl_z[0]).extrude(isl_x - pk_x))

if SIDE < 0:
    body = body.mirror("XZ")
from source_api import Model

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model = Model()
model.add("printed_body", body)
model.material("pla", color="#1d1d1f")
result = model.result()

