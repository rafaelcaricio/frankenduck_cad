# ankle_bearing_outer_keeper_<side>_native: printed ring that clamps the ankle F688 flanged bearing into the ankle eye (Frankenduck).
# Native rebuild of mesh ankle_bearing_outer_keeper_<side>, robot world frame, mm, Z up. Left = +Y, right = -Y (exact mirror about Y = 0).
# Disk on the bearing axis (along Y) with a Ø13.8 opening, two Ø2.2 screw holes, a 0.2 mm lead-in chamfer on the eye side
# and a 0.1 mm tapered crush ring on the outer face.
import cadquery as cq
from source_api import Model

SIDE = 1  # side switch: 1 = left (+Y), -1 = right (-Y); the only line that differs between the two documents

axis_x = -42.899  # mm Bearing axis X
axis_z = 3.975  # mm Bearing axis Z
y_inner = 38.905  # mm |Y| of the face against the ankle eye
y_outer = 40.505  # mm |Y| of the outer face (ring is 1.6 thick)
outer_r = 13.15  # mm Outer radius (Ø26.3)
chamfer = 0.2  # mm 45 degree chamfer on the inner-face outer edge
bore_r = 6.9  # mm Central opening radius (Ø13.8, clears the Ø8 shaft side of the F688)
crush_in_r = 8.7  # mm Crush ring inner radius at its root
crush_out_r = 9.05  # mm Crush ring outer radius at its root
crush_taper = 0.05  # mm Radial taper of each crush ring flank over its height
crush_h = 0.1  # mm Crush ring height above the outer face
screw_offset = 11.0  # mm Screw hole offset from the axis along X
screw_r = 1.1  # mm Screw hole radius (Ø2.2)


def ring_profile():
    # (radius, |Y|) half section, revolved about the bearing axis
    pts = [
        (bore_r, y_inner), (outer_r - chamfer, y_inner), (outer_r, y_inner + chamfer), (outer_r, y_outer),
        (crush_out_r, y_outer), (crush_out_r - crush_taper, y_outer + crush_h),
        (crush_in_r + crush_taper, y_outer + crush_h), (crush_in_r, y_outer), (bore_r, y_outer),
    ]
    vs = [cq.Vector(axis_x + r, SIDE * y, axis_z) for r, y in pts]
    face = cq.Face.makeFromWires(cq.Wire.makePolygon(vs, close=True))
    return cq.Solid.revolve(face, 360, cq.Vector(axis_x, 0, axis_z), cq.Vector(axis_x, 1, axis_z))


model = Model()
model.add("ring", ring_profile())
for n, dx in (("a", screw_offset), ("b", -screw_offset)):
    model.cylinder(f"screw_hole_{n}", radius=screw_r, height=y_outer - y_inner + 2,
                   at=(axis_x + dx, SIDE * (y_inner - 1), axis_z), direction=(0, SIDE, 0), op="cut")

# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#f96c0a")
result = model.result()

