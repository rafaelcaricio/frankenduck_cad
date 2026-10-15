"""
PARKSIDE PAPK 12 A4 - 12 V / 2 Ah stick battery (X12V TEAM), exterior visual model.
Millimetres. Origin at the centre of the underside (red cap), Z up along the stick.
-Y is the front with the X12V TEAM nameplate, the release buttons face +X and -X,
and the T contact sits on the flat back of the stick (+Y).

CONFIRMED (manufacturer values, not caliper measurements):
Lidl product 100406214: about 82 x 52 x 48 mm, product mass about 185 g.
https://www.lidl.nl/p/parkside-accu-12-v-2-ah/p100406214
Manual (PAPK 12 A4 technical data): 3 Li-ion cells, 12 V, 2.0 Ah, 24 Wh.
https://www.manualslib.de/manual/1416739/Parkside-Papk-12-A4.html

PHOTO-DERIVED: every other dimension, the colours and the printed artwork come from six
public product photos of this model (front nameplate, both buttons, three-quarter, side, top
and underside views; eBay listings 157018648802, 366558976475, 355353292849 and 355734851915).
The cameras were fitted together with the listed 82 mm height (stick top 80.5 plus the 1.5 mm
feet) as scale, and the outlines were fitted to the photo silhouettes. Expect about 0.5 mm on
the cap, collar and stick outlines and about 1 mm elsewhere. On that scale the plan comes out
at about 45.2 x 51.5 mm, not the listed 52 x 48 mm; the model follows the photos.
The photos put the stick about 1.2 mm towards -X in the collar and the back wall about 16 mm
below the rim. Three third-party X12 prints centre the stick and put the back wall 17.0 mm
below the rim, and the model follows them on both points; check them with a caliper before
relying on them for a mating part.
The nameplate artwork (X, 12V, TEAM and ///) is traced from the front photo, unwrapped round
the curved collar; PARKSIDE is set in DejaVu Sans Bold at the traced letter positions. The
nameplate's height on the collar varies by about 1.5 mm either way between the photos (the
renders put it higher); label_z follows its gap to the red cap in the photographs. The
underside texts are DejaVu Sans Bold fitted to the measured text boxes, and its small X12V
TEAM plate reuses the traced nameplate shapes fitted to the measured boxes. The faint brand
line on that plate is unreadable in the photo and is drawn as the nameplate's /// PARKSIDE.
This is a visual and packaging model, not a mating drawing: the contacts and latches are
drawn as they look, not to the X12V interface tolerances. The latch teeth are hooks: a notch
under the catch takes the tool's lip, and a lead-in ramp tops each tooth, as in the side photo
and two third-party replicas, which also put the catch 3.0 mm above the rim. Third-party X12
prints were used as fit references; docs/design.md lists them and where they disagree.
The bodies are solid, so the CAD mass is not the battery mass: use reference_product_mass_g.
"""
import bisect
import math
from pathlib import Path
from cadgen import step

listed_size = (82.0, 52.0, 48.0)  # mm Manufacturer height x width x depth; the photos give a 45.2 x 51.5 plan
reference_product_mass_g = 185.0  # g Manufacturer product mass; not CAD mass
# ---------------------------------------------------------------- red cap (base)
cap_radius_terms = (23.43, -2.888, 1.744, 0.383, 0.3, -0.193, 0.249, 0.011)  # mm cos(k*t) terms of the plan outline radius; t from -Y towards +X
cap_centre_y = -2.69  # mm Y of the plan outline's polar centre
cap_side_top = 9.0  # mm Top of the red cap on the sides and back
cap_front_top = 13.7  # mm Top of the raised front part of the red cap
cap_step = ((-16.8, 9.0), (-19.4, 13.7))  # mm (Y, Z) bottom and top of the sloped step to the raised front part
cap_step_round = 1.5  # mm Radius at the top and bottom of the step
cap_bottom_round = 1.2  # mm Radius round the underside edge
cap_top_round = 0.6  # mm Radius round the cap's top outer edge
# ---------------------------------------------------------------- black case and collar
collar_inset = 0.7  # mm Collar outer wall inset from the red cap outline
collar_wall = 2.2  # mm Collar wall thickness at the rim
stick_clearance_wall = 1.0  # mm Minimum collar wall outside the stick
cup_floor = 14.0  # mm Floor of the gap between the collar and the stick
rim_height = 36.5  # mm Collar rim on the front and both latch arms
back_wall_height = 19.5  # mm Lower collar wall behind the stick: 17.0 below the rim, as in the X12 references (photos about 20.6)
rim_drop = (7.0, 13.0)  # mm Y where the side rim starts to slope down, and where it meets the back wall
slit_y = (-14.1, -12.0)  # mm Vertical slit in front of each latch arm
slit_bottom = 14.0  # mm Bottom of the slit
# ---------------------------------------------------------------- stick
stick_lobes = ((-9.21, 7.88), (9.21, 7.88), (-1.85, -9.15))  # mm Plan centres of the stick's three rounded corners: back -X, back +X, front; centred in the collar
stick_radius = 10.56  # mm Corner radius of the stick plan outline
stick_top = 80.5  # mm Top face of the stick
stick_top_round = 1.0  # mm Radius round the stick's top edge
channel_depth = 2.2  # mm Depth of the contact channels in the stick sides
channel_width = 9.0  # mm Width of the contact channels
channel_s = {'plus': 8.2, 'minus': 11.2}  # mm Channel centre along the front side, from the front corner's tangent point
t_channel_x = 0.18  # mm X of the T contact channel centre on the flat back
t_channel_width = 8.0  # mm Width of the T contact channel
plate_width = 7.2  # mm Width of the contact plates
plate_z = (63.1, 79.9)  # mm Bottom of the contact plates and top of their bent-over tabs
plate_face_depth = 1.8  # mm Contact plate face below the stick side
plate_thickness = 0.4  # mm Contact plate thickness
plate_tab = 1.6  # mm Length of the plate tab bent over the stick top
pin_centre = (-1.27, -8.3)  # mm Centre of the ID contact pin on the top
pin_diameter = 5.4  # mm ID contact pin diameter
pin_recess_diameter = 6.9  # mm Recess round the ID pin
# ---------------------------------------------------------------- moulded marks on the stick top
mark_depth = 0.2  # mm Depth of the moulded marks
mark_t = ((-1.77, 1.38, 12.0, 12.77), (-0.82, 0.28, 9.65, 12.0))  # mm X and Y extents of the T's bar and stem, in front of the T contact
mark_plus = (-8.84, -2.87, 3.3, 1.1, 19.5)  # mm Centre X and Y, span and arm width; rotation in degrees
mark_minus = (6.78, -2.9, 3.8, 1.0, 55.0)  # mm Centre X and Y, length and width; rotation in degrees
mark_id = (-3.01, 0.18, -15.45, -12.88)  # mm X and Y extents of the ID legend in front of the pin, read from the back
mark_ring = (-0.74, 2.2, 3.6, 0.45)  # mm Centre X and Y, outer diameter and groove width of the ring mark
# ---------------------------------------------------------------- latch arms and buttons
button_outline = [(-7.50, 32.52), (3.75, 33.02), (8.21, 19.80), (-7.09, 19.25)]  # mm (Y, Z) release button corners, both sides
button_proud = 0.8  # mm Button face proud of the arm
button_gap = 0.9  # mm Gap round the button in its pocket
button_round = 1.5  # mm Button corner radius
button_grooves = (((-4.3, 28.6), (-2.8, 23.8)), ((-2.1, 28.6), (-0.45, 23.7)), ((0.05, 28.6), (1.8, 23.65)))  # mm (Y, Z) ends of the /// grooves, both buttons
button_groove = (0.45, 0.3)  # mm Width and depth of the button grooves
tooth_y = (-9.3, -0.4)  # mm Latch tooth extent along Y on each arm
tooth_top = 43.0  # mm Top of the latch teeth
tooth_depth = (0.3, 3.3)  # mm Latch tooth inner and outer faces, measured in from the collar outline
tooth_catch = 39.5  # mm Underside of the latch hook, 3.0 above the rim; the tool's lip sits in the notch below it
tooth_notch = 1.0  # mm Depth of that notch, in from the tooth's outer face
tooth_ramp = (2.4, 2.0)  # mm Lead-in ramp on the hook's top outer edge: height down the outer face and depth in across the top
# ---------------------------------------------------------------- underside
bottom_holes = ((10.20, -11.82), (13.34, -5.72), (14.80, 0.29), (14.79, 6.16), (12.76, 11.91))  # mm (|X|, Y) of the round holes, on both sides
bottom_hole = (2.0, 3.0)  # mm Diameter and depth of the underside holes
screw_y = (-21.0, 19.0)  # mm Y of the two underside screws, on X = 0
screw_counterbore = (6.0, 2.4)  # mm Diameter and depth of the screw counterbores
screw_head = (4.3, 1.2)  # mm Screw head diameter and height; its face sits 1.2 inside the underside
screw_recess = (2.6, 0.6, 0.8)  # mm Cross recess length, slot width and depth
feet = (8.5, 15.2)  # mm (|X|, Y) of the two red feet
foot = (4.0, 1.5)  # mm Foot base diameter and height below the underside
sticker_radius_terms = (13.128, -1.718, 1.777, 0.437, 0.028)  # mm cos(k*t) terms of the egg-shaped underside sticker; t from -Y towards +X
sticker_centre_y = -0.47  # mm Y of the sticker outline's polar centre
# ---------------------------------------------------------------- artwork
artwork_thickness = 0.12  # mm Underside sticker, proud of the cap
print_thickness = 0.05  # mm Printed legends, proud of the face they are printed on
label_thickness = 0.3  # mm Front nameplate, proud of the collar wall
label_s = -15.9  # mm Nameplate left end, as arc length round the collar wall from its front-most point (+ towards +X)
label_z = 21.6  # mm Bottom of the nameplate

# The nameplate is drawn flat in its own frame: u along the collar wall from the plate's left
# end (mm of arc length), v up from its bottom edge. Each shape is [outline, hole, ...].
label_outline = [(1.74, 0.0), (27.25, 0.0), (27.25, 6.85), (-0.35, 6.85)]  # mm (u, v) corners; the left end leans like the italic artwork
label_round = 0.7  # mm Nameplate corner radius
label_tab = [(27.0, 1.33), (28.5, 1.33), (28.5, 6.09), (27.0, 6.09)]  # mm (u, v) corners of the tab at the right end
label_tab_round = 0.35  # mm Tab corner radius
label_red = [  # X, ///, TEAM
    [[(10.45, 6.07), (8.96, 6.10), (8.42, 6.05), (7.94, 5.70), (7.06, 4.90), (6.74, 4.71), (6.39, 4.81), (6.17, 4.98), (6.02, 4.98), (5.86, 5.27), (5.61, 5.47), (5.45, 5.50), (5.08, 5.86), (4.77, 6.02), (4.42, 6.06), (2.13, 6.01), (1.78, 5.92), (1.68, 5.73), (1.68, 5.51), (1.74, 5.41), (2.40, 5.02), (3.00, 4.78), (3.06, 4.69), (3.47, 4.49), (3.65, 4.32), (3.78, 4.32), (4.41, 3.90), (4.57, 3.87), (4.81, 3.57), (4.94, 3.50), (4.87, 3.31), (4.45, 3.10), (3.49, 2.43), (3.30, 2.36), (2.68, 1.93), (2.61, 1.50), (2.64, 1.20), (2.73, 1.05), (2.95, 0.91), (4.41, 0.91), (4.76, 0.97), (5.18, 1.22), (6.54, 2.36), (6.85, 2.23), (7.23, 1.90), (7.44, 1.65), (7.79, 1.39), (7.97, 1.10), (8.19, 0.90), (8.60, 0.92), (9.49, 0.85), (10.19, 0.96), (10.29, 1.06), (10.39, 1.33), (10.33, 1.56), (10.02, 1.91), (9.86, 2.02), (9.77, 2.17), (9.64, 2.24), (9.55, 2.40), (9.36, 2.50), (8.68, 3.15), (8.58, 3.15), (8.40, 3.38), (8.15, 3.54), (8.12, 3.70), (8.80, 4.35), (9.02, 4.48), (9.61, 5.10), (9.70, 5.13)]],
    [[(13.14, 5.82), (12.89, 5.80), (12.73, 5.61), (12.49, 4.72), (12.49, 4.60), (12.58, 4.47), (12.83, 4.59), (13.17, 5.25), (13.24, 5.76)]],
    [[(13.59, 5.85), (13.46, 5.72), (13.16, 4.74), (13.18, 4.55), (13.31, 4.49), (13.53, 4.58), (13.60, 4.83), (13.70, 4.95), (13.70, 5.14), (13.90, 5.55), (13.91, 5.81)]],
    [[(14.54, 5.76), (14.29, 5.77), (14.13, 5.65), (13.99, 5.17), (13.83, 4.86), (13.79, 4.57), (13.85, 4.50), (14.04, 4.50), (14.23, 4.66), (14.30, 5.01), (14.50, 5.29), (14.50, 5.51), (14.57, 5.64)]],
    [[(26.93, 1.05), (26.92, 2.22), (26.80, 2.50), (25.94, 2.52), (25.84, 2.40), (25.77, 2.09), (25.58, 1.83), (25.21, 2.50), (24.42, 2.58), (24.29, 2.52), (24.26, 2.43), (24.28, 1.95), (24.21, 1.73), (24.05, 1.86), (23.72, 2.53), (23.18, 2.58), (23.02, 2.49), (22.55, 1.38), (21.73, 1.40), (21.66, 1.47), (21.70, 1.62), (21.80, 1.69), (22.46, 1.70), (22.53, 1.76), (22.44, 2.02), (21.74, 2.04), (21.65, 2.10), (21.74, 2.23), (22.51, 2.24), (22.51, 2.56), (20.77, 2.63), (19.50, 2.55), (19.27, 2.47), (19.36, 2.17), (19.55, 2.17), (19.58, 2.10), (19.74, 2.17), (19.83, 2.04), (19.83, 1.91), (19.89, 1.85), (19.88, 1.21), (20.00, 0.92), (20.35, 0.95), (20.51, 1.08), (20.50, 2.12), (20.69, 2.21), (20.92, 2.21), (21.01, 2.11), (21.05, 1.12), (21.12, 1.00), (21.24, 0.96), (22.96, 0.99), (23.15, 1.21), (23.34, 1.28), (23.69, 1.20), (23.88, 1.01), (24.86, 1.02), (24.93, 1.11), (24.91, 1.66), (24.97, 1.75), (25.22, 1.30), (25.22, 1.17), (25.37, 1.01), (25.85, 0.97), (25.98, 1.12), (25.98, 1.29), (26.11, 1.48), (26.15, 1.75), (26.25, 1.89), (26.31, 1.79), (26.32, 1.06), (26.48, 0.95), (26.83, 0.95)], [(23.39, 1.94), (23.51, 1.84), (23.54, 1.71), (23.32, 1.62), (23.26, 1.78)]],
]
label_black = [  # 1, 2 and V
    [[(13.30, 4.17), (12.32, 4.19), (12.16, 4.07), (10.84, 1.49), (10.67, 1.08), (10.70, 0.98), (11.62, 0.96), (11.78, 0.99), (11.91, 1.11), (12.24, 1.97), (13.00, 3.45)]],
    [[(17.30, 4.09), (17.21, 4.19), (17.05, 4.22), (13.87, 4.16), (13.61, 3.62), (13.64, 3.52), (16.05, 3.54), (16.27, 3.45), (16.27, 3.22), (16.11, 3.13), (13.73, 3.14), (13.47, 3.06), (13.28, 2.87), (12.98, 2.11), (12.85, 1.95), (12.75, 1.68), (12.74, 1.35), (12.80, 1.19), (13.05, 0.97), (16.29, 0.93), (16.39, 1.06), (16.62, 1.85), (14.08, 1.84), (13.99, 1.93), (13.99, 2.10), (14.06, 2.25), (14.29, 2.44), (16.86, 2.45), (17.39, 3.74)]],
    [[(21.15, 4.24), (20.07, 4.23), (18.57, 2.16), (18.50, 2.16), (18.44, 2.29), (19.14, 3.96), (19.18, 4.21), (18.07, 4.20), (17.40, 2.66), (17.03, 1.55), (16.97, 1.50), (16.83, 0.92), (18.73, 0.91), (19.02, 0.98), (19.61, 1.98), (19.61, 2.10), (19.23, 2.11), (19.18, 2.59), (19.31, 2.69), (19.56, 2.65), (20.10, 2.70)]],
]
label_brand = list(zip("PARKSIDE", [(15.39, 16.44), (16.42, 17.54), (17.59, 18.68), (18.77, 19.82), (19.93, 20.79), (20.96, 21.28), (21.50, 22.37), (22.58, 23.41)]))  # mm u extents of each letter
label_brand_v = (4.61, 5.97)  # mm Baseline and cap top of PARKSIDE

# The underside artwork is drawn in its reading frame seen from below: x = X, y = -Y (mm).
# Boxes are (x0, x1, y0, y1); texts are stretched to fill their measured boxes.
bottom_texts = (("2.0 Ah", (-4.27, 4.57, 5.68, 8.35)), ("PARKSIDE", (-4.08, 7.55, 1.30, 3.43)), ("PAPK 12 A4", (-6.23, 6.80, -2.82, -0.65)))
bottom_frame = ((-8.12, 8.65, 0.55, 4.05), 0.45)  # mm White frame round /// PARKSIDE and its stroke width
bottom_block = (-7.70, -4.40, 1.00, 3.60)  # mm Red block at the left of the frame
bottom_slashes = (-7.65, -4.85, 1.05, 3.55)  # mm White /// in the red block
pill = ((-8.57, 8.60, -9.35, -4.55), 0.6)  # mm Light X12V TEAM plate and its corner radius
pill_nub = ((8.55, 9.10, -8.40, -5.35), 0.2)  # mm Battery-symbol nub at the plate's right end
pill_x = (-8.00, -4.40, -8.92, -5.08)  # mm Red X
pill_12v = (-4.25, 2.98, -8.67, -6.50)  # mm Black 12V
pill_team = (1.67, 8.20, -8.82, -7.55)  # mm Red TEAM
pill_brand = (-3.25, 3.45, -6.15, -5.60)  # mm Faint /// PARKSIDE line


def _single_component(model):
    """Keep all solids in one STEP leaf, with body colours carried by faces."""
    from cadgen import build123d as bd
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_IndexedMapOfShape

    battery = bd.Compound(model.result().wrapped, label='parkside_papk_12_a4')
    face_map = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(battery.wrapped, TopAbs_FACE, face_map)
    face_colors = {}
    for name, body in model.bodies.items():
        color = bd.Color(model.appearance[name]['color'])
        for face in body.Faces():
            ordinal = face_map.FindIndex(face.wrapped)
            assert ordinal > 0, (name, 'face missing from battery')
            assert ordinal not in face_colors, (name, 'face shared with another body')
            face_colors[ordinal] = color
    assert len(face_colors) == face_map.Extent()
    # The pinned cadgen STEP exporter persists this per-face colour map.
    # No build123d children means one selectable part rather than an assembly.
    battery.cad_face_ordinal_colors = face_colors
    return bd.Compound(children=[battery], label='PARKSIDE PAPK 12 A4')


def make():
    """Return the complete coloured battery as one selectable component."""
    import cadquery as cq
    from cadgen import declare_input
    from lib.cadquery_adapter import Model
    font_path = Path(__file__).resolve().parents[2] / 'source/fonts/DejaVuSans-Bold.ttf'
    declare_input(font_path)
    model = Model("PARKSIDE PAPK 12 A4 - 12V 2Ah")

    def polar_point(terms, centre_y, t, grow=0.0):
        """Plan point of a symmetric outline r(t) = sum(c_k cos(k t)), t from -Y towards +X,
        moved grow along its outward normal (negative moves inwards)."""
        r = sum(c * math.cos(k * t) for k, c in enumerate(terms))
        dr = -sum(k * c * math.sin(k * t) for k, c in enumerate(terms))
        dx, dy = dr * math.sin(t) + r * math.cos(t), -dr * math.cos(t) + r * math.sin(t)
        L = math.hypot(dx, dy)
        return (r * math.sin(t) + grow * dy / L, centre_y - r * math.cos(t) - grow * dx / L)

    def polar_points(terms, centre_y, n=96, grow=0.0):
        return [polar_point(terms, centre_y, 2 * math.pi * i / n, grow) for i in range(n)]

    def spline_wire(points, z=0.0):
        return cq.Wire.assembleEdges([cq.Edge.makeSpline([cq.Vector(x, y, z) for x, y in points], periodic=True)])

    def outline_prism(grow, z0, z1):
        """Prism of the cap outline grown by grow (negative shrinks), from z0 to z1.
        The offset is sampled from the analytic outline, not taken with offset2D: the STEP
        writer drops faces swept from offset curves."""
        face = cq.Face.makeFromWires(spline_wire(polar_points(cap_radius_terms, cap_centre_y, grow=grow), z0))
        return cq.Workplane().add(cq.Solid.extrudeLinear(face, cq.Vector(0, 0, z1 - z0)))

    def yz_prism(points, x0, x1):
        """Polygon in (Y, Z), extruded from x0 to x1."""
        plane = cq.Plane(origin=(x0, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0))
        return cq.Workplane(plane).polyline(points).close().extrude(x1 - x0)

    def block(x0, x1, y0, y1, z0, z1):
        return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))

    def cylinder(d, x, y, z0, z1):
        return cq.Workplane("XY").add(cq.Solid.makeCylinder(d / 2, z1 - z0, cq.Vector(x, y, z0)))

    # ---------------------------------------------------------------- stick plan outline
    lobes = [cq.Vector(x, y, 0) for x, y in stick_lobes]
    order = [0, 2, 1]  # counter-clockwise: back -X, front, back +X

    def side(i, j):
        """Unit direction and outward normal of the straight side from lobe i to lobe j (CCW)."""
        d = (lobes[j] - lobes[i]).normalized()
        return d, cq.Vector(d.y, -d.x, 0)

    def stick_wire(grow=0.0):
        R = stick_radius + grow
        segs = []
        for k in range(3):
            i, j = order[k], order[(k + 1) % 3]
            d, n = side(i, j)
            segs.append((lobes[i] + n * R, lobes[j] + n * R, n))
        w = cq.Workplane("XY").moveTo(segs[0][0].x, segs[0][0].y)
        for k in range(3):
            a, b, n = segs[k]
            nxt = segs[(k + 1) % 3]
            w = w.lineTo(b.x, b.y)
            c = lobes[order[(k + 1) % 3]]
            mid = (n + nxt[2]).normalized() * R + c
            w = w.threePointArc((mid.x, mid.y), (nxt[0].x, nxt[0].y))
        return w.close().wire().val()

    def stick_prism(grow, z0, z1):
        face = cq.Face.makeFromWires(stick_wire(grow).translate(cq.Vector(0, 0, z0)))
        return cq.Workplane().add(cq.Solid.extrudeLinear(face, cq.Vector(0, 0, z1 - z0)))

    # ---------------------------------------------------------------- red cap
    cap = outline_prism(0, 0, cap_front_top)
    (sy0, sz0), (sy1, sz1) = cap_step
    cap = cap.cut(yz_prism([(sy0, sz0), (sy1, sz1), (sy1, 40), (40, 40), (40, sz0)], -40, 40))
    cap = cap.edges("<Z").fillet(cap_bottom_round)
    for y, z in cap_step:
        cap = cap.edges(cq.selectors.BoxSelector((-40, y - 0.05, z - 0.05), (40, y + 0.05, z + 0.05))).fillet(cap_step_round)
    wall = max(cap.faces("not %Plane").vals(), key=lambda f: f.Area())
    cap = cap.newObject([e for e in wall.Edges() if e.Center().z > cap_side_top - 1]).fillet(cap_top_round)
    model.add("red_cap", cap, op="new", body="cap")

    # ---------------------------------------------------------------- black collar
    collar = outline_prism(-collar_inset, 1.0, rim_height)
    collar = collar.union(stick_prism(stick_clearance_wall, 1.0, rim_height))
    rim_profile = [(rim_drop[0], rim_height + 5), (rim_drop[0], rim_height), (rim_drop[1], back_wall_height), (40, back_wall_height), (40, rim_height + 5)]
    collar = collar.cut(yz_prism(rim_profile, -40, 40))
    cup = outline_prism(-(collar_inset + collar_wall), cup_floor, rim_height + 1).cut(stick_prism(0, cup_floor - 1, rim_height + 2))
    collar = collar.cut(cup)
    for s in (-1, 1):
        x0, x1 = sorted((s * 15.0, s * 30.0))
        slit = block(x0, x1, slit_y[0], slit_y[1], slit_bottom + (slit_y[1] - slit_y[0]) / 2, rim_height + 1)
        slit = slit.union(cq.Workplane("XY").add(cq.Solid.makeCylinder((slit_y[1] - slit_y[0]) / 2, x1 - x0, cq.Vector(x0, sum(slit_y) / 2, slit_bottom + (slit_y[1] - slit_y[0]) / 2), cq.Vector(1, 0, 0))))
        collar = collar.cut(slit)
    collar = collar.cut(cap.val().copy())  # a copy, so the case does not share the cap's step face
    model.add("collar", collar, op="new", body="case")

    # ---------------------------------------------------------------- stick
    stick = stick_prism(0, cup_floor, stick_top).faces(">Z").edges().fillet(stick_top_round)
    model.add("stick", stick, op="join", body="case")

    def side_frame(name):
        """Origin on the stick side at the channel centre, tangent direction and outward normal."""
        if name == 'T':
            return cq.Vector(t_channel_x, stick_lobes[0][1] + stick_radius, 0), cq.Vector(1, 0, 0), cq.Vector(0, 1, 0)
        i, j = (0, 2) if name == 'plus' else (2, 1)
        d, n = side(i, j)
        start = lobes[2] + n * stick_radius
        sgn = 1 if name == 'minus' else -1  # distance measured from the front corner
        return start + d * (sgn * channel_s[name]), d, n

    def oriented_box(origin, d, n, along, across_in, across_out, z0, z1):
        """Box centred on origin along d (length along), from across_in inside to across_out outside along n."""
        w = cq.Workplane("XY").box(along, across_in + across_out, z1 - z0, centered=(True, False, False))
        w = w.translate((0, -across_in, z0))
        ang = math.degrees(math.atan2(n.y, n.x)) - 90.0
        return w.rotate((0, 0, 0), (0, 0, 1), ang).translate((origin.x, origin.y, 0))

    plates = []
    for name in ('plus', 'minus', 'T'):
        o, d, n = side_frame(name)
        width = t_channel_width if name == 'T' else channel_width
        model.add("channel_%s" % name, oriented_box(o, d, n, width, channel_depth, 5.0, rim_height - 3.0, stick_top + 1), op="cut", body="case")
        face_in = plate_face_depth + plate_thickness
        tab_recess = oriented_box(o, d, n, plate_width + 0.6, plate_face_depth + plate_tab + 0.4, 0.0, plate_z[1] - plate_thickness - 0.3, stick_top + 1)
        model.add("tab_recess_%s" % name, tab_recess, op="cut", body="case")
        plate = oriented_box(o, d, n, plate_width, face_in, -plate_face_depth, plate_z[0], plate_z[1])
        plate = plate.union(oriented_box(o, d, n, plate_width, plate_face_depth + plate_tab, -plate_face_depth, plate_z[1] - plate_thickness, plate_z[1]))
        model.add("plate_%s" % name, plate, op="new", body="plate_%s" % name)
        plates.append("plate_%s" % name)

    # ID pin in a round recess on the top.
    px, py = pin_centre
    model.add("pin_recess", cylinder(pin_recess_diameter, px, py, stick_top - 0.8, stick_top + 1.2), op="cut", body="case")
    pin = cylinder(pin_diameter, px, py, stick_top - 0.8, stick_top - 0.2).faces(">Z").edges().fillet(0.25)
    model.add("id_pin", pin, op="new", body="pin")

    # Moulded marks beside the contacts: T, +, -, ID and a ring.
    top = stick_top - mark_depth

    def mark_bar(cx, cy, length, width, angle):
        return cq.Workplane("XY").box(length, width, 2, centered=(True, True, False)).rotate((0, 0, 0), (0, 0, 1), angle).translate((cx, cy, top))

    for i, (x0, x1, y0, y1) in enumerate(mark_t):
        model.add("mark_t_%d" % i, block(x0, x1, y0, y1, top, top + 2), op="cut", body="case")
    cx, cy, span, arm, angle = mark_plus
    model.add("mark_plus", mark_bar(cx, cy, span, arm, angle).union(mark_bar(cx, cy, span, arm, angle + 90)), op="cut", body="case")
    model.add("mark_minus", mark_bar(*mark_minus), op="cut", body="case")
    cx, cy, d, groove = mark_ring
    model.add("mark_ring", cylinder(d, cx, cy, top, top + 2).cut(cylinder(d - 2 * groove, cx, cy, top - 1, top + 3)), op="cut", body="case")

    def text_solid(string, box, plane, thickness):
        """DejaVu Sans Bold `string` stretched to fill box (x0, x1, y0, y1) of `plane`, extruded `thickness` along its normal."""
        x0, x1, y0, y1 = box
        glyphs = cq.Workplane("XY").text(string, 10, thickness, combine=False, font="DejaVu Sans", fontPath=str(font_path), kind="bold", halign="left", valign="bottom").val()
        bb = glyphs.BoundingBox()
        sx, sy = (x1 - x0) / bb.xlen, (y1 - y0) / bb.ylen
        glyphs = glyphs.transformGeometry(cq.Matrix([[sx, 0, 0, x0 - sx * bb.xmin], [0, sy, 0, y0 - sy * bb.ymin], [0, 0, 1, 0]]))
        return glyphs.transformShape(plane.rG)

    x0, x1, y0, y1 = mark_id
    id_plane = cq.Plane(origin=(0, 0, top), xDir=(-1, 0, 0), normal=(0, 0, 1))  # read from the back: x = -X, y = -Y
    model.add("mark_id", text_solid("ID", (-x1, -x0, -y1, -y0), id_plane, 2), op="cut", body="case")

    # ---------------------------------------------------------------- latch arms: buttons and teeth
    def wall_shell(outside, inside, x_side):
        """Region between the collar outline grown by outside and shrunk by inside, on one side of the YZ plane."""
        outer = outline_prism(-collar_inset + outside, 0, 60)
        inner = outline_prism(-collar_inset - inside, -1, 61)
        x0, x1 = sorted((0.0, x_side * 40.0))
        return outer.cut(inner).intersect(block(x0, x1, -40, 40, -1, 61))

    def grown(points, g):
        cy = sum(p[0] for p in points) / len(points)
        cz = sum(p[1] for p in points) / len(points)
        out = []
        for y, z in points:
            dy, dz = y - cy, z - cz
            L = math.hypot(dy, dz)
            out.append((y + g * dy / L, z + g * dz / L))
        return out

    def yz_slot(p0, p1, width, x0, x1):
        """Round-ended slot from p0 to p1 in (Y, Z), extruded from x0 to x1."""
        (ya, za), (yb, zb) = p0, p1
        plane = cq.Plane(origin=(x0, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0))
        length = math.hypot(yb - ya, zb - za) + width
        angle = math.degrees(math.atan2(zb - za, yb - ya))
        return cq.Workplane(plane).center((ya + yb) / 2, (za + zb) / 2).slot2D(length, width, angle).extrude(x1 - x0)

    def outline_loft(grow0, z0, grow1, z1):
        """Ruled solid from the cap outline grown by grow0 at z0 to the outline grown by grow1 at z1."""
        wires = [spline_wire(polar_points(cap_radius_terms, cap_centre_y, grow=g), z) for g, z in ((grow0, z0), (grow1, z1))]
        return cq.Workplane().add(cq.Solid.makeLoft(wires, True))

    # The lead-in ramp follows the curved collar: everything outside a ruled surface that leaves the
    # teeth's outer face tooth_ramp[0] below their top and is tooth_ramp[1] further in at the top.
    ramp_h, ramp_in = tooth_ramp
    face_grow = -collar_inset - tooth_depth[0]
    ramp_z = (tooth_top - ramp_h - 1.0, tooth_top + 1.0)
    ramp_grow = [face_grow - ramp_in * (z - (tooth_top - ramp_h)) / ramp_h for z in ramp_z]
    ramp_cut = outline_prism(face_grow + 2.0, ramp_z[0], ramp_z[1]).cut(outline_loft(ramp_grow[0], ramp_z[0], ramp_grow[1], ramp_z[1]))

    buttons = []
    for j, s in enumerate((-1, 1)):
        x0, x1 = sorted((0.0, s * 40.0))
        pocket = yz_prism(grown(button_outline, button_gap), x0, x1).edges("|X").fillet(button_round + button_gap)
        model.add("button_pocket_%d" % j, pocket.intersect(wall_shell(1.0, 1.0, s)), op="cut", body="case")
        button = yz_prism(button_outline, x0, x1).edges("|X").fillet(button_round)
        button = button.intersect(wall_shell(button_proud, 1.0, s))
        model.add("button_%d" % j, button, op="new", body="button_%d" % j)
        width, depth = button_groove
        for k, (p0, p1) in enumerate(button_grooves):
            groove = yz_slot(p0, p1, width, x0, x1).intersect(wall_shell(button_proud + 1.0, depth - button_proud, s))
            model.add("button_groove_%d_%d" % (j, k), groove, op="cut", body="button_%d" % j)
        buttons.append("button_%d" % j)
        tooth = block(x0, x1, tooth_y[0], tooth_y[1], rim_height - 0.5, tooth_top).intersect(wall_shell(-tooth_depth[0], tooth_depth[1], s))
        tooth = tooth.cut(block(x0, x1, tooth_y[0] - 1, tooth_y[1] + 1, rim_height, tooth_catch).intersect(wall_shell(1.0, tooth_depth[0] + tooth_notch, s)))
        tooth = tooth.cut(ramp_cut)
        model.add("tooth_%d" % j, tooth, op="join", body="case")

    # ---------------------------------------------------------------- underside: holes, screws and feet
    for s in (-1, 1):
        for x, y in bottom_holes:
            model.add("hole", cylinder(bottom_hole[0], s * x, y, -1, bottom_hole[1]), op="cut", body="cap")
        x, y = feet
        d, h = foot
        R = (d * d / 4 + h * h) / (2 * h)  # sphere radius of a dome with base d and height h
        dome = cq.Workplane("XY").add(cq.Solid.makeSphere(R, cq.Vector(s * x, y, R - h), angleDegrees1=-90, angleDegrees2=90))
        model.add("foot", dome.intersect(block(s * x - d, s * x + d, y - d, y + d, -h - 1, 0.3)), op="join", body="cap")
    head_d, head_h = screw_head
    length, slot, depth = screw_recess
    screws = None
    for y in screw_y:
        model.add("screw_counterbore", cylinder(screw_counterbore[0], 0, y, -1, screw_counterbore[1]), op="cut", body="cap")
        face = screw_counterbore[1] - head_h
        head = cylinder(head_d, 0, y, face, screw_counterbore[1]).faces("<Z").edges().fillet(0.4)
        for angle in (0, 90):
            recess = cq.Workplane("XY").box(length, slot, depth + 1, centered=(True, True, False)).rotate((0, 0, 0), (0, 0, 1), angle).translate((0, y, face - 1))
            head = head.cut(recess)
        screws = head if screws is None else screws.union(head)
    model.add("screws", screws, op="new", body="screws")

    # ---------------------------------------------------------------- front nameplate
    # The nameplate frame is wrapped round the collar wall: u is arc length along the wall,
    # so traced widths stay true on the curve.
    path_s, path_x = [0.0], [0.0]
    last = None
    n = 2000
    for i in range(n + 1):
        p = polar_point(cap_radius_terms, cap_centre_y, math.pi * i / n, -collar_inset)
        if last:
            path_s.append(path_s[-1] + math.dist(p, last))
            path_x.append(p[0])
        last = p

    def wall_x(s):
        """X of the collar wall at arc length s from its front-most point (the wall is symmetric about X = 0)."""
        a = abs(s)
        i = min(max(bisect.bisect(path_s, a), 1), len(path_s) - 1)
        f = (a - path_s[i - 1]) / (path_s[i] - path_s[i - 1])
        return math.copysign(path_x[i - 1] + f * (path_x[i] - path_x[i - 1]), s)

    def densify(loop, step=0.25):
        pts = []
        for (u0, v0), (u1, v1) in zip(loop, list(loop[1:]) + [loop[0]]):
            L = math.hypot(u1 - u0, v1 - v0)
            if L < 1e-6:
                continue
            k = max(1, math.ceil(L / step))
            pts += [(u0 + (u1 - u0) * i / k, v0 + (v1 - v0) * i / k) for i in range(k)]
        return pts

    def rounded(points, r, step=0.08):
        """Polygon loop with every corner rounded to radius r."""
        out = []
        for i, p in enumerate(points):
            a, b = points[i - 1], points[(i + 1) % len(points)]
            u1 = (a[0] - p[0], a[1] - p[1])
            u2 = (b[0] - p[0], b[1] - p[1])
            l1, l2 = math.hypot(*u1), math.hypot(*u2)
            u1, u2 = (u1[0] / l1, u1[1] / l1), (u2[0] / l2, u2[1] / l2)
            half = math.acos(max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))) / 2
            bis = (u1[0] + u2[0], u1[1] + u2[1])
            lb = math.hypot(*bis)
            c = (p[0] + bis[0] / lb * r / math.sin(half), p[1] + bis[1] / lb * r / math.sin(half))
            t1 = (p[0] + u1[0] * r / math.tan(half), p[1] + u1[1] * r / math.tan(half))
            t2 = (p[0] + u2[0] * r / math.tan(half), p[1] + u2[1] * r / math.tan(half))
            a1 = math.atan2(t1[1] - c[1], t1[0] - c[0])
            sweep = (math.atan2(t2[1] - c[1], t2[0] - c[0]) - a1 + math.pi) % (2 * math.pi) - math.pi
            k = max(2, int(abs(sweep) * r / step))
            out += [(c[0] + r * math.cos(a1 + sweep * j / k), c[1] + r * math.sin(a1 + sweep * j / k)) for j in range(k + 1)]
        return out

    def glyph_shapes(string, box, step=0.03):
        """DejaVu Sans Bold `string` stretched to fill box (x0, x1, y0, y1), as point loops per glyph face."""
        x0, x1, y0, y1 = box
        faces = cq.Workplane("XY").text(string, 10, 1, combine=False, font="DejaVu Sans", fontPath=str(font_path), kind="bold", halign="left", valign="bottom").faces("<Z").vals()
        bb = cq.Compound.makeCompound(faces).BoundingBox()
        sx, sy = (x1 - x0) / bb.xlen, (y1 - y0) / bb.ylen
        shapes = []
        for face in faces:
            loops = []
            for wire in [face.outerWire()] + face.innerWires():
                k = max(16, int(wire.Length() * max(sx, sy) / step))
                loops.append([(x0 + sx * (q.x - bb.xmin), y0 + sy * (q.y - bb.ymin)) for q in (wire.positionAt(i / k) for i in range(k))])
            shapes.append(loops)
        return shapes

    def fitted(shapes, box):
        """Shapes moved and stretched so that together they fill box (x0, x1, y0, y1)."""
        pts = [p for loops in shapes for loop in loops for p in loop]
        u0, u1 = min(p[0] for p in pts), max(p[0] for p in pts)
        v0, v1 = min(p[1] for p in pts), max(p[1] for p in pts)
        x0, x1, y0, y1 = box
        return [[[(x0 + (u - u0) * (x1 - x0) / (u1 - u0), y0 + (v - v0) * (y1 - y0) / (v1 - v0)) for u, v in loop] for loop in loops] for loops in shapes]

    brand = [loops for ch, (u0, u1) in label_brand for loops in glyph_shapes(ch, (u0, u1) + label_brand_v)]
    front = cq.Plane(origin=(0, 0, 0), xDir=(1, 0, 0), normal=(0, -1, 0))  # seen from the front: x = X, y = Z

    def wall_decal(shapes, depth_in, depth_out):
        """Nameplate-frame shapes wrapped round the collar wall, from depth_in inside to depth_out outside its surface."""
        tool = None
        for loops in shapes:
            w = cq.Workplane(front)
            for loop in loops:
                w = w.polyline([(wall_x(label_s + u), label_z + v) for u, v in densify(loop)]).close()
            w = w.extrude(40)
            tool = w if tool is None else tool.union(w)
        z0, z1 = label_z - 1, label_z + 10
        shell = outline_prism(-collar_inset + depth_out, z0, z1).cut(outline_prism(-collar_inset - depth_in, z0 - 1, z1 + 1))
        return tool.intersect(shell)

    model.add("label", wall_decal([[rounded(label_outline, label_round)], [rounded(label_tab, label_tab_round)]], 0.05, label_thickness), op="new", body="label")
    model.add("label_red", wall_decal(label_red, -label_thickness, label_thickness + print_thickness), op="new", body="label_red")
    model.add("label_black", wall_decal(label_black + brand, -label_thickness, label_thickness + print_thickness), op="new", body="label_black")

    # ---------------------------------------------------------------- underside sticker
    def under(level):
        """Underside reading frame at `level` below the cap, seen from below: x = X, y = -Y."""
        return cq.Plane(origin=(0, 0, -level), xDir=(1, 0, 0), normal=(0, 0, -1))

    def ink(plane, shapes, thickness=print_thickness):
        solid = None
        for outline, *holes in shapes:
            w = cq.Workplane(plane).polyline(outline).close()
            for hole in holes:
                w = w.polyline(hole).close()
            w = w.extrude(thickness)
            solid = w if solid is None else solid.union(w)
        return solid

    def plate(plane, box, r, thickness=print_thickness):
        x0, x1, y0, y1 = box
        w = cq.Workplane(plane).center((x0 + x1) / 2, (y0 + y1) / 2).rect(x1 - x0, y1 - y0).extrude(thickness)
        return w.edges("|Z").fillet(r) if r else w

    sticker_face = cq.Face.makeFromWires(spline_wire(polar_points(sticker_radius_terms, sticker_centre_y), 0.05))
    model.add("sticker", cq.Solid.extrudeLinear(sticker_face, cq.Vector(0, 0, -(artwork_thickness + 0.05))), op="new", body="sticker")
    first, second = under(artwork_thickness), under(artwork_thickness + print_thickness)
    white = None
    for string, box in bottom_texts:
        glyphs = cq.Workplane().add(text_solid(string, box, first, print_thickness))
        white = glyphs if white is None else white.union(glyphs)
    (x0, x1, y0, y1), stroke = bottom_frame
    inside = plate(under(artwork_thickness - 0.01), (x0 + stroke, x1 - stroke, y0 + stroke, y1 - stroke), 0, print_thickness + 0.02)
    white = white.union(plate(first, (x0, x1, y0, y1), 0.25).cut(inside))
    white = white.union(ink(second, fitted(label_red[1:4], bottom_slashes)))
    model.add("bottom_white", white, op="new", body="bottom_white")
    red = plate(first, bottom_block, 0).union(ink(second, fitted(label_red[:1], pill_x))).union(ink(second, fitted(label_red[4:], pill_team)))
    model.add("bottom_red", red, op="new", body="bottom_red")
    model.add("pill", plate(first, *pill).union(plate(first, *pill_nub)), op="new", body="pill")
    model.add("pill_black", ink(second, fitted(label_black, pill_12v)), op="new", body="pill_black")
    model.add("pill_brand", ink(second, fitted(label_red[1:4] + brand, pill_brand)), op="new", body="pill_brand")

    model.material("abs", bodies=["cap"] + buttons, finish="matte", color="#c8141f")
    model.material("abs", bodies=["case"], finish="textured", color="#1b1c1d")
    model.material("nickel-plated steel", bodies=plates + ["pin"], finish="polished", color="#c9cbca")
    model.material("zinc-plated steel", bodies=["screws"], finish="satin", color="#4a4c4f")
    model.material("pc", bodies=["label", "pill"], finish="glossy", color="#dfe0e0")
    model.material("pc", bodies=["sticker"], finish="matte", color="#26282a")
    model.material("abs", bodies=["label_red", "bottom_red"], finish="matte", color="#c8141f")
    model.material("abs", bodies=["label_black", "pill_black"], finish="matte", color="#141516")
    model.material("abs", bodies=["bottom_white"], finish="matte", color="#ebebea")
    model.material("abs", bodies=["pill_brand"], finish="matte", color="#bfc0c0")
    return _single_component(model)


@step(out="../../STEP/parts/parkside_papk_12_a4.step")
def parkside_papk_12_a4():
    return make()


if __name__ == '__main__':
    parkside_papk_12_a4()
