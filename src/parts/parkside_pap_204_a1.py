"""
PARKSIDE PAP 204 A1 - 20 V / 4 Ah battery (X20V TEAM), exterior visual model.
Millimetres. Origin at the centre of the bottom face.
X = 123 mm length and slide direction: +X is the red release-paddle end, -X the contact nose.
Y = 83 mm width, Z up.

CONFIRMED (manufacturer values, not caliper measurements):
Lidl product 100407757: 123 x 83 x 70 mm, product mass 711 g.
https://www.lidl.de/p/parkside-20v-akku-4-ah-pap-204-a1/p100407757
Manufacturer manual:
https://manuals.sit-connect.com/public/articlemanual/0b3a082d-89bc-419c-89f6-042c875dc7e2.pdf

PHOTO-DERIVED: every other dimension, the colours and the printed artwork come from the
three Lidl product photos (both long sides and a three-quarter view). The three cameras were
fitted together (bundle adjustment) with the listed length and width as scale, and the
features were triangulated from them. Expect about 0.5 mm on corners seen in two photos and
about 1 mm elsewhere. On that scale the photos put the top of the release paddle at 66.4 mm,
not at the listed 70 mm height; the model follows the photos.
The X20V TEAM logo, the /// and the 4.0Ah are outlines traced from the photos; PARKSIDE is
set in DejaVu Sans Bold at the traced letter positions.
This is a visual and packaging model, not a mating drawing: the rail, contacts and latch
are drawn as they look, not to the X20V interface tolerances.
The bodies are solid, so the CAD mass is not the battery mass: use reference_product_mass_g.
"""
import math
from pathlib import Path
from cadgen import step

length = 123.0  # mm Manufacturer overall length
width = 83.0  # mm Manufacturer overall width
listed_height = 70.0  # mm Manufacturer overall height; the photos give 66.4 at the paddle
reference_product_mass_g = 711.0  # g Manufacturer product mass; not CAD mass
corner_clip = 9.5  # mm Plan 45 degree corner chamfer of the full outline
lower_inset = 0.7  # mm Lower case step under the upper case lip
bottom_chamfer = 0.8  # mm 45 degree chamfer round the underside
paddle_end_corner_cut = ((0.761, 0.390, -0.518), 53.51)  # (unit normal, mm offset) Plane cutting the +X+Y bottom corner; mirrored to +X-Y
nose_end_corner_cut = ((-0.439, 0.374, -0.817), 35.86)  # (unit normal, mm offset) Plane cutting the -X+Y bottom corner; mirrored to -X-Y
end_slot_y = (-17.0, 0.0, 17.0)  # mm Centres of the three red-lined slots at the bottom of each end
seam_height = 41.5  # mm Top of the lower case
lip_bottom = 42.3  # mm Bottom of the upper case's vertical lip (shadow gap below)
lip_top = 46.3  # mm Top of the upper case's vertical lip
deck_height = 52.55  # mm Flat deck under the rail, top of the chamfered band
band_side_inset = 6.5  # mm Band top inset on the long sides
band_end_inset = 5.7  # mm Band top inset at the ends
nose_end_face = ((-61.5, 42.91), (-56.12, 52.55))  # mm (X, Z) Bottom and top of the sloped -X end face; that end has no vertical lip
nose_corner_start = 42.3  # mm Height where the sloped -X corner faces leave the lip's corner edges
nose_corner_slope = 0.826  # mm Inward run of the -X corner faces per mm of height
wall_top_y = 30.9  # mm Half width of the latch housing at the top of its steep side wall
wall_top_z = 60.45  # mm Height of the top of the housing's steep side wall
strip_top_y = 27.95  # mm Half width at the top of the housing's sloped strip
strip_top_z = 64.03  # mm Height of the top of the housing's sloped strip
roof_half = 12.8  # mm Half width of the housing's flat roof
roof_height = 66.0  # mm Height of the housing's flat roof
end_bevel_slope = 1.35  # Rise per mm of the +X end bevel, which starts at the lip top
back_face_x = 11.45  # mm X of the housing's -X face at wall_top_z
back_face_slope = 0.772  # mm of X per mm of height on the housing's -X face
corner_bevel_point = (45.78, 28.42, 59.57)  # mm A point on the +X+Y corner bevel
corner_bevel_normal = (0.696, 0.375, 0.612)  # Outward normal of the +X+Y corner bevel
rail_nose = -45.0  # mm X of the rail nose
rail_half_width = 22.0  # mm Half width over the rail flanges
rail_neck_half = 19.5  # mm Half width of the rail neck under the flanges
rail_neck_top = 58.3  # mm Underside of the rail flanges
rail_top = 62.0  # mm Top of the rail
nose_chamfer = 5.0  # mm Plan chamfer at the rail nose corners
nose_cut_end = -31.0  # mm X where the open contact pocket ends
nose_cut_half = 14.5  # mm Half width of the open contact pocket
nose_floor = 57.0  # mm Floor of the open contact pocket
nose_round = 1.5  # mm Radius of the rail end top edge beside the pocket
slot_end = -18.6  # mm X where the contact slots end
slot_y = (3.9, 11.55)  # mm Contact slot centre lines, each side
slot_width = 1.3  # mm Contact slot width
vent_x = (-1.04, -8.46, -15.65, -23.17, -30.41, -37.7)  # mm Flange vent centres
vent_length = 6.2  # mm Flange vent length
vent_z = (59.1, 60.65)  # mm Flange vent bottom and top
tooth_front = 12.5  # mm X of the red locking tooth's ramp foot
tooth_back = 19.5  # mm X of the red locking tooth's vertical face
tooth_height = 64.6  # mm Top of the red locking tooth
tooth_half = 16.0  # mm Half width of the red locking tooth
pocket_x = (10.5, 24.5)  # mm Pocket round the tooth, in the housing's -X face
pocket_half = 17.5  # mm Half width of the tooth pocket
paddle_half = 14.6  # mm Half width of the red release paddle
paddle_back = 28.3  # mm X of the paddle top's rounded back end
paddle_top = 66.4  # mm Height of the paddle top
paddle_foot = 54.6  # mm Height where the paddle ends on the end bevel
check_button = (28.6, 37.75, 18.55, 23.07)  # mm Charge-check button X and Y extents (+Y side)
hexagon = [(17.2, 26.2), (36.0, 26.2), (43.6, 44.2), (35.6, 57.2), (16.4, 57.2), (8.6, 44.2)]  # mm (X, Z) on both sides
hexagon_depth = 2.0  # mm Depth of the hexagonal panel, measured in Y
label_outline = [(-45.84, 4.44), (27.33, 4.44), (10.72, 27.26), (-45.84, 27.26)]  # mm (X, Z) on both sides
led_half = 21.0  # mm Half length of the green charge-level bar
led_z = (40.0, 42.3)  # mm Bottom and top of the green charge-level bar
foot_groove_width = 6.6  # mm Width of the underside grooves
foot_groove_depth = 1.2  # mm Depth of the underside grooves
artwork_thickness = 0.12  # mm Printed label, proud of the case
print_thickness = 0.05  # mm Printed legends, proud of the face they are printed on
logo_start = {1: 15.73, -1: -40.01}  # mm X where the X20V TEAM logo starts, +Y and -Y side
logo_shift = {1: 0.0, -1: -0.08}  # mm Height of the logo above its +Y position
brand_start = {1: -20.28, -1: -43.45}  # mm X where /// PARKSIDE starts
brand_shift = {1: 0.0, -1: -0.29}  # mm Height of /// PARKSIDE above its +Y position
capacity_start = {1: 36.94, -1: 15.51}  # mm X where 4.0Ah starts

def _single_component(model):
    """Keep all solids in one STEP leaf, with body colours carried by faces."""
    from cadgen import build123d as bd
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_IndexedMapOfShape

    battery = bd.Compound(model.result().wrapped, label='parkside_pap_204_a1')
    face_map = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(battery.wrapped, TopAbs_FACE, face_map)
    face_colors = {}
    for name, body in model.bodies.items():
        color = bd.Color(model.appearance[name]['color'])
        for face in body.Faces():
            ordinal = face_map.FindIndex(face.wrapped)
            assert ordinal > 0, (name, 'face missing from battery')
            face_colors[ordinal] = color
    assert len(face_colors) == face_map.Extent()
    # The pinned cadgen STEP exporter persists this per-face colour map.
    # No build123d children means one selectable part rather than an assembly.
    battery.cad_face_ordinal_colors = face_colors
    return bd.Compound(children=[battery], label='PARKSIDE PAP 204 A1')


def make():
    """Return the complete coloured battery as one selectable component."""
    import cadquery as cq
    from cadgen import declare_input
    from lib.cadquery_adapter import Model
    font_path = Path(__file__).resolve().parents[2] / 'source/fonts/DejaVuSans-Bold.ttf'
    declare_input(font_path)
    model = Model("PARKSIDE PAP 204 A1 - 20V 4Ah")
    SQ = 2.0 - math.sqrt(2.0)  # inset of a 45 degree chamfer leg per mm of offset
    HL, HW = length / 2, width / 2


    def octagon(dx, dy=None):
        """Plan outline inset by dx at the ends and dy on the sides, 45 degree corners."""
        dy = dx if dy is None else dy
        a, b = HL - dx, HW - dy
        c = a + b - (HL + HW - corner_clip - (dx + dy) / 2 * math.sqrt(2.0))
        return [(-a + c, -b), (a - c, -b), (a, -b + c), (a, b - c), (a - c, b), (-a + c, b), (-a, b - c), (-a, -b + c)]


    def prism(points, z0, z1):
        return cq.Workplane("XY").workplane(offset=z0).polyline(points).close().extrude(z1 - z0)


    def ruled(p0, z0, p1, z1):
        w0 = cq.Wire.makePolygon([cq.Vector(x, y, z0) for x, y in p0], close=True)
        w1 = cq.Wire.makePolygon([cq.Vector(x, y, z1) for x, y in p1], close=True)
        return cq.Workplane().add(cq.Solid.makeLoft([w0, w1], True))


    def block(x0, x1, y0, y1, z0, z1):
        return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


    def xz_prism(points, y0, y1):
        """Polygon in (X, Z), extruded from y0 to y1."""
        plane = cq.Plane(origin=(0, y1, 0), xDir=(1, 0, 0), normal=(0, -1, 0))
        return cq.Workplane(plane).polyline(points).close().extrude(y1 - y0)


    def yz_prism(points, x0, x1):
        """Polygon in (Y, Z), extruded from x0 to x1."""
        plane = cq.Plane(origin=(x0, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0))
        return cq.Workplane(plane).polyline(points).close().extrude(x1 - x0)


    def side_prism(points, side, depth_in, depth_out):
        """Polygon in (X, Z) on a long side (side -1 or +1), from depth_in inside the y = 41.5 face to depth_out outside."""
        y_out = side * (HW + depth_out)
        y_in = side * (HW - depth_in)
        return xz_prism(points, min(y_in, y_out), max(y_in, y_out))


    def halfspace(point, normal, size=400.0):
        """A large block on the side of the plane through point that normal points to."""
        n = cq.Vector(*normal).normalized()
        x_dir = cq.Vector(1, 0, 0) if abs(n.z) > 0.99 else cq.Vector(0, 0, 1).cross(n).normalized()
        return cq.Workplane(cq.Plane(origin=cq.Vector(*point), xDir=x_dir, normal=n)).rect(size, size).extrude(size)


    # ---------------------------------------------------------------- lower case
    lower = ruled(octagon(lower_inset + bottom_chamfer), 0, octagon(lower_inset), bottom_chamfer)
    lower = lower.union(prism(octagon(lower_inset), bottom_chamfer, seam_height))
    model.add("lower_case_form", lower, op="new", body="lower_case")

    # Both ends' bottom corners are cut off by a sloping plane, larger at the paddle end.
    for name, (n, d) in (("paddle_end", paddle_end_corner_cut), ("nose_end", nose_end_corner_cut)):
        for j, side in enumerate((1, -1)):
            normal = (n[0], side * n[1], n[2])
            model.add("%s_corner_cut_%d" % (name, j), halfspace(tuple(d * c for c in normal), normal), op="cut", body="lower_case")

    # Five grooves across the underside, each with a red strip in its roof.
    foot_groove_x = (-30.0, -15.0, 0.0, 15.0, 30.0)
    for i, x in enumerate(foot_groove_x):
        model.add("foot_groove_%d" % i, block(x - foot_groove_width / 2, x + foot_groove_width / 2, -45, 45, -1, foot_groove_depth), op="cut", body="lower_case")
        strip = block(x - foot_groove_width / 2 + 0.3, x + foot_groove_width / 2 - 0.3, -(HW - lower_inset - 1.2), HW - lower_inset - 1.2, foot_groove_depth - 0.35, foot_groove_depth)
        model.add("foot_strip_%d" % i, strip, op="new", body="foot_strip_%d" % i)

    # Both ends: three slots open at the bottom, each with a recessed red insert.
    end_face_x = HL - lower_inset
    end_red = []
    for j, s in enumerate((1, -1)):
        for i, y in enumerate(end_slot_y):
            k = 3 * j + i
            slot_x = sorted((s * (end_face_x - 3.5), s * (end_face_x + 1)))
            insert_x = sorted((s * (end_face_x - 3.5), s * (end_face_x - 1.5)))
            model.add("end_slot_%d" % k, block(*slot_x, y - 3.25, y + 3.25, -1, 10.5), op="cut", body="lower_case")
            model.add("end_insert_%d" % k, block(*insert_x, y - 2.75, y + 2.75, 0.0, 9.3), op="new", body="end_red_%d" % k)
            end_red.append("end_red_%d" % k)

    # +X end: green charge-level bar in the seam, pointed 45 degree ends.
    led_h = led_z[1] - led_z[0]
    led_mid = (led_z[0] + led_z[1]) / 2
    led_outline = [(-led_half, led_mid), (-led_half + led_h / 2, led_z[0]), (led_half - led_h / 2, led_z[0]),
                   (led_half, led_mid), (led_half - led_h / 2, led_z[1]), (-led_half + led_h / 2, led_z[1])]
    model.add("led_window", yz_prism(led_outline, end_face_x - 1.0, end_face_x + 1.0), op="cut", body="lower_case")
    model.add("led_lens", yz_prism(led_outline, end_face_x - 1.0, HL - 0.5), op="new", body="led_lens")

    # ---------------------------------------------------------------- upper case
    # Vertical lip, then a 45 degree band up to the flat deck.
    deck_outline = octagon(band_end_inset, band_side_inset)
    upper = prism(octagon(0), lip_bottom, lip_top)
    upper = upper.union(ruled(octagon(0), lip_top, deck_outline, deck_height))
    model.add("upper_case_form", upper, op="new", body="upper_case")

    # The -X end has no vertical lip: its end face and both corner faces slope from the seam to the deck.
    (ex0, ez0), (ex1, ez1) = nose_end_face
    model.add("nose_end_face", halfspace((ex0, 0, ez0), (-(ez1 - ez0), 0, ex1 - ex0)), op="cut", body="upper_case")
    for j, side in enumerate((1, -1)):
        corner_normal = (-1 / math.sqrt(2.0), side / math.sqrt(2.0), nose_corner_slope)
        model.add("nose_corner_face_%d" % j, halfspace((-HL, side * (HW - corner_clip), nose_corner_start), corner_normal), op="cut", body="upper_case")

    # Latch housing over the +X end: the band, a steep side wall, a sloped strip, a gently sloped
    # top face and a flat roof, cut by the end bevel, two corner bevels and a sloped -X face.
    band_top_y = HW - band_side_inset
    housing_section = [(-HW, lip_top - 0.5), (-HW, lip_top), (-band_top_y, deck_height), (-wall_top_y, wall_top_z), (-strip_top_y, strip_top_z),
                       (-roof_half, roof_height), (roof_half, roof_height), (strip_top_y, strip_top_z), (wall_top_y, wall_top_z),
                       (band_top_y, deck_height), (HW, lip_top), (HW, lip_top - 0.5)]
    housing = yz_prism(housing_section, 0.0, HL)
    housing = housing.intersect(prism(octagon(0), lip_top - 0.5, listed_height))
    housing = housing.cut(halfspace((HL, 0, lip_top), (end_bevel_slope, 0, 1)))
    cbp, cbn = corner_bevel_point, corner_bevel_normal
    housing = housing.cut(halfspace(cbp, cbn)).cut(halfspace((cbp[0], -cbp[1], cbp[2]), (cbn[0], -cbn[1], cbn[2])))
    housing = housing.cut(halfspace((back_face_x, 0, wall_top_z), (-1, 0, back_face_slope)))
    model.add("latch_housing", housing, op="join", body="upper_case")

    # Pocket round the locking tooth, in the housing's -X face.
    pocket = block(pocket_x[0], pocket_x[1], -pocket_half, pocket_half, rail_neck_top, listed_height).edges("|Z").fillet(3.0)
    model.add("tooth_pocket", pocket, op="cut", body="upper_case")

    # Channel for the red release paddle: over the roof and down the end bevel.
    bevel_len = math.hypot(end_bevel_slope, 1.0)


    def bevel_x(offset, z):
        """X on the line parallel to the end bevel, moved by offset along its outward normal, at height z."""
        return (end_bevel_slope * HL + lip_top + offset * bevel_len - z) / end_bevel_slope


    channel = [(paddle_back - 0.8, listed_height), (paddle_back - 0.8, 63.6), (bevel_x(-2.8, 63.6), 63.6), (bevel_x(-2.8, paddle_foot - 0.6), paddle_foot - 0.6),
               (HL + 1, paddle_foot - 0.6), (HL + 1, listed_height)]
    channel_solid = xz_prism(channel, -(paddle_half + 0.6), paddle_half + 0.6).edges("|Z").fillet(1.0)
    model.add("paddle_channel", channel_solid, op="cut", body="upper_case")

    paddle = [(paddle_back, paddle_top), (bevel_x(0.0, paddle_top), paddle_top), (bevel_x(0.0, paddle_foot), paddle_foot),
              (bevel_x(-2.4, paddle_foot), paddle_foot), (bevel_x(-2.4, 64.0), 64.0), (paddle_back, 64.0)]
    paddle_solid = xz_prism(paddle, -paddle_half, paddle_half).edges("|Z").fillet(3.5).edges("|Y").fillet(0.5)
    model.add("release_paddle", paddle_solid, op="new", body="release_paddle")
    for i, x in enumerate((36.0, 38.6, 41.2, 43.8)):
        model.add("paddle_grip_%d" % i, block(x - 0.5, x + 0.5, -(paddle_half - 2.5), paddle_half - 2.5, paddle_top - 0.6, paddle_top + 1), op="cut", body="release_paddle")

    # Charge-check button on the +Y top face, with a lightning bolt.
    top_face_angle = math.degrees(math.atan2(roof_height - strip_top_z, strip_top_y - roof_half))
    bx0, bx1, by0, by1 = check_button
    button_y = (by0 + by1) / 2
    button_z = strip_top_z + (strip_top_y - button_y) * math.tan(math.radians(top_face_angle))


    def on_top_face(shape):
        return shape.rotate((0, 0, 0), (1, 0, 0), -top_face_angle).translate(((bx0 + bx1) / 2, button_y, button_z))


    button_w, button_d = bx1 - bx0, by1 - by0
    check_pocket = cq.Workplane("XY").rect(button_w + 0.8, button_d + 0.8).extrude(1.6).translate((0, 0, -0.6)).edges("|Z").fillet(1.0)
    model.add("check_button_pocket", on_top_face(check_pocket), op="cut", body="upper_case")
    button = cq.Workplane("XY").rect(button_w, button_d).extrude(1.5).translate((0, 0, -1.2)).edges("|Z").fillet(0.8).edges(">Z").fillet(0.25)
    bolt = [(2.4, -0.6), (-0.1, 1.0), (-0.1, -0.1), (-2.4, 0.6), (0.3, -1.1), (0.3, 0.0)]
    button = button.cut(cq.Workplane("XY").polyline(bolt).close().extrude(1.0).translate((0, 0, 0.15)))
    model.add("charge_check_button", on_top_face(button), op="new", body="check_button")

    # ---------------------------------------------------------------- rail
    rail = block(rail_nose, tooth_front + 1.0, -rail_neck_half, rail_neck_half, deck_height - 0.5, rail_neck_top)
    plate_outline = [(rail_nose, -rail_half_width + nose_chamfer), (rail_nose + nose_chamfer, -rail_half_width), (tooth_front + 1.0, -rail_half_width),
                     (tooth_front + 1.0, rail_half_width), (rail_nose + nose_chamfer, rail_half_width), (rail_nose, rail_half_width - nose_chamfer)]
    plate = prism(plate_outline, rail_neck_top, rail_top).faces(">Z").edges("not(>X)").chamfer(0.6)
    rail = rail.union(plate)
    model.add("rail_form", rail, op="new", body="rail")
    model.add("contact_pocket", block(rail_nose - 1, nose_cut_end, -nose_cut_half, nose_cut_half, nose_floor, rail_top + 1), op="cut", body="rail")
    # Round the rail end's top edge beside the pocket.
    for j, side in enumerate((-1, 1)):
        y0, y1 = sorted((side * (nose_cut_half - 0.1), side * (rail_half_width - nose_chamfer + 0.1)))
        edge = block(rail_nose - 1, rail_nose + nose_round, y0, y1, rail_top - nose_round, rail_top + 1)
        edge = edge.cut(cq.Workplane("XZ").center(rail_nose + nose_round, rail_top - nose_round).circle(nose_round).extrude(50, both=True))
        model.add("rail_end_round_%d" % j, edge, op="cut", body="rail")
    for i, y in enumerate((-slot_y[1], -slot_y[0], slot_y[0], slot_y[1])):
        model.add("contact_slot_%d" % i, block(nose_cut_end - 1, slot_end, y - slot_width / 2, y + slot_width / 2, rail_neck_top + 0.5, rail_top + 1), op="cut", body="rail")
        tip = cq.Workplane("XY").polyline([(nose_cut_end - 0.01, y - 1.1), (nose_cut_end + 1.1, y), (nose_cut_end - 0.01, y + 1.1)]).close().extrude(1.0).translate((0, 0, rail_top - 0.8))
        model.add("contact_slot_tip_%d" % i, tip, op="cut", body="rail")
        model.add("contact_blade_%d" % i, block(nose_cut_end + 1.0, slot_end - 1.0, y - 0.25, y + 0.25, rail_neck_top + 0.5, rail_top - 2.0), op="new", body="contact_%d" % i)

    # Rectangular vents along both flanges, and one on each nose chamfer.
    for j, side in enumerate((-1, 1)):
        for i, xc in enumerate(vent_x):
            y0, y1 = sorted((side * (rail_half_width - 2.2), side * (rail_half_width + 1)))
            x0, x1 = xc - vent_length / 2, min(xc + vent_length / 2, 9.0)
            x0 = max(x0, rail_nose + nose_chamfer + 0.8)
            model.add("rail_vent_%d_%d" % (j, i), block(x0, x1, y0, y1, *vent_z), op="cut", body="rail")
        nose_vent = block(-2.0, 2.0, -1.0, 2.2, *vent_z).rotate((0, 0, 0), (0, 0, 1), 45 if side > 0 else -45)
        cx, cy = rail_nose + nose_chamfer / 2, side * (rail_half_width - nose_chamfer / 2)
        nose_vent = nose_vent.translate((cx, cy, 0)) if side < 0 else nose_vent.mirror("XZ").translate((cx, cy, 0))
        model.add("rail_nose_vent_%d" % j, nose_vent, op="cut", body="rail")

    # ---------------------------------------------------------------- red locking tooth
    tooth = xz_prism([(tooth_front, rail_neck_top), (tooth_front, rail_top), (tooth_front + 2.6, tooth_height), (tooth_back, tooth_height), (tooth_back, rail_neck_top)], -tooth_half, tooth_half)
    tooth = tooth.edges("|Y and >Z").fillet(0.4)
    model.add("locking_tooth", tooth, op="new", body="locking_tooth")
    for i, y in enumerate((-10.5, 0.0, 10.5)):
        notch = xz_prism([(tooth_front - 0.1, rail_top - 0.1), (tooth_front + 2.7, tooth_height + 0.05), (tooth_front + 2.7, tooth_height - 1.4)], y - 2.0, y + 2.0)
        model.add("tooth_notch_%d" % i, notch, op="cut", body="locking_tooth")

    # ---------------------------------------------------------------- side hexagons and labels
    # A recessed hexagonal panel crosses the seam on both sides. Its floor follows each face it
    # crosses: the case minus itself moved inwards is a skin of that depth on the side faces.
    upper_form = upper.union(housing)
    for j, side in enumerate((-1, 1)):
        hex_tool = side_prism(hexagon, side, 20.0, 2.0)
        shift = (0, -side * hexagon_depth, 0)
        model.add("hex_panel_lower_%d" % j, hex_tool.intersect(lower.cut(lower.translate(shift))), op="cut", body="lower_case")
        model.add("hex_panel_upper_%d" % j, hex_tool.intersect(upper_form.cut(upper_form.translate(shift))), op="cut", body="upper_case")
        label = side_prism(label_outline, side, lower_inset + 0.05, -lower_inset + artwork_thickness)
        model.add("side_label_%d" % j, label, op="new", body="side_label_%d" % j)
    model.material("abs", bodies=["side_label_0", "side_label_1"], finish="matte", color="#323b42")

    # ---------------------------------------------------------------- printed artwork
    # Traced from the photos. Each legend is drawn in its face's reading frame (x to the right as
    # seen from outside, y up, mm); each shape is [outline, hole, ...]. Both sides carry the same
    # print. The 4.0Ah is printed on the hexagon's sloped floor, y measured up the slope.
    logo_red = [  # the X and TEAM
        [[(12.03, 17.52), (11.83, 17.60), (9.63, 17.61), (8.75, 17.49), (6.44, 14.76), (6.04, 14.43), (5.96, 14.43), (5.36, 15.06), (4.07, 16.67), (3.88, 17.05), (3.25, 17.55), (1.43, 17.61), (0.43, 17.57), (0.05, 17.44), (0.00, 17.09), (0.07, 16.44), (0.22, 16.07), (2.18, 13.63), (2.46, 13.40), (2.55, 13.15), (3.00, 12.68), (3.70, 11.72), (3.64, 11.50), (0.83, 8.24), (0.96, 6.21), (1.06, 6.09), (1.91, 6.01), (3.01, 6.00), (3.29, 6.07), (3.79, 6.55), (4.02, 6.95), (4.35, 7.27), (4.62, 7.74), (4.92, 7.96), (5.23, 8.51), (5.68, 9.04), (5.93, 8.98), (8.02, 6.50), (8.22, 6.37), (8.31, 6.39), (8.19, 7.00), (8.27, 7.82), (8.81, 9.09), (8.91, 9.54), (9.19, 10.09), (9.19, 10.29), (8.96, 10.72), (8.09, 11.80), (8.05, 12.12), (8.65, 12.97), (9.31, 13.66), (9.43, 13.99), (9.73, 14.24), (9.79, 14.41), (10.16, 14.78), (10.67, 15.53), (11.35, 16.25), (11.85, 17.05), (12.13, 17.32)]],
        [[(55.19, 6.33), (55.20, 8.03), (55.10, 9.03), (55.01, 9.33), (54.78, 9.46), (52.66, 9.47), (52.38, 9.37), (51.98, 8.72), (51.67, 7.95), (51.42, 7.63), (50.93, 8.18), (50.53, 9.01), (50.28, 9.26), (49.98, 9.41), (48.23, 9.44), (47.68, 9.37), (47.53, 9.22), (47.65, 7.40), (47.54, 7.07), (47.12, 7.42), (46.18, 8.86), (45.76, 9.33), (45.56, 9.41), (44.01, 9.39), (43.83, 9.29), (43.65, 8.85), (43.38, 8.47), (42.67, 7.10), (42.39, 6.93), (42.09, 6.85), (40.77, 6.84), (39.79, 6.92), (39.64, 7.07), (39.57, 7.44), (39.72, 7.54), (41.47, 7.51), (42.10, 7.58), (42.17, 7.75), (42.12, 7.93), (41.73, 8.21), (40.15, 8.19), (39.65, 8.27), (39.48, 8.42), (39.48, 8.59), (39.55, 8.67), (40.13, 8.74), (42.20, 8.73), (42.28, 8.85), (42.28, 9.15), (42.16, 9.30), (41.86, 9.43), (39.78, 9.37), (37.46, 9.46), (35.41, 9.37), (34.78, 9.45), (33.06, 9.43), (32.91, 9.36), (32.83, 9.13), (32.83, 8.88), (33.05, 8.68), (33.40, 8.58), (34.13, 8.65), (34.53, 8.40), (34.63, 8.15), (34.69, 6.45), (34.84, 6.24), (35.11, 6.14), (36.24, 6.21), (36.29, 6.84), (36.20, 8.44), (36.28, 8.56), (36.45, 8.64), (37.60, 8.65), (37.80, 8.45), (37.99, 6.43), (38.11, 6.25), (38.36, 6.15), (42.72, 6.20), (43.49, 6.14), (43.84, 6.29), (44.09, 6.67), (44.59, 6.74), (45.64, 6.68), (46.41, 6.23), (46.74, 6.15), (49.34, 6.21), (49.42, 6.29), (49.27, 7.84), (49.35, 8.16), (49.73, 7.89), (50.49, 6.46), (50.87, 6.18), (51.39, 6.13), (52.02, 6.15), (52.16, 6.22), (52.57, 6.77), (53.02, 7.82), (53.30, 8.06), (53.38, 8.06), (53.50, 7.86), (53.49, 6.46), (53.64, 6.24), (54.94, 6.13), (55.12, 6.20)], [(44.83, 8.31), (44.93, 8.31), (45.27, 7.99), (45.42, 7.58), (45.35, 7.43), (44.77, 7.39), (44.47, 7.54), (44.52, 7.89)]],
    ]
    logo_white = [  # 2, 0 and V
        [[(18.65, 13.04), (18.30, 13.34), (17.55, 13.44), (11.25, 13.40), (10.60, 11.88), (10.62, 11.71), (15.97, 11.73), (16.22, 11.60), (16.29, 11.33), (16.04, 10.80), (11.24, 10.80), (10.61, 10.73), (10.29, 10.56), (10.09, 10.36), (9.83, 9.91), (9.00, 7.72), (8.92, 7.37), (8.94, 6.82), (9.09, 6.49), (9.39, 6.24), (10.14, 6.13), (16.31, 6.17), (17.00, 7.80), (16.95, 7.85), (11.52, 7.85), (11.30, 8.03), (11.38, 8.53), (11.55, 8.88), (11.95, 9.10), (17.43, 9.09), (17.66, 9.49), (18.70, 12.09), (18.77, 12.51)]],
        [[(27.68, 13.01), (27.45, 13.26), (26.91, 13.44), (20.83, 13.42), (20.53, 13.38), (20.15, 13.18), (19.80, 12.68), (17.85, 7.82), (17.72, 7.29), (17.72, 6.87), (17.82, 6.57), (18.14, 6.26), (18.89, 6.13), (24.51, 6.13), (25.22, 6.33), (25.52, 6.62), (27.65, 11.81), (27.77, 12.36), (27.78, 12.76)], [(25.27, 11.55), (25.29, 11.25), (24.05, 8.21), (23.80, 7.93), (23.45, 7.83), (20.60, 7.83), (20.23, 7.98), (20.18, 8.33), (21.32, 11.25), (21.62, 11.60), (22.12, 11.74), (25.05, 11.73)]],
        [[(38.28, 13.40), (38.18, 13.47), (35.06, 13.44), (30.80, 8.49), (30.53, 8.49), (30.45, 8.72), (32.58, 13.36), (32.48, 13.46), (29.41, 13.43), (26.21, 6.19), (31.96, 6.14), (33.85, 8.28), (33.78, 8.40), (32.65, 8.41), (32.58, 8.61), (32.53, 9.58), (35.03, 9.64)]],
    ]
    brand_red = [  # ///
        [[(1.19, 25.43), (0.86, 25.46), (0.71, 25.36), (0.36, 24.34), (0.00, 23.61), (0.05, 23.41), (0.32, 23.31), (0.60, 23.36), (1.36, 25.08), (1.33, 25.28)]],
        [[(2.39, 25.45), (2.16, 25.43), (1.96, 25.30), (1.73, 24.95), (1.45, 24.13), (1.20, 23.71), (1.17, 23.48), (1.25, 23.36), (1.52, 23.28), (1.82, 23.48), (2.53, 25.12), (2.51, 25.32)]],
        [[(3.51, 25.44), (3.26, 25.47), (2.88, 24.92), (2.68, 24.17), (2.45, 23.92), (2.35, 23.65), (2.47, 23.32), (2.82, 23.32), (3.03, 23.50), (3.41, 24.57), (3.68, 25.04), (3.68, 25.24)]],
    ]
    brand_letters = list(zip("PARKSIDE", (6.40, 8.39, 10.98, 13.58, 15.90, 17.73, 19.67, 22.13)))  # letter centres
    brand_baseline, brand_cap_height = 23.37, 2.19
    cap_height_ratio = 0.729  # DejaVu Sans Bold cap height per unit of font size
    capacity = [  # 4.0Ah
        [[(1.78, 4.14), (0.08, 2.04), (0.00, 1.81), (0.00, 0.89), (0.38, 0.84), (2.80, 0.86), (2.93, 0.76), (2.90, 0.29), (2.95, 0.24), (4.13, 0.26), (4.15, 0.81), (4.50, 0.86), (4.58, 0.94), (4.58, 1.89), (4.50, 1.94), (4.25, 1.91), (4.18, 1.99), (4.23, 4.11), (4.05, 4.16)], [(3.00, 2.99), (2.98, 1.96), (2.93, 1.89), (1.30, 1.86), (1.25, 1.91), (2.20, 3.04), (2.93, 3.06)]],
        [[(4.98, 1.51), (4.93, 1.44), (4.93, 0.26), (6.20, 0.26), (6.25, 1.49), (6.15, 1.54)]],
        [[(7.00, 3.69), (6.80, 3.34), (6.68, 2.81), (6.65, 1.61), (6.80, 0.96), (7.10, 0.56), (7.70, 0.29), (9.00, 0.16), (10.40, 0.31), (10.83, 0.49), (11.23, 0.86), (11.45, 1.54), (11.50, 2.51), (11.35, 3.44), (11.03, 3.86), (10.73, 4.04), (10.25, 4.16), (7.95, 4.16), (7.50, 4.04), (7.20, 3.89)], [(8.05, 2.86), (8.33, 3.06), (8.73, 3.11), (9.58, 3.11), (9.90, 3.04), (10.10, 2.89), (10.20, 2.61), (10.23, 2.21), (10.15, 1.66), (9.95, 1.44), (9.65, 1.34), (8.43, 1.34), (8.23, 1.39), (8.00, 1.59), (7.93, 1.74), (7.93, 2.59)]],
        [[(15.25, 4.19), (13.28, 4.16), (12.40, 2.04), (11.65, 0.41), (11.68, 0.26), (12.90, 0.26), (13.10, 0.71), (13.20, 0.79), (15.18, 0.79), (15.28, 0.71), (15.38, 0.39), (15.48, 0.29), (16.75, 0.31), (16.78, 0.36), (15.73, 3.09)], [(14.45, 3.04), (14.85, 1.89), (14.75, 1.81), (13.60, 1.81), (13.55, 1.96), (13.93, 2.96), (14.00, 3.04)]],
        [[(17.18, 4.41), (17.13, 4.36), (17.05, 0.31), (18.25, 0.31), (18.35, 2.19), (18.60, 2.41), (19.10, 2.49), (19.63, 2.44), (19.88, 2.21), (19.93, 1.94), (19.90, 0.31), (21.10, 0.31), (21.18, 2.64), (21.13, 3.01), (20.98, 3.31), (20.75, 3.49), (20.53, 3.56), (19.05, 3.54), (18.75, 3.41), (18.40, 3.11), (18.33, 3.11), (18.38, 4.36), (18.20, 4.44)]],
    ]


    def print_plane(side, x_start, z0, sloped=False):
        """Reading-frame plane on the long side `side`, starting at x_start: the label face, or the hexagon's sloped floor."""
        if not sloped:
            return cq.Plane(origin=(x_start, side * (HW - lower_inset + artwork_thickness), z0), xDir=(-side, 0, 0), normal=(0, side, 0))
        rise, run = deck_height - lip_top, band_side_inset
        return cq.Plane(origin=(x_start, side * (HW - hexagon_depth), lip_top), xDir=(-side, 0, 0), normal=(0, side * rise, run))


    def ink(plane, shapes):
        solid = None
        for outline, *holes in shapes:
            w = cq.Workplane(plane).polyline(outline).close()
            for hole in holes:
                w = w.polyline(hole).close()
            w = w.extrude(print_thickness)
            solid = w if solid is None else solid.union(w)
        return solid


    for j, side in enumerate((-1, 1)):
        logo = print_plane(side, logo_start[side], logo_shift[side])
        brand = print_plane(side, brand_start[side], brand_shift[side])
        model.add("print_red_%d" % j, ink(logo, logo_red).union(ink(brand, brand_red)), op="new", body="print_red_%d" % j)
        model.add("print_white_%d" % j, ink(logo, logo_white), op="new", body="print_white_%d" % j)
        for k, (letter, x) in enumerate(brand_letters):
            glyph = cq.Workplane(brand).center(x, brand_baseline).text(letter, brand_cap_height / cap_height_ratio, print_thickness, combine=False,
                                                                         font="DejaVu Sans", fontPath=str(font_path), kind="bold", halign="center", valign="bottom")
            model.add("brand_%d_%d" % (j, k), glyph, op="join", body="print_white_%d" % j)
        model.add("capacity_%d" % j, ink(print_plane(side, capacity_start[side], 0.0, sloped=True), capacity), op="new", body="capacity_%d" % j)
    model.material("abs", bodies=["print_red_0", "print_red_1"], finish="matte", color="#c8141f")
    model.material("abs", bodies=["print_white_0", "print_white_1", "capacity_0", "capacity_1"], finish="matte", color="#ebebea")

    model.material("abs", bodies=["lower_case", "upper_case"], finish="textured", color="#1b1c1d")
    model.material("abs", bodies=["rail"], finish="matte", color="#1d1e1f")
    model.material("abs", bodies=["release_paddle", "locking_tooth", "check_button"], finish="matte", color="#c8141f")
    model.material("abs", bodies=end_red + ["foot_strip_%d" % i for i in range(5)], finish="matte", color="#c8141f")
    model.material("pc", bodies=["led_lens"], finish="glossy", color="#12c46a")
    model.material("stainless-304", bodies=["contact_%d" % i for i in range(4)], finish="polished", color="#b7b9b8")
    return _single_component(model)


@step(out="../../STEP/parts/parkside_pap_204_a1.step")
def parkside_pap_204_a1():
    return make()


if __name__ == '__main__':
    parkside_pap_204_a1()
