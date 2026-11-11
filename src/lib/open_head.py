"""Open development head: a skeleton and a clip-on skin carved from the top head shell.

Part-local millimetres, as the top head shell: +X forward, +Y to the robot's left, +Z up.

The skeleton is the original top head shell with everything removed except
- the front band, which keeps the face-plate slot (rib behind, band in front), both
  front screw bosses and the original front edge, and
- a low belt along the seam with the bottom head shell, which keeps the seam lip and the
  rear and side screw bosses and rises in an arch over both jaw-pivot cutouts.
No material is added inside the original shell wall, so the skeleton takes the place of
the top head shell without coming closer to any other part.

The skin is the outer 0.8 mm of the same shell above the belt and behind the band, thin
for translucent filament. Its outside is the original surface. The inner part of the
skeleton's wall stands 2 mm higher than the outer part, as a lip behind the skin's edge.
Two front tongues hook under the band; three 1.2 mm snap tabs, one each side and one at
the rear, lie in outside recesses of the belt and click into windows.
"""
from __future__ import annotations

import math
from functools import lru_cache

SKIN_WALL = 0.8        # clip skin thickness, measured inward from the original outer surface
LIP_HEIGHT = 2.0       # the skeleton's inner wall stands this far past the seam, behind the skin edge
LIP_GAP = 0.25         # between the skin's inner face and the lip
SHELL_WALL = 2.295     # original top head shell wall (top_head_shell_native.wall)
SEAM_GAP = 0.3         # gap between skin edge and skeleton edge, all round

# Belt upper edge, side view.
BELT_Z = 285.0
ARCH_X = 35.237        # jaw-pivot cutout centre (top_head_shell_native.cut_xc)
ARCH_RISE = 22.5       # belt edge above BELT_Z over the cutout; cutout top is 15.6 above its centre
ARCH_HALF_WIDTH = 22.0
SIDE_BOSS_X = -1.5     # side screw boss, top at Z 290
SIDE_BOSS_RISE = 6.0
SIDE_BOSS_HALF_WIDTH = 9.0
REAR_RISE = 5.0        # rear screw boss, top at Z 288
REAR_X0, REAR_X1 = -16.0, -24.0

# Front band rear edge, side view: far enough back to keep both front bosses whole, then
# forward at the top to leave the Radxa USB-C side open. The face rib starts at X 106.785.
BAND_X_LOW = 96.0
BAND_X_HIGH = 101.0
BAND_Z0, BAND_Z1 = 310.0, 326.0

# Front tongues of the skin, under the band.
TONGUE_Y = (14.0, 24.0)            # |Y| range of each tongue
TONGUE_GAP = 0.3                   # under the band's inner wall
TONGUE_T = 1.2
TONGUE_X1 = 105.8                  # 1.0 mm short of the face rib
PAD_X0 = 94.0                      # thick root of each tongue, behind the band
HINGE_X, HINGE_Z = 100.85, 335.5   # the skin swings on its top front edge, against the band, when it is put on

# Snap tabs of the skin: one each side at TAB_X and one at the rear, on the centre plane.
TAB_T = 1.2
TAB_X = -14.0
TAB_HALF = 5.0
TAB_Z0 = 278.6
TAB_SIDE_GAP = 0.3
TAB_FACE_GAP = 0.25
BUMP_Z = 281.2
BUMP_R = 0.9
BUMP_LEN = 5.0
WINDOW = (6.4, 3.2)                # X length, Z height
OPEN_ANGLES = tuple(range(0, 13))  # degrees the recess is swept for


def _smooth(t):
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


def belt_top(x):
    z = BELT_Z + ARCH_RISE * math.exp(-((x - ARCH_X) / ARCH_HALF_WIDTH) ** 2)
    z += SIDE_BOSS_RISE * math.exp(-((x - SIDE_BOSS_X) / SIDE_BOSS_HALF_WIDTH) ** 2)
    z += REAR_RISE * _smooth((REAR_X0 - x) / (REAR_X0 - REAR_X1))
    return z


def band_rear(z):
    return BAND_X_LOW + (BAND_X_HIGH - BAND_X_LOW) * _smooth((z - BAND_Z0) / (BAND_Z1 - BAND_Z0))


def _prism(curve, closing):
    """Side-view region extruded across the whole head width.

    `curve` is a list of (X, Z) points joined by one smooth spline, so the cut it makes is
    one smooth face; `closing` continues from its end back to its start with straight lines.
    """
    import build123d as bd
    spline = bd.Spline(*[(x, 0.0, z) for x, z in curve])
    pts = [curve[-1], *closing, curve[0]]
    lines = bd.Polyline(*[(x, 0.0, z) for x, z in pts])
    face = bd.Face(bd.Wire([spline.edge(), *lines.edges()]))
    return bd.extrude(face, amount=90.0, both=True, dir=(0, 1, 0))


def _keep_region(grow=0.0):
    """Side-view region the skeleton keeps, grown by `grow` towards the skin."""
    step = 3.0
    xs = [-50.0 + step * i for i in range(int((140.0 + 50.0) / step) + 1)]
    belt = _prism([(x, belt_top(x) + grow) for x in xs], [(xs[-1], 260.0), (xs[0], 260.0)])
    zs = [260.0 + step * i for i in range(int((350.0 - 260.0) / step) + 1)]
    band = _prism([(band_rear(z) - grow, z) for z in zs], [(140.0, zs[-1]), (140.0, zs[0])])
    return belt + band


@lru_cache(maxsize=1)
def _source():
    import build123d as bd
    from parts.top_head_shell_native import make
    lid = make().solids()[0]
    skin = max(lid.faces(), key=lambda f: f.area)
    return lid, skin


@lru_cache(maxsize=None)
def _layer(depth):
    """The shell wall from the original outer surface to `depth` inside it."""
    import build123d as bd
    return bd.Solid.thicken(_source()[1], -depth)


def _skin_point(x, z, side):
    """Point and outward normal of the outer surface at (x, z) on one side."""
    import build123d as bd
    hits = _source()[1].find_intersection_points(bd.Axis((x, side * 90.0, z), (0, -side, 0)))
    point, normal = hits[0]
    if normal.Y * side < 0:
        normal = -normal
    return point, normal


def _box(x0, x1, y0, y1, z0, z1):
    import build123d as bd
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, abs(y1 - y0), z1 - z0)


def _tabs():
    """Each tab: its footprint box (by clearance), the surface point and outward normal at
    the bump, the bump's axis rotation and the belt window's box size."""
    import build123d as bd
    out = []
    for side in (1, -1):
        y0, y1 = sorted((side * 20.0, side * 90.0))
        point, normal = _skin_point(TAB_X, BUMP_Z, side)
        out.append(dict(name='left' if side > 0 else 'right',
                        box=lambda g, y0=y0, y1=y1: _box(TAB_X - TAB_HALF - g, TAB_X + TAB_HALF + g, y0, y1, TAB_Z0 - g, 300.0),
                        point=point, normal=normal, bump_rot=bd.Rot(0, 90, 0), window=(WINDOW[0], 5.0, WINDOW[1])))
    hits = _source()[1].find_intersection_points(bd.Axis((-90.0, 0.0, BUMP_Z), (1, 0, 0)))
    point, normal = min(hits, key=lambda h: h[0].X)
    if normal.X > 0:
        normal = -normal
    out.append(dict(name='rear', box=lambda g: _box(-60.0, -30.0, -TAB_HALF - g, TAB_HALF + g, TAB_Z0 - g, 300.0),
                    point=point, normal=normal, bump_rot=bd.Rot(90, 0, 0), window=(5.0, WINDOW[0], WINDOW[1])))
    return out


def _tongue_boxes(grow=0.0):
    out = []
    for side in (1, -1):
        y0, y1 = sorted((side * TONGUE_Y[0], side * TONGUE_Y[1]))
        out.append((y0 - grow, y1 + grow))
    return out


def _swing(shape, degrees):
    """The skin's opening motion: rear end up, about the front tongues."""
    import build123d as bd
    axis = bd.Axis((HINGE_X, 0.0, HINGE_Z), (0, 1, 0))
    return shape.rotate(axis, degrees)


def _solids(shape):
    return sorted(shape.solids(), key=lambda s: -s.volume)


DEBRIS_LIMIT = 0.5     # mm3 in total; loose fragments below this are removed and reported
removed_debris = {}


def _one_solid(shape, what):
    """The part as one solid. Loose fragments left by the cuts are removed, never kept."""
    solids = _solids(shape)
    debris = sum(s.volume for s in solids[1:])
    if not solids or debris > DEBRIS_LIMIT:
        raise ValueError(f"{what}: expected one solid, got {len(solids)} with volumes {[round(s.volume, 2) for s in solids]}")
    removed_debris[what] = dict(fragments=len(solids) - 1, volume_mm3=round(debris, 4))
    return solids[0]


@lru_cache(maxsize=1)
def _skin_parts():
    """Skin body, tabs, bumps and tongues, kept apart so the skeleton can be cut round them."""
    import build123d as bd
    body = _layer(SKIN_WALL) - _keep_region(SEAM_GAP)
    tabs, bumps = [], []
    for tab in _tabs():
        tabs.append(_layer(TAB_T) & tab['box'](0.0))
        centre = tab['point'] - tab['normal'] * (TAB_T + 0.05)
        bumps.append(bd.Pos(*centre) * tab['bump_rot'] * bd.Cylinder(BUMP_R, BUMP_LEN))
    tongues = []
    deep = SHELL_WALL + TONGUE_GAP + TONGUE_T
    for y0, y1 in _tongue_boxes():
        pad = _layer(deep) & _box(PAD_X0, BAND_X_HIGH - SEAM_GAP, y0, y1, 300.0, 345.0)
        blade = (_layer(deep) - _layer(SHELL_WALL + TONGUE_GAP)) & _box(BAND_X_HIGH - SEAM_GAP - 1.0, TONGUE_X1, y0, y1, 300.0, 345.0)
        tongues += [pad, blade]
    return body, tabs, bumps, tongues


@lru_cache(maxsize=1)
def make_skin():
    import build123d as bd
    body, tabs, bumps, tongues = _skin_parts()
    out = body
    for piece in (*tabs, *bumps, *tongues):
        out = out + piece
    solid = _one_solid(out, "open head skin")
    solid.label = "open_head_skin"
    solid.color = bd.Color("#99deef")
    return solid


@lru_cache(maxsize=1)
def make_skeleton():
    import build123d as bd
    lid, _ = _source()
    # Keep the wall up to LIP_HEIGHT past the seam, then take the outer layer back down to
    # the seam: the inner part that is left is the locating lip behind the skin's edge.
    out = (lid & _keep_region(LIP_HEIGHT)) - (_layer(SKIN_WALL + LIP_GAP) - _keep_region(0.0))
    for y0, y1 in _tongue_boxes(TAB_SIDE_GAP):
        out = out - _box(PAD_X0 - TAB_SIDE_GAP, BAND_X_HIGH, y0, y1, 300.0, 345.0)
    for tab in _tabs():
        # outside recess for the tab, swept through the skin's opening swing
        grown = _layer(TAB_T + TAB_FACE_GAP) & tab['box'](TAB_SIDE_GAP)
        for degrees in OPEN_ANGLES:
            out = out - _swing(grown, degrees)
        centre = tab['point'] - tab['normal'] * 2.6
        out = out - bd.Pos(*centre) * bd.Box(*tab['window'])
    solid = _one_solid(out, "open head skeleton")
    solid.label = "open_head_skeleton"
    solid.color = bd.Color("#1c1c1e")
    return solid
