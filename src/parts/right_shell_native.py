from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    """Frankenduck body shell side, native STEVE part (port of cad_blender/code_cad/body_shell/build.py,
    canonical variant `--collar --locators`). World millimetres at the saved assembly pose.
    
    Parameter SIDE: +1 builds left_shell (Y >= 0), -1 builds right_shell (Y <= 0, KCD1 port and collar).
    The skin is the analytic fair surface (periodic cubic B-spline profile, straight band, rational
    quarter-ellipse return, flat side panel), trimmed and thickened 3 mm inward; the rest are solid
    features measured on the canonical mesh.
    """
    import math
    import cadquery as cq
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing, BRepBuilderAPI_Transform
    from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeThickSolid
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.Geom import Geom_BSplineSurface
    from OCP.ShapeCustom import ShapeCustom
    from OCP.TColStd import TColStd_Array1OfReal, TColStd_Array1OfInteger, TColStd_Array2OfReal
    from OCP.TColgp import TColgp_Array2OfPnt
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf
    from lib.steve_compat import Model
    SIDE = -1
    SWITCH_PORT = 1
    NECK_NOTCH_HALF_WIDTH = 20.6
    SKIN_CENTRE_X = -17.0
    SKIN_CENTRE_Z = 148.5
    Y_RETURN = 30.5
    RETURN_DEPTH = 12.5
    PANEL_SCALE = 0.72
    WALL = 3.0
    PROFILE_CTL_XZ = [(-61.969364, 141.721413), (-60.769652, 133.872594), (-57.276993, 126.803988), (-50.794014, 122.358513), (-43.022885, 120.769484), (-35.077182, 120.67624), (-27.125195, 120.697049), (-19.174847, 120.866402), (-11.224499, 121.035755), (-3.274151, 121.205108), (4.676196, 121.374461), (12.626544, 121.543815), (20.576892, 121.713168), (26.636799, 126.072967), (28.103164, 133.853673), (28.415338, 141.798651), (28.519343, 149.748904), (26.829561, 157.478466), (22.453743, 164.065265), (16.146531, 168.833552), (8.767016, 171.7796), (1.031361, 173.596575), (-6.854049, 174.594652), (-14.797595, 174.91515), (-22.749282, 174.893843), (-30.701087, 174.819608), (-38.641541, 174.419028), (-46.516012, 173.371864), (-53.96208, 170.678255), (-59.525821, 165.128391), (-61.936164, 157.59931), (-62.307761, 149.666182)]
    HIP_Y_HALF = 38.2
    HIP_Z_TOP = 127.5
    FRONT_X_MIN = 6.0
    FRONT_Y_HALF = 36.0
    FRONT_Z_MIN = 154.5
    SEAM_GAP = 0.075
    PATCH_X_MAX = -48.0
    PATCH_Z_MIN = 147.6
    PATCH_Z_MAX = 169.0
    CORRIDOR_X_MIN = -56.0
    CORRIDOR_X_MAX = -37.25
    CORRIDOR_Y_HALF = 22.5
    CORRIDOR_Z_TOP = 160.75
    SERVICE_X_MIN = -28.5
    SERVICE_Y_HALF = 24.5
    SERVICE_Z_MIN = 164.0
    SERVICE_CORNER = 3.0
    DOOR_X_MAX = -55.5
    DOOR_Y_HALF = 22.5
    DOOR_Z_TOP = 147.6
    CHAMFER_NX = -0.70248
    CHAMFER_NZ = -0.7117
    CHAMFER_D = -61.7262
    NOTCH_NX = 0.51308
    NOTCH_NZ = 0.85834
    NOTCH_D = 142.1346
    NOTCH_X_MIN = 20.0
    DECK_Y = 37.0
    SPOT_R = 4.85
    BOSS_Z = 162.0
    BOSS_REAR_X = -24.0
    BOSS_R = 4.6
    BOSS_ROUND = 0.45
    PAD_X = -20.0
    PAD_R = 3.5
    FRONT_X = 10.0
    WELL_R = 2.6
    REAR_WELL_FLOOR = 41.15
    FRONT_WELL_FLOOR = 40.35
    BORE_R = 1.1
    BORE_END = 36.9
    BASE_TOP_Z = 164.45
    SOCKET_FLAT_X = 5.6
    SOCKET_R = 3.2
    RING_START = 40.5
    RING_END = 35.0
    ENTRY_CHAMFER = 0.6
    LIP_X_MIN = -21.6
    LIP_X_MAX = -16.0
    LIP_START = 40.5
    LIP_END = 35.6
    LIP_T = 1.5
    LIP_LEAD = 0.6
    SW_X = -36.0
    SW_Z = 139.0
    SW_HOLE_R = 7.5
    SW_KEY_HALF = 1.0
    SW_KEY_TOP = 146.7
    SW_RELIEF_Y = 41.0
    SW_RELIEF_R = 10.0
    SW_RELIEF_DEPTH = 3.0
    COLLAR_RI = 10.0
    COLLAR_RO = 12.5
    COLLAR_RIM_Y = 50.6
    COLLAR_CH = 0.6
    COLLAR_RF = 13.5
    COLLAR_BASE_Y = 41.5
    S = 1.0 if SIDE > 0 else -1.0
    Y_PANEL = Y_RETURN + RETURN_DEPTH
    
    def _y(a):
        """World Y of a shell-side |Y| value."""
        return S * a
    
    def _box(x0, x1, y0, y1, z0, z1):
        y0, y1 = sorted((y0, y1))
        return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))
    
    def _cyl_y(x, z, r, a0, a1):
        """Cylinder about an axis parallel to Y between shell-side |Y| a0 and a1."""
        y0, y1 = sorted((_y(a0), _y(a1)))
        return cq.Solid.makeCylinder(r, y1 - y0, cq.Vector(x, y0, z), cq.Vector(0, 1, 0))
    
    def _cone_in(x, z, r0, r1, a, h):
        """Cone starting at |Y| = a and growing toward the inside (smaller |Y|)."""
        return cq.Solid.makeCone(r0, r1, h, cq.Vector(x, _y(a), z), cq.Vector(0, -S, 0))
    
    def _halfspace(n, d, lo, hi):
        """{p: n.p >= d} within the axis box lo..hi (lo/hi already world)."""
        l = math.sqrt(sum((c * c for c in n)))
        n = [c / l for c in n]
        ax = gp_Ax2(gp_Pnt(*(c * d for c in n)), gp_Dir(*n))
        xd, yd = (ax.XDirection(), ax.YDirection())
        big = 400.0
        o = [n[i] * d - (xd.Coord(i + 1) + yd.Coord(i + 1)) * big / 2 for i in range(3)]
        slab = cq.Shape.cast(BRepPrimAPI_MakeBox(gp_Ax2(gp_Pnt(*o), gp_Dir(*n), xd), big, big, big).Shape())
        return _box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]).intersect(slab)
    
    def _row(q, y):
        return [(SKIN_CENTRE_X + q * (x - SKIN_CENTRE_X), y, SKIN_CENTRE_Z + q * (z - SKIN_CENTRE_Z)) for x, z in PROFILE_CTL_XZ]
    
    def _bspline(rows, weights, vdeg, vmults):
        n = len(rows[0])
        poles = TColgp_Array2OfPnt(1, n, 1, len(rows))
        w = TColStd_Array2OfReal(1, n, 1, len(rows))
        for j, row in enumerate(rows):
            for i in range(n):
                poles.SetValue(i + 1, j + 1, gp_Pnt(*row[i]))
                w.SetValue(i + 1, j + 1, weights[j])
        uk = TColStd_Array1OfReal(1, n + 1)
        um = TColStd_Array1OfInteger(1, n + 1)
        for i in range(n + 1):
            uk.SetValue(i + 1, float(i))
            um.SetValue(i + 1, 1)
        vk = TColStd_Array1OfReal(1, 2)
        vk.SetValue(1, 0.0)
        vk.SetValue(2, 1.0)
        vm = TColStd_Array1OfInteger(1, 2)
        vm.SetValue(1, vmults)
        vm.SetValue(2, vmults)
        surf = Geom_BSplineSurface(poles, w, uk, vk, um, vm, 3, vdeg, True, False)
        return cq.Face(BRepBuilderAPI_MakeFace(surf, 1e-07).Face())
    
    def _mirror_y(shape):
        t = gp_Trsf()
        t.SetMirror(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)))
        return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape())
    
    def thickened_wall():
        """Skin in the right-shell frame (Y <= 0), trimmed, thickened inward, mirrored for the left."""
        y1, y2 = (-Y_RETURN, -Y_PANEL)
        band = _bspline([_row(1.0, 0.0), _row(1.0, y1)], [1.0, 1.0], 1, 2)
        ret = _bspline([_row(1.0, y1), _row(1.0, y2), _row(PANEL_SCALE, y2)], [1.0, math.sqrt(0.5), 1.0], 2, 3)
        rim = [e for e in ret.Edges() if e.IsClosed() and abs(e.Center().y - y2) < 1e-06]
        panel = cq.Face.makeFromWires(cq.Wire.assembleEdges(rim))
        sew = BRepBuilderAPI_Sewing(1e-06)
        for f in (band, ret, panel):
            sew.Add(f.wrapped)
        sew.Perform()
        skin = cq.Shape.cast(sew.SewedShape())
        hip = _box(-120, 120, -HIP_Y_HALF, HIP_Y_HALF, 60, HIP_Z_TOP)
        front = _box(FRONT_X_MIN, 120, -FRONT_Y_HALF, FRONT_Y_HALF, FRONT_Z_MIN, 260)
        trimmed = skin.cut(hip, front)
        mk = BRepOffsetAPI_MakeThickSolid()
        mk.MakeThickSolidBySimple(trimmed.wrapped, -WALL)
        mk.Build()
        wall = cq.Shape.cast(mk.Shape()).Solids()[0]
        wall = cq.Shape.cast(ShapeCustom.ConvertToBSpline_s(wall.wrapped, False, False, True, False))
        return _mirror_y(wall) if S > 0 else wall
    
    def seam_cut():
        slab = _box(-120, 120, _y(SEAM_GAP), _y(-1.0), 60, 260)
        keep = _box(-120, PATCH_X_MAX, _y(SEAM_GAP + 1), _y(-2.0), PATCH_Z_MIN, PATCH_Z_MAX)
        return slab.cut(keep)
    
    def top_service_cut():
        b = _box(SERVICE_X_MIN, 120, -SERVICE_Y_HALF, SERVICE_Y_HALF, SERVICE_Z_MIN, 260)
        edges = [e for e in b.Edges() if abs(e.Center().x - SERVICE_X_MIN) < 1e-06 and abs(e.endPoint().z - e.startPoint().z) > 1.0]
        return b.fillet(SERVICE_CORNER, edges)
    
    def boss(x, r, rnd):
        b = _cyl_y(x, BOSS_Z, r, Y_PANEL - WALL + 0.5, DECK_Y)
        if rnd > 0:
            edges = [e for e in b.Edges() if e.IsClosed() and abs(e.Center().y - _y(DECK_Y)) < 1e-06]
            b = b.fillet(rnd, edges)
        return b
    
    def locator_ring():
        ring = _cyl_y(FRONT_X, BOSS_Z, SPOT_R, RING_START, RING_END)
        return ring.cut(_box(FRONT_X - 10, SOCKET_FLAT_X, _y(DECK_Y), 0.0, BOSS_Z - 10, BOSS_Z + 10))
    
    def locator_socket():
        past = 0.2
        y_ch = RING_END + ENTRY_CHAMFER
        sock = _cyl_y(FRONT_X, BOSS_Z, SOCKET_R, DECK_Y, RING_END - past).intersect(_box(FRONT_X - 10, FRONT_X + 10, _y(60), 0.0, BOSS_Z - 10, BASE_TOP_Z))
        cone = _cone_in(FRONT_X, BOSS_Z, SOCKET_R, SOCKET_R + ENTRY_CHAMFER + past, y_ch, ENTRY_CHAMFER + past)
        roof = _halfspace((0.0, -S, -1.0), -(BASE_TOP_Z + y_ch) / math.sqrt(2.0), (FRONT_X - 10, min(_y(y_ch), _y(RING_END - past)), BOSS_Z - 10), (FRONT_X + 10, max(_y(y_ch), _y(RING_END - past)), BASE_TOP_Z + 5))
        return sock.fuse(cone.intersect(roof))
    
    def rear_lip():
        yz = [(LIP_START, BASE_TOP_Z), (LIP_END + LIP_LEAD, BASE_TOP_Z), (LIP_END, BASE_TOP_Z + LIP_LEAD), (LIP_END, BASE_TOP_Z + LIP_T), (LIP_START, BASE_TOP_Z + LIP_T)]
        w = cq.Wire.makePolygon([cq.Vector(LIP_X_MIN, _y(a), z) for a, z in yz], close=True)
        return cq.Solid.extrudeLinear(cq.Face.makeFromWires(w), cq.Vector(LIP_X_MAX - LIP_X_MIN, 0, 0))
    
    def collar():
        pts = [(COLLAR_RI, COLLAR_BASE_Y), (COLLAR_RF, COLLAR_BASE_Y), (COLLAR_RF, Y_PANEL), (COLLAR_RO, Y_PANEL + (COLLAR_RF - COLLAR_RO)), (COLLAR_RO, COLLAR_RIM_Y - COLLAR_CH), (COLLAR_RO - COLLAR_CH, COLLAR_RIM_Y), (COLLAR_RI + COLLAR_CH, COLLAR_RIM_Y), (COLLAR_RI, COLLAR_RIM_Y - COLLAR_CH)]
        w = cq.Wire.makePolygon([cq.Vector(SW_X + r, _y(a), SW_Z) for r, a in pts], close=True)
        return cq.Solid.revolve(cq.Face.makeFromWires(w), 360, cq.Vector(SW_X, 0, SW_Z), cq.Vector(SW_X, 1, SW_Z))
    model = Model()
    model.add('wall', thickened_wall(), name='Fair skin, trimmed, thickened 3 mm inward')
    model.add('seam', seam_cut(), op='cut', name='Seam gap with rear patch window')
    model.add('top_service', top_service_cut(), op='cut', name='Top service opening')
    model.add('rear_doorway', _box(-120, DOOR_X_MAX, -DOOR_Y_HALF, DOOR_Y_HALF, 60, DOOR_Z_TOP), op='cut', name='Rear battery doorway')
    model.add('rear_chamfer', _halfspace((CHAMFER_NX, 0.0, CHAMFER_NZ), CHAMFER_D, (-80, -DOOR_Y_HALF, 140), (-55, DOOR_Y_HALF, 152)), op='cut', name='Rear patch chamfer')
    model.add('corridor', _box(CORRIDOR_X_MIN, CORRIDOR_X_MAX, -CORRIDOR_Y_HALF, CORRIDOR_Y_HALF, 60, CORRIDOR_Z_TOP), op='cut', name='Carrier corridor')
    model.add('neck_notch', _halfspace((NOTCH_NX, 0.0, NOTCH_NZ), NOTCH_D, (NOTCH_X_MIN, -NECK_NOTCH_HALF_WIDTH, 120), (60, NECK_NOTCH_HALF_WIDTH, 200)), op='cut', name='Neck notch')
    model.add('boss_rear', boss(BOSS_REAR_X, BOSS_R, BOSS_ROUND), op='join', name='Rear screw boss')
    model.add('pad_rear', boss(PAD_X, PAD_R, 0.0), op='join', name='Rear screw pad')
    model.add('boss_front', boss(FRONT_X, BOSS_R, BOSS_ROUND), op='join', name='Front screw boss')
    model.add('spot_rear', _cyl_y(PAD_X, BOSS_Z, SPOT_R, DECK_Y, -1.0), op='cut', name='Rear deck spot face')
    model.add('spot_front', _cyl_y(FRONT_X, BOSS_Z, SPOT_R, DECK_Y, -1.0), op='cut', name='Front deck spot face')
    model.add('locator_ring', locator_ring(), op='join', name='Front locator ring')
    model.add('locator_lip', rear_lip(), op='join', name='Rear locator lip')
    model.add('locator_socket', locator_socket(), op='cut', name='Front locator socket')
    model.add('well_rear', _cyl_y(PAD_X, BOSS_Z, WELL_R, 60.0, REAR_WELL_FLOOR), op='cut', name='Rear head well')
    model.add('bore_rear', _cyl_y(PAD_X, BOSS_Z, BORE_R, 60.0, BORE_END), op='cut', name='Rear screw bore')
    model.add('well_front', _cyl_y(FRONT_X, BOSS_Z, WELL_R, 60.0, FRONT_WELL_FLOOR), op='cut', name='Front head well')
    model.add('bore_front', _cyl_y(FRONT_X, BOSS_Z, BORE_R, 60.0, BORE_END), op='cut', name='Front screw bore')
    if SWITCH_PORT:
        model.add('collar', collar(), op='join', name='Switch guard collar')
        model.add('switch_hole', _cyl_y(SW_X, SW_Z, SW_HOLE_R, 60.0, SW_RELIEF_Y - 0.5), op='cut', name='KCD1 snap-in hole')
        model.add('switch_key', _box(SW_X - SW_KEY_HALF, SW_X + SW_KEY_HALF, _y(60.0), _y(SW_RELIEF_Y - 0.5), SW_Z, SW_KEY_TOP), op='cut', name='KCD1 key')
        model.add('switch_relief', _cone_in(SW_X, SW_Z, SW_RELIEF_R, SW_RELIEF_R + SW_RELIEF_DEPTH, SW_RELIEF_Y, SW_RELIEF_DEPTH), op='cut', name='KCD1 45-degree relief')
    model.material('pla', color='#99deef')
    result = model.result()
    return to_build123d(model, 'right_shell_native')


@step(out="../../STEP/parts/right_shell_native.step")
def right_shell_native():
    return make()


if __name__ == "__main__":
    right_shell_native()
