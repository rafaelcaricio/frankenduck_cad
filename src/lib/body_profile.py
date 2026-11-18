"""Shared original body silhouette, in native shell X/Z millimetres.

World placement rotates native X to -Y. Both shells and the LiPo front cowl
use these exact periodic cubic B-spline poles; no traced approximation.
"""
PROFILE_CTL_XZ = [
    (-61.969364, 141.721413),
    (-60.769652, 133.872594),
    (-57.276993, 126.803988),
    (-50.794014, 122.358513),
    (-43.022885, 120.769484),
    (-35.077182, 120.67624),
    (-27.125195, 120.697049),
    (-19.174847, 120.866402),
    (-11.224499, 121.035755),
    (-3.274151, 121.205108),
    (4.676196, 121.374461),
    (12.626544, 121.543815),
    (20.576892, 121.713168),
    (26.636799, 126.072967),
    (28.103164, 133.853673),
    (28.415338, 141.798651),
    (28.519343, 149.748904),
    (26.829561, 157.478466),
    (22.453743, 164.065265),
    (16.146531, 168.833552),
    (8.767016, 171.7796),
    (1.031361, 173.596575),
    (-6.854049, 174.594652),
    (-14.797595, 174.91515),
    (-22.749282, 174.893843),
    (-30.701087, 174.819608),
    (-38.641541, 174.419028),
    (-46.516012, 173.371864),
    (-53.96208, 170.678255),
    (-59.525821, 165.128391),
    (-61.936164, 157.59931),
    (-62.307761, 149.666182),
]


def world_profile_prism(x0=-50.0, width=100.0):
    """Extrude the body's longitudinal outline across world X."""
    import cadquery as cq
    from OCP.Geom import Geom_BSplineCurve
    from OCP.TColgp import TColgp_Array1OfPnt
    from OCP.TColStd import TColStd_Array1OfReal, TColStd_Array1OfInteger
    from OCP.gp import gp_Pnt
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
    n = len(PROFILE_CTL_XZ)
    poles = TColgp_Array1OfPnt(1, n)
    knots = TColStd_Array1OfReal(1, n+1)
    mults = TColStd_Array1OfInteger(1, n+1)
    for i, (x, z) in enumerate(PROFILE_CTL_XZ, 1):
        poles.SetValue(i, gp_Pnt(x0, -x, z))
    for i in range(n+1):
        knots.SetValue(i+1, float(i))
        mults.SetValue(i+1, 1)
    curve = Geom_BSplineCurve(poles, knots, mults, 3, True)
    edge = cq.Edge(BRepBuilderAPI_MakeEdge(curve).Edge())
    face = cq.Face.makeFromWires(cq.Wire.assembleEdges([edge]))
    return cq.Solid.extrudeLinear(face, cq.Vector(width, 0, 0))
