"""top_head_shell (R05) - native rebuild of the Frankenduck head top cover (lid), robot world frame, mm.

Outer skin: a loft through 29 fitted cross-sections (x-stations), each a symmetric cubic B-spline (14 poles)
fitted to the R05 lid mesh (fit <= 0.14 mm in-plane). Wall, ribs, front band and bottom lip are true 3D
offsets of that skin. The front bosses are the measured primitives (the source lid's six
locating pins and rear block are removed: no Frankenduck part uses them); the two R05 screw
bosses are the exact values of build_top_shell_r05.py. The four cover screws thread into self-forming M3 holes
(three-ridge bore, no heat-set inserts) on the R05 screw axes.
"""
import math
import cadquery as cq
from OCP.BRep import BRep_Tool
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.GeomAbs import GeomAbs_C2
from OCP.GeomAPI import GeomAPI_PointsToBSpline
from OCP.GeomLProp import GeomLProp_SLProps
from OCP.TColgp import TColgp_Array1OfPnt
from OCP.gp import gp_Pnt
from source_api import Model

z_bottom = 274.503  # mm Bottom plane of the lid
wall = 2.295  # mm Wall thickness, normal to the outer skin
lip = 1.03  # mm Bottom lip thickness where the inner rebate is cut
rebate_top = 276.53  # mm Top of the inner bottom rebate
rebate_rear_end = 20.78  # mm Rear rebate segment ends at this X
rebate_front_start = 54.72  # mm Front rebate segment starts at this X
rebate_front_end = 108.405  # mm Front rebate segment ends at this X
rib_depth = 4.88  # mm Ribs 1 and 2 inner edge, depth below the outer skin
rib_bottom = 278.825  # mm Lowest Z of ribs 1 and 2
rib1_x0 = 17.013  # mm Rib 1 rear face X
rib1_x1 = 18.39  # mm Rib 1 front face X
rib2_x0 = 70.25  # mm Rib 2 rear face X
rib2_x1 = 71.627  # mm Rib 2 front face X
band_depth = 3.52  # mm Rib 3 and front band inner surface, depth below the outer skin
rib3_x0 = 106.785  # mm Rib 3 rear face X
rib3_x1 = 107.865  # mm Rib 3 front face X
rib3_bottom = 285.305  # mm Lowest Z of rib 3
band_x0 = 110.16  # mm Front band (face-plate groove front wall) starts at this X
front_slope = 0.13455  # mm Front cut plane: x = front_slope * z + front_x0 (dx per mm of z)
front_x0 = 79.6288  # mm Front cut plane X at z = 0
front_round = 2.5  # mm Round on the outer front edge
cut_xc = 35.237  # mm Side cutout arc centre X
cut_zc = 284.623  # mm Side cutout arc centre Z
cut_r = 12.827  # mm Side cutout arc radius (both sides)
cut_rear_k = 0.1765  # mm Side cutout rear edge: x = k * z + x0 (slope)
cut_rear_x0 = -28.013  # mm Side cutout rear edge X at z = 0
cut_front_k = -0.5773  # mm Side cutout front edge: x = k * z + x0 (slope)
cut_front_x0 = 214.368  # mm Side cutout front edge X at z = 0
cut_a_front = 29.9978  # deg Side cutout arc start angle (front tangent point), CCW from +X
cut_a_rear = 169.9904  # deg Side cutout arc end angle (rear tangent point)
cut_plus_r = 15.64  # mm Larger cutout arc radius on the +Y side only
cut_plus_zc = 284.64  # mm Larger +Y cutout arc centre Z
fboss_x = 102.06  # mm Front screw boss axis X (mirrored in Y)
fboss_y = 52.65  # mm Front screw boss axis |Y|
fboss_z0 = 278.825  # mm Front screw boss bottom Z
fboss_r0 = 4.39  # mm Front screw boss radius at its bottom
fboss_taper = 0.0348  # mm Front screw boss radius growth per mm of height
fboss_screw_x = 102.1  # mm Front boss screw axis X (the bottom shell's cover_front axis)
fboss_hole_top = 295.0  # mm Front boss screw hole top (start of the 118 deg drill tip)
rboss_z0 = 277.2  # mm Rear/side boss bottom face (screw entry)
rboss_hole_top = 287.5  # mm Rear/side boss screw hole top Z
# Self-forming M3 thread holes (no heat-set inserts): a bore slightly over the screw size with three ridges
# that the screw forms its thread into (Thomas Sanladerer, toms3d.org 2025-01-14). Values follow the FreeCAD
# hole-tool implementation of that design (FreeCAD PR 19149); tune after a print test.
sft_d = 3.0  # mm Screw nominal diameter (M3)
sft_margin = 0.15  # mm Bore oversize over the screw diameter
sft_ridge_h = 0.4  # mm Ridge height into the bore (ridge tips at d + margin - 2 x ridge)
sft_taper = 3.0  # mm Entry length over which the ridges grow from 0 to full height
sft_ridges = 3  # Number of ridges, evenly spaced
pin_hole_r = 0.8  # mm Locating pin hole radius

# Outer-skin sections: x-station and symmetric B-spline poles (y, z), fitted to the R05 mesh.
model = Model()
model.plane("st00", on="YZ", offset=-40.95, name="Station x=-40.95")
with model.sketch("sk00", on="st00", name="Section x=-40.95") as s:
    s.spline("sp", [(3.851, 273.003), (3.757, 273.425), (3.34, 273.654), (3.082, 274.86), (2.392, 275.396), (1.411, 275.742), (0.501, 275.835), (-0.501, 275.835), (-1.411, 275.742), (-2.392, 275.396), (-3.082, 274.86), (-3.34, 273.654), (-3.757, 273.425), (-3.851, 273.003)])
    s.line("ln", (-3.851, 273.003), (3.851, 273.003))
model.plane("st01", on="YZ", offset=-40.85, name="Station x=-40.85")
with model.sketch("sk01", on="st01", name="Section x=-40.85") as s:
    s.spline("sp", [(6.745, 273.003), (6.559, 273.126), (6.321, 274.863), (5.779, 275.996), (3.956, 276.783), (2.529, 277.181), (0.787, 277.31), (-0.787, 277.31), (-2.529, 277.181), (-3.956, 276.783), (-5.779, 275.996), (-6.321, 274.863), (-6.559, 273.126), (-6.745, 273.003)])
    s.line("ln", (-6.745, 273.003), (6.745, 273.003))
model.plane("st02", on="YZ", offset=-40.7, name="Station x=-40.7")
with model.sketch("sk02", on="st02", name="Section x=-40.7") as s:
    s.spline("sp", [(8.576, 273.003), (8.497, 273.903), (8.071, 275.008), (6.851, 277.03), (5.208, 278.32), (3.276, 279.512), (1.09, 279.494), (-1.09, 279.494), (-3.276, 279.512), (-5.208, 278.32), (-6.851, 277.03), (-8.071, 275.008), (-8.497, 273.903), (-8.576, 273.003)])
    s.line("ln", (-8.576, 273.003), (8.576, 273.003))
model.plane("st03", on="YZ", offset=-40.45, name="Station x=-40.45")
with model.sketch("sk03", on="st03", name="Section x=-40.45") as s:
    s.spline("sp", [(10.9, 273.003), (10.828, 273.924), (10.636, 275.886), (8.922, 278.109), (6.584, 279.945), (4.251, 281.039), (1.204, 281.993), (-1.204, 281.993), (-4.251, 281.039), (-6.584, 279.945), (-8.922, 278.109), (-10.636, 275.886), (-10.828, 273.924), (-10.9, 273.003)])
    s.line("ln", (-10.9, 273.003), (10.9, 273.003))
model.plane("st04", on="YZ", offset=-40.1, name="Station x=-40.1")
with model.sketch("sk04", on="st04", name="Section x=-40.1") as s:
    s.spline("sp", [(14.004, 273.003), (13.555, 273.959), (13.041, 276.655), (11.249, 279.417), (8.447, 282.133), (5.393, 283.454), (1.663, 284.471), (-1.663, 284.471), (-5.393, 283.454), (-8.447, 282.133), (-11.249, 279.417), (-13.041, 276.655), (-13.555, 273.959), (-14.004, 273.003)])
    s.line("ln", (-14.004, 273.003), (14.004, 273.003))
model.plane("st05", on="YZ", offset=-39.6, name="Station x=-39.6")
with model.sketch("sk05", on="st05", name="Section x=-39.6") as s:
    s.spline("sp", [(16.443, 273.003), (16.496, 274.48), (16.03, 277.389), (13.515, 281.034), (10.164, 283.95), (6.452, 286.013), (2.072, 287.426), (-2.072, 287.426), (-6.452, 286.013), (-10.164, 283.95), (-13.515, 281.034), (-16.03, 277.389), (-16.496, 274.48), (-16.443, 273.003)])
    s.line("ln", (-16.443, 273.003), (16.443, 273.003))
model.plane("st06", on="YZ", offset=-38.9, name="Station x=-38.9")
with model.sketch("sk06", on="st06", name="Section x=-38.9") as s:
    s.spline("sp", [(19.875, 273.003), (19.286, 274.753), (19.326, 278.226), (15.983, 282.719), (12.384, 286.302), (7.7, 289.185), (2.726, 290.607), (-2.726, 290.607), (-7.7, 289.185), (-12.384, 286.302), (-15.983, 282.719), (-19.326, 278.226), (-19.286, 274.753), (-19.875, 273.003)])
    s.line("ln", (-19.875, 273.003), (19.875, 273.003))
model.plane("st07", on="YZ", offset=-38.0, name="Station x=-38.0")
with model.sketch("sk07", on="st07", name="Section x=-38.0") as s:
    s.spline("sp", [(22.865, 273.003), (22.823, 275.13), (22.199, 279.123), (18.855, 284.564), (14.43, 288.855), (9.062, 291.909), (3.038, 293.885), (-3.038, 293.885), (-9.062, 291.909), (-14.43, 288.855), (-18.855, 284.564), (-22.199, 279.123), (-22.823, 275.13), (-22.865, 273.003)])
    s.line("ln", (-22.865, 273.003), (22.865, 273.003))
model.plane("st08", on="YZ", offset=-36.8, name="Station x=-36.8")
with model.sketch("sk08", on="st08", name="Section x=-36.8") as s:
    s.spline("sp", [(26.153, 273.003), (26.224, 275.431), (25.432, 280.134), (21.718, 286.44), (16.598, 291.438), (10.55, 295.346), (3.598, 297.425), (-3.598, 297.425), (-10.55, 295.346), (-16.598, 291.438), (-21.718, 286.44), (-25.432, 280.134), (-26.224, 275.431), (-26.153, 273.003)])
    s.line("ln", (-26.153, 273.003), (26.153, 273.003))
model.plane("st09", on="YZ", offset=-35.3, name="Station x=-35.3")
with model.sketch("sk09", on="st09", name="Section x=-35.3") as s:
    s.spline("sp", [(29.602, 273.003), (29.52, 275.73), (28.703, 281.143), (24.531, 288.267), (18.893, 294.195), (11.994, 298.58), (4.092, 300.959), (-4.092, 300.959), (-11.994, 298.58), (-18.893, 294.195), (-24.531, 288.267), (-28.703, 281.143), (-29.52, 275.73), (-29.602, 273.003)])
    s.line("ln", (-29.602, 273.003), (29.602, 273.003))
model.plane("st10", on="YZ", offset=-33.5, name="Station x=-33.5")
with model.sketch("sk10", on="st10", name="Section x=-33.5") as s:
    s.spline("sp", [(32.736, 273.003), (32.792, 276.053), (31.755, 282.082), (27.309, 290.142), (21.136, 296.868), (13.392, 301.812), (4.595, 304.388), (-4.595, 304.388), (-13.392, 301.812), (-21.136, 296.868), (-27.309, 290.142), (-31.755, 282.082), (-32.792, 276.053), (-32.736, 273.003)])
    s.line("ln", (-32.736, 273.003), (32.736, 273.003))
model.plane("st11", on="YZ", offset=-31.2, name="Station x=-31.2")
with model.sketch("sk11", on="st11", name="Section x=-31.2") as s:
    s.spline("sp", [(35.945, 273.003), (36.047, 276.388), (34.914, 283.06), (30.171, 292.103), (23.45, 299.642), (14.858, 305.21), (5.115, 307.913), (-5.115, 307.913), (-14.858, 305.21), (-23.45, 299.642), (-30.171, 292.103), (-34.914, 283.06), (-36.047, 276.388), (-35.945, 273.003)])
    s.line("ln", (-35.945, 273.003), (35.945, 273.003))
model.plane("st12", on="YZ", offset=-28.5, name="Station x=-28.5")
with model.sketch("sk12", on="st12", name="Section x=-28.5") as s:
    s.spline("sp", [(39.017, 273.003), (39.01, 276.7), (37.871, 283.983), (32.863, 293.981), (25.676, 302.331), (16.268, 308.475), (5.591, 311.247), (-5.591, 311.247), (-16.268, 308.475), (-25.676, 302.331), (-32.863, 293.981), (-37.871, 283.983), (-39.01, 276.7), (-39.017, 273.003)])
    s.line("ln", (-39.017, 273.003), (39.017, 273.003))
model.plane("st13", on="YZ", offset=-25.3, name="Station x=-25.3")
with model.sketch("sk13", on="st13", name="Section x=-25.3") as s:
    s.spline("sp", [(41.821, 273.003), (41.813, 277.018), (40.602, 284.837), (35.428, 295.809), (27.852, 304.945), (17.59, 311.607), (6.071, 314.408), (-6.071, 314.408), (-17.59, 311.607), (-27.852, 304.945), (-35.428, 295.809), (-40.602, 284.837), (-41.813, 277.018), (-41.821, 273.003)])
    s.line("ln", (-41.821, 273.003), (41.821, 273.003))
model.plane("st14", on="YZ", offset=-21.5, name="Station x=-21.5")
with model.sketch("sk14", on="st14", name="Section x=-21.5") as s:
    s.spline("sp", [(44.404, 273.003), (44.369, 277.296), (43.118, 285.674), (37.865, 297.533), (29.892, 307.467), (18.903, 314.599), (6.465, 317.352), (-6.465, 317.352), (-18.903, 314.599), (-29.892, 307.467), (-37.865, 297.533), (-43.118, 285.674), (-44.369, 277.296), (-44.404, 273.003)])
    s.line("ln", (-44.404, 273.003), (44.404, 273.003))
model.plane("st15", on="YZ", offset=-17.0, name="Station x=-17.0")
with model.sketch("sk15", on="st15", name="Section x=-17.0") as s:
    s.spline("sp", [(46.725, 273.003), (46.685, 277.536), (45.406, 286.489), (40.136, 299.111), (31.831, 309.911), (20.13, 317.316), (6.816, 320.073), (-6.816, 320.073), (-20.13, 317.316), (-31.831, 309.911), (-40.136, 299.111), (-45.406, 286.489), (-46.685, 277.536), (-46.725, 273.003)])
    s.line("ln", (-46.725, 273.003), (46.725, 273.003))
model.plane("st16", on="YZ", offset=-11.5, name="Station x=-11.5")
with model.sketch("sk16", on="st16", name="Section x=-11.5") as s:
    s.spline("sp", [(48.869, 273.003), (48.809, 277.794), (47.488, 287.211), (42.231, 300.656), (33.747, 312.27), (21.298, 320.0), (7.164, 322.553), (-7.164, 322.553), (-21.298, 320.0), (-33.747, 312.27), (-42.231, 300.656), (-47.488, 287.211), (-48.809, 277.794), (-48.869, 273.003)])
    s.line("ln", (-48.869, 273.003), (48.869, 273.003))
model.plane("st17", on="YZ", offset=-5.0, name="Station x=-5.0")
with model.sketch("sk17", on="st17", name="Section x=-5.0") as s:
    s.spline("sp", [(50.764, 273.003), (50.675, 277.991), (49.313, 287.913), (44.189, 302.003), (35.483, 314.473), (22.363, 322.279), (7.446, 324.714), (-7.446, 324.714), (-22.363, 322.279), (-35.483, 314.473), (-44.189, 302.003), (-49.313, 287.913), (-50.675, 277.991), (-50.764, 273.003)])
    s.line("ln", (-50.764, 273.003), (50.764, 273.003))
model.plane("st18", on="YZ", offset=3.0, name="Station x=3.0")
with model.sketch("sk18", on="st18", name="Section x=3.0") as s:
    s.spline("sp", [(52.448, 273.003), (52.35, 278.175), (50.984, 288.551), (45.949, 303.258), (37.144, 316.501), (23.316, 324.42), (7.748, 326.591), (-7.748, 326.591), (-23.316, 324.42), (-37.144, 316.501), (-45.949, 303.258), (-50.984, 288.551), (-52.35, 278.175), (-52.448, 273.003)])
    s.line("ln", (-52.448, 273.003), (52.448, 273.003))
model.plane("st19", on="YZ", offset=12.0, name="Station x=12.0")
with model.sketch("sk19", on="st19", name="Section x=12.0") as s:
    s.spline("sp", [(53.749, 273.003), (53.762, 278.333), (52.264, 289.093), (47.453, 304.301), (38.542, 318.282), (24.146, 326.136), (7.956, 328.149), (-7.956, 328.149), (-24.146, 326.136), (-38.542, 318.282), (-47.453, 304.301), (-52.264, 289.093), (-53.762, 278.333), (-53.749, 273.003)])
    s.line("ln", (-53.749, 273.003), (53.749, 273.003))
model.plane("st20", on="YZ", offset=22.0, name="Station x=22.0")
with model.sketch("sk20", on="st20", name="Section x=22.0") as s:
    s.spline("sp", [(55.037, 273.003), (54.482, 277.618), (54.06, 286.598), (49.61, 302.895), (41.073, 318.664), (25.878, 327.342), (8.457, 329.388), (-8.457, 329.388), (-25.878, 327.342), (-41.073, 318.664), (-49.61, 302.895), (-54.06, 286.598), (-54.482, 277.618), (-55.037, 273.003)])
    s.line("ln", (-55.037, 273.003), (55.037, 273.003))
model.plane("st21", on="YZ", offset=33.0, name="Station x=33.0")
with model.sketch("sk21", on="st21", name="Section x=33.0") as s:
    s.spline("sp", [(55.988, 273.003), (55.64, 277.713), (54.56, 288.903), (50.339, 304.74), (41.404, 320.584), (25.949, 328.667), (8.453, 330.522), (-8.453, 330.522), (-25.949, 328.667), (-41.404, 320.584), (-50.339, 304.74), (-54.56, 288.903), (-55.64, 277.713), (-55.988, 273.003)])
    s.line("ln", (-55.988, 273.003), (55.988, 273.003))
model.plane("st22", on="YZ", offset=45.0, name="Station x=45.0")
with model.sketch("sk22", on="st22", name="Section x=45.0") as s:
    s.spline("sp", [(57.027, 273.003), (56.808, 278.427), (55.356, 290.225), (50.922, 306.397), (41.902, 322.117), (25.968, 329.851), (8.57, 331.514), (-8.57, 331.514), (-25.968, 329.851), (-41.902, 322.117), (-50.922, 306.397), (-55.356, 290.225), (-56.808, 278.427), (-57.027, 273.003)])
    s.line("ln", (-57.027, 273.003), (57.027, 273.003))
model.plane("st23", on="YZ", offset=57.0, name="Station x=57.0")
with model.sketch("sk23", on="st23", name="Section x=57.0") as s:
    s.spline("sp", [(58.065, 273.003), (57.79, 278.82), (56.37, 290.525), (51.588, 307.248), (42.676, 323.061), (26.257, 330.781), (8.738, 332.418), (-8.738, 332.418), (-26.257, 330.781), (-42.676, 323.061), (-51.588, 307.248), (-56.37, 290.525), (-57.79, 278.82), (-58.065, 273.003)])
    s.line("ln", (-58.065, 273.003), (58.065, 273.003))
model.plane("st24", on="YZ", offset=72.0, name="Station x=72.0")
with model.sketch("sk24", on="st24", name="Section x=72.0") as s:
    s.spline("sp", [(59.148, 273.003), (58.876, 278.917), (57.396, 290.927), (52.7, 307.909), (43.512, 324.206), (26.874, 331.923), (8.843, 333.55), (-8.843, 333.55), (-26.874, 331.923), (-43.512, 324.206), (-52.7, 307.909), (-57.396, 290.927), (-58.876, 278.917), (-59.148, 273.003)])
    s.line("ln", (-59.148, 273.003), (59.148, 273.003))
model.plane("st25", on="YZ", offset=88.0, name="Station x=88.0")
with model.sketch("sk25", on="st25", name="Section x=88.0") as s:
    s.spline("sp", [(60.284, 273.003), (60.026, 279.02), (58.484, 291.273), (53.69, 308.534), (44.272, 325.144), (27.404, 333.031), (8.982, 334.692), (-8.982, 334.692), (-27.404, 333.031), (-44.272, 325.144), (-53.69, 308.534), (-58.484, 291.273), (-60.026, 279.02), (-60.284, 273.003)])
    s.line("ln", (-60.284, 273.003), (60.284, 273.003))
model.plane("st26", on="YZ", offset=104.0, name="Station x=104.0")
with model.sketch("sk26", on="st26", name="Section x=104.0") as s:
    s.spline("sp", [(61.398, 273.003), (61.173, 279.13), (59.565, 291.584), (54.619, 309.147), (45.012, 325.985), (27.877, 334.118), (9.147, 335.798), (-9.147, 335.798), (-27.877, 334.118), (-45.012, 325.985), (-54.619, 309.147), (-59.565, 291.584), (-61.173, 279.13), (-61.398, 273.003)])
    s.line("ln", (-61.398, 273.003), (61.398, 273.003))
model.plane("st27", on="YZ", offset=116.0, name="Station x=116.0")
with model.sketch("sk27", on="st27", name="Section x=116.0") as s:
    s.spline("sp", [(62.244, 273.003), (62.013, 279.219), (60.37, 291.802), (55.31, 309.616), (45.571, 326.611), (28.228, 334.933), (9.274, 336.636), (-9.274, 336.636), (-28.228, 334.933), (-45.571, 326.611), (-55.31, 309.616), (-60.37, 291.802), (-62.013, 279.219), (-62.244, 273.003)])
    s.line("ln", (-62.244, 273.003), (62.244, 273.003))
model.plane("st28", on="YZ", offset=128.0, name="Station x=128.0")
with model.sketch("sk28", on="st28", name="Section x=128.0") as s:
    s.spline("sp", [(62.867, 273.003), (62.604, 279.282), (60.971, 291.958), (55.806, 309.956), (45.982, 327.066), (28.48, 335.527), (9.37, 337.253), (-9.37, 337.253), (-28.48, 335.527), (-45.982, 327.066), (-55.806, 309.956), (-60.971, 291.958), (-62.604, 279.282), (-62.867, 273.003)])
    s.line("ln", (-62.867, 273.003), (62.867, 273.003))
model.loft("skin", ["sk00", "sk01", "sk02", "sk03", "sk04", "sk05", "sk06", "sk07", "sk08", "sk09", "sk10", "sk11", "sk12", "sk13", "sk14", "sk15", "sk16", "sk17", "sk18", "sk19", "sk20", "sk21", "sk22", "sk23", "sk24", "sk25", "sk26", "sk27", "sk28"], body="core", name="Outer skin")


def _solid(shape):
    return cq.Shape.cast(shape).clean()


def _common(a, b):
    return _solid(BRepAlgoAPI_Common(a.wrapped, b.wrapped).Shape())


def _cut(a, b):
    return _solid(BRepAlgoAPI_Cut(a.wrapped, b.wrapped).Shape())


def _fuse(a, b):
    return _solid(BRepAlgoAPI_Fuse(a.wrapped, b.wrapped).Shape())


def _union(shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = _fuse(out, s)
    return out


def _box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def _cyl(x, y, r, z0, z1):
    return cq.Solid.makeCylinder(r, z1 - z0, cq.Vector(x, y, z0))


def _cone(x, y, r0, r1, z0, z1):
    return cq.Solid.makeCone(r0, r1, z1 - z0, cq.Vector(x, y, z0))


class _Offset:
    """Planar sections (x = const) of the inward 3D offset of the outer-skin surface."""

    def __init__(self, face, core):
        self.surf = BRep_Tool.Surface_s(face.wrapped)
        u0, u1, v0, v1 = self.surf.Bounds()
        self.u0, self.u1, self.v0, self.v1 = u0, u1, v0, v1
        q = self._pt(0.5 * (u0 + u1), 0.5 * (v0 + v1), 1.0)
        self.sign = 1.0 if core.isInside(cq.Vector(*q)) else -1.0  # makes positive depth point inward

    def _pt(self, u, v, depth):
        pr = GeomLProp_SLProps(self.surf, u, v, 1, 1e-9)
        p, n = pr.Value(), pr.Normal()
        s = getattr(self, "sign", 1.0) * depth
        return (p.X() + s * n.X(), p.Y() + s * n.Y(), p.Z() + s * n.Z())

    def section(self, x, depth, n=72):
        pts = []
        for i in range(n + 1):
            u = self.u0 + (self.u1 - self.u0) * i / n
            lo, hi = self.v0, self.v1
            flo = self._pt(u, lo, depth)[0] - x
            fhi = self._pt(u, hi, depth)[0] - x
            if flo * fhi > 0:
                continue
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                fm = self._pt(u, mid, depth)[0] - x
                if (fm > 0) == (fhi > 0):
                    hi, fhi = mid, fm
                else:
                    lo, flo = mid, fm
            p = self._pt(u, 0.5 * (lo + hi), depth)
            pts.append((x, p[1], p[2]))
        return pts

    def wire(self, x, depth, z_low):
        pts = [p for p in self.section(x, depth) if p[2] >= z_low]
        arr = TColgp_Array1OfPnt(1, len(pts))
        for i, p in enumerate(pts):
            arr.SetValue(i + 1, gp_Pnt(*p))
        curve = GeomAPI_PointsToBSpline(arr, 3, 8, GeomAbs_C2, 0.002).Curve()
        edge = cq.Edge(BRepBuilderAPI_MakeEdge(curve).Edge())
        a, b = edge.startPoint(), edge.endPoint()
        lo = z_bottom - 3.0
        a2, b2 = cq.Vector(a.x, a.y, lo), cq.Vector(b.x, b.y, lo)
        return cq.Wire.assembleEdges([edge, cq.Edge.makeLine(b, b2), cq.Edge.makeLine(b2, a2), cq.Edge.makeLine(a2, a)])

    def core(self, stations, depth, z_low):
        maker = BRepOffsetAPI_ThruSections(True, False)
        for x in stations:
            maker.AddWire(self.wire(x, depth, z_low).wrapped)
        maker.Build()
        return _solid(maker.Shape())


def _additions():
    adds = []
    for side in (1, -1):  # front screw bosses (tapered) with the neck to the side wall
        y = side * fboss_y
        adds.append(_cone(fboss_x, y, fboss_r0, fboss_r0 + fboss_taper * (320.0 - fboss_z0), fboss_z0, 320.0))
        adds.append(_box(100.94, 102.44, min(y, side * 62.0), max(y, side * 62.0), fboss_z0, 320.0))
    # the six locating pins of the source lid are removed: no Frankenduck part locates on them (user, 2026-10-04)
    # Obsolete MAX98357A pad removed; audio is now mounted to the front roll bulkhead.
    # the source lid's rear block (tab, post and rib on the rear wall) is removed: unused (user, 2026-10-04)
    adds += [  # R05 cover screw bosses with ribs into the wall (build_top_shell_r05.py)
        _cyl(-30.0, -26.0, 4.5, 277.2, 288.0), _box(-35.5, -30.0, -29.0, -23.0, 279.0, 286.0),
        _cyl(-1.5, 43.5, 4.5, 277.2, 290.0), _box(-6.0, 3.0, 43.5, 49.8, 279.0, 286.0)]
    return adds


def lid_body(core):
    skin = max(core.Faces(), key=lambda f: f.Area())
    off = _Offset(skin, core)
    zl = z_bottom - 1.0
    wall_x = [-38.5, -38.0, -36.8, -35.3, -33.5, -31.2, -28.5, -25.3, -21.5, -17.0, -11.5, -5.0, 3.0, 12.0, 22.0, 33.0, 45.0, 57.0, 72.0, 88.0, 104.0, 116.0, 126.0]
    cavity = off.core(wall_x, wall, zl)  # inside of the 2.295 mm wall
    lip_x = [-39.6, -38.9, -38.0, -36.8, -35.3, -33.5, -31.2, -28.5, -25.3, -21.5, -17.0, -11.5, -5.0, 3.0, 12.0, 22.0, 33.0, 45.0, 57.0, 72.0, 88.0, 104.0, 110.0]
    rebate = _union([_box(-60.0, rebate_rear_end, -80, 80, zl - 2.0, rebate_top), _box(rebate_front_start, rebate_front_end, -80, 80, zl - 2.0, rebate_top)])
    cavity = _fuse(cavity, _common(off.core(lip_x, lip, zl), rebate))  # inner bottom rebate
    for x0, x1, zb, depth in ((rib1_x0, rib1_x1, rib_bottom, rib_depth), (rib2_x0, rib2_x1, rib_bottom, rib_depth), (rib3_x0, rib3_x1, rib3_bottom, band_depth)):
        slab = _box(x0, x1, -80, 80, zb, 345)  # ribs: inside the slab only the deeper offset stays empty
        cavity = _fuse(_cut(cavity, slab), _common(off.core([x0 - 0.5, x1 + 0.5], depth, zl), slab))
    band = _box(band_x0, 140.0, -80, 80, zl - 2.0, 345)
    cavity = _fuse(_cut(cavity, band), _common(off.core([band_x0 - 1.0, 116.0, 126.0], band_depth, zl), band))
    cavity = _cut(cavity, _union(_additions()))
    body = _cut(core, cavity)
    body = _cut(body, _box(-60.0, 140.0, -80.0, 80.0, z_bottom - 10.0, z_bottom))
    front = cq.Workplane("XZ").polyline([(front_slope * 260 + front_x0, 260), (front_slope * 350 + front_x0, 350), (200, 350), (200, 260)]).close().extrude(80, both=True).val()
    body = _cut(body, front)
    face = [f for f in body.Faces() if f.geomType() == "PLANE" and f.Center().x > 110 and abs(f.normalAt().x - 0.991) < 0.01][0]
    edges = [e for e in face.Edges() if skin.distance(cq.Vertex.makeVertex(*e.positionAt(0.5).toTuple())) < 1e-3]
    return body.fillet(front_round, edges)


def self_forming_hole(x, y, z_face, z_end, d=sft_d, margin=sft_margin, ridge_h=sft_ridge_h, taper=sft_taper,
                      ridges=sft_ridges, overshoot=2.0):
    """Cutting tool for a self-forming thread hole on a vertical axis at (x, y).

    The screw enters at z_face and the hole runs to z_end (either direction). Bore radius r = (d + margin) / 2;
    each ridge is what a large cylinder (radius R = r + 5) leaves when it overlaps the bore by ridge_h, so the
    ridge tips sit at r - ridge_h. Over the first `taper` mm from z_face the overlap grows from 0 to ridge_h.
    The bore also runs `overshoot` mm outside the entry face so the cut opens cleanly.
    """
    s = 1.0 if z_end > z_face else -1.0
    r = (d + margin) / 2.0
    big_r = r + 5.0
    off = big_r + r - ridge_h
    z_out = z_face - s * overshoot
    z_lo, z_hi = min(z_out, z_end), max(z_out, z_end)
    tool = cq.Solid.makeCylinder(r, z_hi - z_lo, cq.Vector(x, y, z_lo))
    dz = cq.Vector(0, 0, s)
    for i in range(int(ridges)):
        a = 2.0 * math.pi * i / ridges
        cx, cy = x + off * math.cos(a), y + off * math.sin(a)
        entry = cq.Solid.makeCylinder(big_r - ridge_h, overshoot, cq.Vector(cx, cy, z_out), dz)
        ramp = cq.Solid.makeCone(big_r - ridge_h, big_r, taper, cq.Vector(cx, cy, z_face), dz)
        full = cq.Solid.makeCylinder(big_r, abs(z_end - z_face) - taper, cq.Vector(cx, cy, z_face + s * taper), dz)
        tool = tool.cut(entry.fuse(ramp).fuse(full).clean())
    return tool.clean()


model.add("lid", lid_body(model.bodies.pop("core")), body="lid", name="Wall, ribs, bosses, rebate, front round")
for side in (1, -1):
    tag = "p" if side > 0 else "m"
    model.add(f"fhole_{tag}", self_forming_hole(fboss_screw_x, side * fboss_y, fboss_z0, fboss_hole_top), op="cut",
              body="lid", name="Front boss self-forming M3 hole")
    model.cone(f"ftip_{tag}", (sft_d + sft_margin) / 2.0, 0.0, (sft_d + sft_margin) / 2.0 / math.tan(math.radians(59.0)),
               at=(fboss_screw_x, side * fboss_y, fboss_hole_top), op="cut", body="lid", name="Front boss drill tip")
model.add("rhole_a", self_forming_hole(-30.0, -26.0, rboss_z0, rboss_hole_top), op="cut", body="lid",
          name="Rear boss self-forming M3 hole")
model.add("rhole_b", self_forming_hole(-1.5, 43.5, rboss_z0, rboss_hole_top), op="cut", body="lid",
          name="Side boss self-forming M3 hole")
with model.sketch("cutout_sk", on="XZ", name="Side cutout profile") as s:
    s.line("rear", (cut_rear_k * 271.503 + cut_rear_x0, 271.503), (cut_xc + cut_r * math.cos(math.radians(cut_a_rear)), cut_zc + cut_r * math.sin(math.radians(cut_a_rear))))
    s.arc("top", (cut_xc, cut_zc), cut_r, cut_a_front, cut_a_rear)
    s.line("front", (cut_xc + cut_r * math.cos(math.radians(cut_a_front)), cut_zc + cut_r * math.sin(math.radians(cut_a_front))), (cut_front_k * 271.503 + cut_front_x0, 271.503))
    s.line("bottom", (cut_front_k * 271.503 + cut_front_x0, 271.503), (cut_rear_k * 271.503 + cut_rear_x0, 271.503))
model.extrude("cutout", "cutout_sk", distance=140.0, both=True, op="cut", body="lid", name="Side cutouts")
model.cylinder("cutout_plus", cut_plus_r, 80.0, at=(cut_xc, 0.0, cut_plus_zc), direction=(0, 1, 0), op="cut", body="lid", name="Larger +Y cutout arc")
model.box("recess_plus", 5.66, 10.085, 8.4, at=(24.97, 45.0425, 294.2), op="cut", body="lid", name="+Y cutout inner recess")
# Rigid print: provisional PLA library properties; existing local profile uses Bambu PLA Matte.
model.material("pla", color="#99deef")
result = model.result()

