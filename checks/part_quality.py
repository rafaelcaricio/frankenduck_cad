"""Quality checks for one printed part: geometry, edges, printing, fasteners and holes.

Each checker first runs on small fixture parts with a known error and must find
that error, and must find nothing on a clean fixture, before it judges a real
part (self_test). Accepted problems are named and carry a box in the part frame.
The box must hug the findings it covers, so a later shape change cannot hide a
new problem inside it. Units are millimetres.
"""
from pathlib import Path
import json
import math
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
AREA_MARGIN = 2.0      # an accepted box may reach this far past the findings it covers
AREA_MAX_SIDE = 40.0   # and no accepted box side may be longer
TOLERANCE = 1e-5       # mm3; numerical overlap of coincident faces


# Shared geometry helpers

def _cq(shape):
    import cadquery as cq
    return cq.Shape.cast(getattr(shape, 'wrapped', shape))


def box(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    b = Bnd_Box()
    BRepBndLib.AddOptimal_s(getattr(shape, 'wrapped', shape), b, False, False)
    return [round(v, 4) for v in b.Get()]


def _grow(b, d):
    return [b[0] - d, b[1] - d, b[2] - d, b[3] + d, b[4] + d, b[5] + d]


def _inside(inner, outer, eps=1e-6):
    return all(inner[i] >= outer[i] - eps for i in range(3)) and all(inner[i] <= outer[i] + eps for i in range(3, 6))


def _union(boxes):
    return [min(b[i] for b in boxes) for i in range(3)] + [max(b[i] for b in boxes) for i in range(3, 6)]


def _volume(shape):
    """Volume by adaptive integration; the fixed-order default is off by about 0.5 % on dense B-spline faces."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(getattr(shape, 'wrapped', shape), p, 1e-6)
    return p.Mass()


def _map(shape, kind, parent=None):
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_IndexedMapOfShape, TopTools_IndexedDataMapOfShapeListOfShape
    if parent is None:
        m = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(shape, kind, m)
        return [m.FindKey(i) for i in range(1, m.Extent() + 1)]
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape, kind, parent, m)
    return [(m.FindKey(i), list(m.FindFromIndex(i))) for i in range(1, m.Extent() + 1)]


def _length(edge):
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_AbscissaPoint
    return GCPnts_AbscissaPoint.Length_s(BRepAdaptor_Curve(edge))


def _area(face):
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    p = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, p)
    return p.Mass()


def mesh(shape, deflection=0.02, angle=0.2):
    """Triangles of the part with outward normals; checked against the B-rep volume."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    s = getattr(shape, 'wrapped', shape)
    BRepMesh_IncrementalMesh(s, deflection, False, angle, True)
    points, triangles, offset = [], [], 0
    for face in _map(s, TopAbs_FACE):
        face = TopoDS.Face_s(face)
        location = TopLoc_Location()
        t = BRep_Tool.Triangulation_s(face, location)
        assert t is not None, 'face without triangulation'
        trsf = location.Transformation()
        for i in range(1, t.NbNodes() + 1):
            p = t.Node(i).Transformed(trsf)
            points.append((p.X(), p.Y(), p.Z()))
        reverse = face.Orientation() == TopAbs_REVERSED
        for i in range(1, t.NbTriangles() + 1):
            a, b, c = t.Triangle(i).Get()
            triangles.append((offset + a - 1, offset + c - 1, offset + b - 1) if reverse else
                             (offset + a - 1, offset + b - 1, offset + c - 1))
        offset += t.NbNodes()
    v, f = np.array(points), np.array(triangles)
    cross = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    area = np.linalg.norm(cross, axis=1) / 2
    keep = area > 1e-12
    f, cross, area = f[keep], cross[keep], area[keep]
    normal = cross / (2 * area)[:, None]
    signed = float(np.einsum('ij,ij->i', v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum() / 6)
    solid = _volume(_cq(shape))
    assert signed > 0 and abs(signed - solid) <= 0.005 * solid, f'mesh volume {signed} differs from solid {solid}'
    return dict(v=v, f=f, area=area, normal=normal, mesh_volume_mm3=signed, solid_volume_mm3=solid)


def _clusters(points, link):
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    if not len(points):
        return np.zeros(0, int)
    pairs = cKDTree(points).query_pairs(link, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(points),) * 2)
    return connected_components(graph, directed=False)[1]


# Geometry: short edges, small and narrow faces, wall thickness, symmetry

def topology(shape, short_edge, small_face, narrow_face):
    """Edges shorter than short_edge; faces under small_face mm2 or narrower than narrow_face (2 x area / perimeter)."""
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.TopoDS import TopoDS
    s = getattr(shape, 'wrapped', shape)
    findings = []
    for edge in _map(s, TopAbs_EDGE):
        edge = TopoDS.Edge_s(edge)
        if BRep_Tool.Degenerated_s(edge):
            continue
        length = _length(edge)
        if length < short_edge:
            findings.append(dict(kind='short edge', length_mm=length, box=box(edge)))
    for face in _map(s, TopAbs_FACE):
        face = TopoDS.Face_s(face)
        area = _area(face)
        perimeter = sum(_length(TopoDS.Edge_s(e)) for e in _map(face, TopAbs_EDGE)
                        if not BRep_Tool.Degenerated_s(TopoDS.Edge_s(e)))
        width = 2 * area / perimeter if perimeter else 0.0
        if area < small_face:
            findings.append(dict(kind='small face', area_mm2=area, box=box(face)))
        elif width < narrow_face:
            findings.append(dict(kind='narrow face', width_mm=width, area_mm2=area, box=box(face)))
    return findings


GRAZE_DEG = 25.0      # thickness rays ignore faces they leave through at less than this angle
SLIVER_MM2 = 1e-4     # thickness samples skip triangles smaller than this


def thickness(m, limit, spacing=0.5, seed=1):
    """Wall thickness by rays cast inward from area-weighted surface samples.

    Each sample's ray runs against its outward normal to the first surface it
    meets, up to the length of the part's box diagonal. Samples under limit are
    grouped (links of 2 x spacing) into findings. A ray that never meets the
    surface means a leaking mesh or a flipped normal and is a finding too.

    A wall has two sides that face away from each other, so a ray crosses it
    steeply. Hits on faces within GRAZE_DEG of the ray's direction are left
    out: such a ray starts beside a convex edge and runs along the wall. Where
    the next face curves away with radius R, the ray leaves it after
    sqrt(2 R d) from a start d from the edge, at asin(length / R) to it, so a
    reading under 1.2 mm on a radius of 3 mm or more meets it at under 24
    degrees. The edge check covers those edges. A wedge sharper than
    90 - GRAZE_DEG degrees is still a thin wall here. A ray that leaves only
    through such faces is not measured; one that meets no face at all means a
    leaking mesh or a flipped normal and is a finding. Samples on triangles
    under SLIVER_MM2 are skipped: their normals are unreliable (e.g. at a
    surface pole).
    """
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
    v, f = m['v'], m['f']
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(v, deep=True))
    cells = vtk.vtkCellArray()
    cells.SetCells(len(f), numpy_to_vtkIdTypeArray(np.hstack([np.full((len(f), 1), 3), f]).astype(np.int64).ravel(), deep=True))
    poly = vtk.vtkPolyData()
    poly.SetPoints(points)
    poly.SetPolys(cells)
    tree = vtk.vtkOBBTree()
    tree.SetDataSet(poly)
    tree.BuildLocator()
    rng = np.random.default_rng(seed)
    counts = np.where(m['area'] < SLIVER_MM2, 0, np.maximum(1, np.round(m['area'] / spacing ** 2).astype(int)))
    owner = np.repeat(np.arange(len(f)), counts)
    weight = np.repeat(m['area'] / np.maximum(counts, 1), counts)
    r1, r2 = rng.random(len(owner)), rng.random(len(owner))
    flip = r1 + r2 > 1
    r1[flip], r2[flip] = 1 - r1[flip], 1 - r2[flip]
    single = counts[owner] == 1
    r1[single] = r2[single] = 1 / 3
    a, b, c = v[f[owner, 0]], v[f[owner, 1]], v[f[owner, 2]]
    sample = a + r1[:, None] * (b - a) + r2[:, None] * (c - a)
    normal = m['normal'][owner]
    hits, ids = vtk.vtkPoints(), vtk.vtkIdList()
    reach = float(np.linalg.norm(v.max(0) - v.min(0))) * 1.01
    start = 1e-3
    graze = math.sin(math.radians(GRAZE_DEG))
    out = np.full(len(sample), np.inf)
    grazed = np.zeros(len(sample), bool)
    for k in range(len(sample)):
        p0 = sample[k] - normal[k] * start
        tree.IntersectWithLine(p0.tolist(), (sample[k] - normal[k] * reach).tolist(), hits, ids)
        best = math.inf
        for j in range(hits.GetNumberOfPoints()):
            if ids.GetId(j) == owner[k]:
                continue
            if abs(m['normal'][ids.GetId(j)] @ normal[k]) < graze:
                grazed[k] = True
                continue
            q = hits.GetPoint(j)
            d = math.dist(q, p0)
            if 1e-6 < d < best:
                best = d
        out[k] = best + start
    thin = out < limit
    findings = []
    labels = _clusters(sample[thin], 2 * spacing)
    for label in range(labels.max() + 1 if len(labels) else 0):
        pick = np.flatnonzero(thin)[labels == label]
        low = pick[np.argmin(out[pick])]
        pts = sample[pick]
        findings.append(dict(kind='thin wall', min_mm=float(out[low]), area_mm2=float(weight[pick].sum()),
                             samples=int(len(pick)), at=np.round(sample[low], 3).tolist(),
                             box=[round(x, 4) for x in [*pts.min(0), *pts.max(0)]]))
    lost = np.flatnonzero(~np.isfinite(out) & ~grazed)
    for label in range(_clusters(sample[lost], 2 * spacing).max() + 1 if len(lost) else 0):
        pick = lost[_clusters(sample[lost], 2 * spacing) == label]
        pts = sample[pick]
        findings.append(dict(kind='ray without exit', samples=int(len(pick)), at=np.round(pts[0], 3).tolist(),
                             box=[round(x, 4) for x in [*pts.min(0), *pts.max(0)]]))
    finite = out[np.isfinite(out)]
    summary = dict(samples=int(len(out)), spacing_mm=spacing, reach_mm=reach,
                   min_mm=float(finite.min()) if len(finite) else None,
                   thin_area_mm2=float(weight[thin].sum()), rays_without_exit=int(len(lost)),
                   rays_only_grazing=int((~np.isfinite(out) & grazed).sum()))
    return findings, summary


# A mesh fold may cross a section outline over at most this area (deflection is 0.02 mm).
FOLD_AREA_MM2 = 1e-3


def _slice(m, z):
    """Section of a closed mesh at height z, as one shapely region (even-odd over its closed outlines)."""
    from shapely import LineString, MultiLineString, Polygon, make_valid
    from shapely.ops import linemerge
    v, f = m['v'], m['f']
    h = v[f][:, :, 2] - z
    above = h > 0
    crossing = above.any(axis=1) & ~above.all(axis=1)
    segments = []
    for tri, d in zip(v[f[crossing]], h[crossing]):
        points = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            if (d[i] > 0) != (d[j] > 0):
                t = d[i] / (d[i] - d[j])
                points.append(tuple(np.round(tri[i, :2] + t * (tri[j, :2] - tri[i, :2]), 7)))
        if points[0] != points[1]:
            segments.append(points)
    region = Polygon()
    for line in getattr(linemerge(MultiLineString(segments)), 'geoms', None) or [linemerge(MultiLineString(segments))]:
        assert line.is_closed, f'open section outline at z {z:.3f}: the mesh is not closed'
        loop = Polygon(line)
        if not line.is_simple:
            # A solid's section never crosses itself; a crossing is a fold in the
            # triangulation. Accept it only when the folded lobes are mesh-scale.
            pieces = sorted((g for g in getattr(make_valid(loop), 'geoms', [make_valid(loop)])
                             if g.geom_type in ('Polygon', 'MultiPolygon')), key=lambda g: -g.area)
            folded = sum(g.area for g in pieces[1:])
            assert pieces and folded <= FOLD_AREA_MM2, \
                f'self-crossing section outline at z {z:.3f}: {folded:.6f} mm2 beyond the main loop'
            loop = pieces[0]
        region = region.symmetric_difference(loop.buffer(0))
    return region


def symmetry(m, step=0.25, tolerance=0.05):
    """Mirror symmetry about the X = 0 plane, on horizontal slices of the mesh.

    Each slice (every step mm, offset from the mesh's flat faces) is compared with
    its mirror image by an exact 2D symmetric difference. Bands up to tolerance
    wide each side are mesh error (deflection 0.02 mm) and are ignored; any
    differing region that survives that inset is a finding, with its full area.
    Findings in touching boxes are merged; their volume is area times step.
    """
    from shapely import affinity
    lo, hi = m['v'][:, 2].min(), m['v'][:, 2].max()
    found = []
    for z in np.arange(lo + step / 2 + 0.0123, hi, step):
        region = _slice(m, z)
        diff = region.symmetric_difference(affinity.scale(region, -1.0, 1.0, origin=(0, 0)))
        for piece in getattr(diff, 'geoms', [diff]):
            if piece.is_empty or piece.buffer(-tolerance).is_empty:
                continue
            x0, y0, x1, y1 = piece.bounds
            found.append(dict(volume=piece.area * step, box=[float(b) for b in (x0, y0, z - step / 2, x1, y1, z + step / 2)]))
    merged = [dict(item) for item in found]
    joined = True
    while joined:
        joined = False
        for a in range(len(merged)):
            for b in range(len(merged) - 1, a, -1):
                if _boxes_touch(_grow(merged[a]['box'], 0.5), merged[b]['box']):
                    merged[a]['box'] = _union([merged[a]['box'], merged[b]['box']])
                    merged[a]['volume'] += merged.pop(b)['volume']
                    joined = True
    return [dict(kind='asymmetry', volume_mm3=g['volume'], box=[round(b, 4) for b in g['box']]) for g in merged]


# Edges: every outside edge is rounded with at least the minimum radius

def _outward_normal(face, uv):
    from OCP.BRepGProp import BRepGProp_Face
    from OCP.gp import gp_Pnt, gp_Vec
    point, normal = gp_Pnt(), gp_Vec()
    BRepGProp_Face(face).Normal(uv.X(), uv.Y(), point, normal)
    return np.array(normal.Coord()) / max(normal.Magnitude(), 1e-300)


def _convex(solid, edge, f1, f2, step=0.02):
    """True when the material angle at the edge midpoint is under 180 degrees (an outside edge).

    The point step mm along (n1 - n2) from the edge lies outside the part at an
    outside edge and inside the material at an inside corner.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.TopAbs import TopAbs_IN
    from OCP.gp import gp_Pnt
    first, last = BRep_Tool.Range_s(edge)
    mid = (first + last) / 2
    p = np.array(BRepAdaptor_Curve(edge).Value(mid).Coord())
    normals = []
    for face in (f1, f2):
        curve = BRep_Tool.CurveOnSurface_s(edge, face, 0.0, 0.0)
        normals.append(_outward_normal(face, curve.Value(mid)))
    d = normals[0] - normals[1]
    if np.linalg.norm(d) < 1e-9:
        return True
    probe = p + step * d / np.linalg.norm(d)
    classifier = BRepClass3d_SolidClassifier(solid, gp_Pnt(*probe), 1e-7)
    return classifier.State() != TopAbs_IN


def _crease_deg(edge, f1, f2, count=9):
    """Largest angle between the two faces' normals at points along the edge (its ends excluded)."""
    from OCP.BRep import BRep_Tool
    first, last = BRep_Tool.Range_s(edge)
    curves = [BRep_Tool.CurveOnSurface_s(edge, f, 0.0, 0.0) for f in (f1, f2)]
    worst = 0.0
    for t in np.linspace(first, last, count + 2)[1:-1]:
        n1, n2 = (_outward_normal(f, c.Value(t)) for f, c in zip((f1, f2), curves))
        worst = max(worst, math.degrees(math.acos(max(-1.0, min(1.0, float(n1 @ n2))))))
    return worst


def edges(shape, min_radius, smooth_deg=0.5):
    """Sharp outside edges, and faces whose smallest outside curvature radius at their mesh nodes is under min_radius.

    An edge is sharp when OCCT marks it C0 and the face normals measured along it
    differ by more than smooth_deg (a STEP round trip can drop the tangency mark
    of a fillet's boundary). Inside corners and concave curvature print cleanly
    in FDM and are counted only.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepLib import BRepLib
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepLProp import BRepLProp_SLProps
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GeomAbs import GeomAbs_C0, GeomAbs_Plane
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    s = getattr(shape, 'wrapped', shape)
    BRepMesh_IncrementalMesh(s, 0.02, False, 0.2, True)
    BRepLib.EncodeRegularity_s(s, math.radians(smooth_deg))
    findings, inside = [], dict(sharp_inside_edges=0, sharp_inside_length_mm=0.0, smallest_inside_radius_mm=math.inf)
    for edge, faces in _map(s, TopAbs_EDGE, TopAbs_FACE):
        edge = TopoDS.Edge_s(edge)
        if BRep_Tool.Degenerated_s(edge):
            continue
        distinct = []
        for face in faces:
            if not any(face.IsSame(other) for other in distinct):
                distinct.append(face)
        if len(distinct) == 1 and len(faces) == 1:
            findings.append(dict(kind='free edge', length_mm=_length(edge), box=box(edge)))
        elif len(distinct) == 2:
            f1, f2 = TopoDS.Face_s(distinct[0]), TopoDS.Face_s(distinct[1])
            if BRep_Tool.Continuity_s(edge, f1, f2) == GeomAbs_C0 and _crease_deg(edge, f1, f2) > smooth_deg:
                if _convex(s, edge, f1, f2):
                    findings.append(dict(kind='sharp edge', length_mm=_length(edge), box=box(edge)))
                else:
                    inside['sharp_inside_edges'] += 1
                    inside['sharp_inside_length_mm'] += _length(edge)
    for face in _map(s, TopAbs_FACE):
        face = TopoDS.Face_s(face)
        surface = BRepAdaptor_Surface(face)
        if surface.GetType() == GeomAbs_Plane:
            continue
        # Curvature is signed toward the surface normal; outward it is negative where the part is convex.
        sign = -1.0 if face.Orientation() == TopAbs_REVERSED else 1.0
        t = BRep_Tool.Triangulation_s(face, TopLoc_Location())
        assert t is not None and t.HasUVNodes(), 'face without UV mesh nodes'
        smallest, at = math.inf, None
        for i in range(1, t.NbNodes() + 1):
            uv = t.UVNode(i)
            props = BRepLProp_SLProps(surface, uv.X(), uv.Y(), 2, 1e-7)
            if props.IsCurvatureDefined():
                for k in (sign * props.MaxCurvature(), sign * props.MinCurvature()):
                    if k < -1e-9 and -1 / k < smallest:
                        smallest, at = -1 / k, props.Value().Coord()
                    elif k > 1e-9:
                        inside['smallest_inside_radius_mm'] = min(inside['smallest_inside_radius_mm'], 1 / k)
        if smallest < min_radius - 1e-3:
            # The finding's box is the face's region within 1 mm of its tightest point.
            near = [round(c, 4) for c in at]
            findings.append(dict(kind='small radius', radius_mm=smallest, area_mm2=_area(face), at=near,
                                 face_box=box(face), box=[c - 1.0 for c in near] + [c + 1.0 for c in near]))
    return findings, inside


# Printing: support area per build direction and closed voids

ORIENTATIONS = {'+Z up': (0, 0, 1), '-Z up': (0, 0, -1), '+X up': (1, 0, 0), '-X up': (-1, 0, 0),
                '+Y up': (0, 1, 0), '-Y up': (0, -1, 0)}


def support(m, up, angle_deg=45.0, bed=0.05):
    """Area facing down more than angle_deg past vertical, away from the bed, for build direction up."""
    d = np.array(up, float) / np.linalg.norm(up)
    height = m['v'] @ d
    facing = m['normal'] @ d
    on_bed = height[m['f']].max(axis=1) <= height.min() + bed
    need = (facing < -math.sin(math.radians(angle_deg))) & ~on_bed
    return dict(surface_mm2=float(m['area'][need].sum()), footprint_mm2=float((m['area'][need] * -facing[need]).sum()),
                bed_contact_mm2=float(m['area'][on_bed & (facing < -0.999)].sum()))


def voids(shape):
    """Closed inner shells: spaces with no opening that trap powder or resin."""
    from OCP.BRepClass3d import BRepClass3d
    from OCP.TopAbs import TopAbs_SHELL, TopAbs_SOLID
    from OCP.TopoDS import TopoDS
    import cadquery as cq
    findings = []
    for solid in _map(getattr(shape, 'wrapped', shape), TopAbs_SOLID):
        solid = TopoDS.Solid_s(solid)
        outer = BRepClass3d.OuterShell_s(solid)
        for shell in _map(solid, TopAbs_SHELL):
            if not shell.IsSame(outer):
                cavity = cq.Solid.makeSolid(cq.Shell(TopoDS.Shell_s(shell)))
                findings.append(dict(kind='closed void', volume_mm3=abs(cavity.Volume()), box=box(shell)))
    return findings


# Holes and fasteners

def holes(shape, max_diameter=12.0):
    """Round holes: inward-facing cylinder faces on one axis covering at least half a turn."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepGProp import BRepGProp_Face
    from OCP.BRepTools import BRepTools
    from OCP.GeomAbs import GeomAbs_Cylinder
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    from OCP.gp import gp_Pnt, gp_Vec
    groups = []
    for face in _map(getattr(shape, 'wrapped', shape), TopAbs_FACE):
        face = TopoDS.Face_s(face)
        surface = BRepAdaptor_Surface(face)
        if surface.GetType() != GeomAbs_Cylinder:
            continue
        cylinder = surface.Cylinder()
        radius = cylinder.Radius()
        if 2 * radius > max_diameter:
            continue
        u0, u1, v0, v1 = BRepTools.UVBounds_s(face)
        point, normal = gp_Pnt(), gp_Vec()
        BRepGProp_Face(face).Normal((u0 + u1) / 2, (v0 + v1) / 2, point, normal)
        axis = cylinder.Axis()
        o = np.array(axis.Location().Coord())
        d = np.array(axis.Direction().Coord())
        p = np.array(point.Coord())
        radial = (p - o) - ((p - o) @ d) * d
        if np.array(normal.Coord()) @ radial >= 0:
            continue
        ends = [np.array(surface.Value((u0 + u1) / 2, v).Coord()) for v in (v0, v1)]
        for g in groups:
            offset = (o - g['o']) - ((o - g['o']) @ g['d']) * g['d']
            if abs(radius - g['r']) < 1e-4 and abs(abs(d @ g['d']) - 1) < 1e-8 and np.linalg.norm(offset) < 1e-4:
                g['angle'] += u1 - u0
                g['ends'] += ends
                g['boxes'].append(box(face))
                break
        else:
            groups.append(dict(o=o, d=d, r=radius, angle=u1 - u0, ends=ends, boxes=[box(face)]))
    out = []
    for g in groups:
        if g['angle'] < math.pi - 1e-6:
            continue
        t = [(p - g['o']) @ g['d'] for p in g['ends']]
        a, b = g['o'] + g['d'] * min(t), g['o'] + g['d'] * max(t)
        out.append(dict(kind='hole', diameter_mm=round(2 * g['r'], 4), axis=np.round(g['d'], 6).tolist(),
                        ends=[np.round(a, 4).tolist(), np.round(b, 4).tolist()],
                        turn_deg=round(math.degrees(g['angle']), 1), box=_union(g['boxes'])))
    return out


def _on_axis(hole, origin, direction, offset=0.05, angle_deg=0.5):
    """Hole axis distance (mm) and angle (degrees) to a fastener axis, if within limits."""
    d = np.array(direction, float) / np.linalg.norm(direction)
    a, b = np.array(hole['ends'][0]), np.array(hole['ends'][1])
    h = (b - a) / np.linalg.norm(b - a)
    angle = math.degrees(math.acos(min(1.0, abs(h @ d))))
    mid = (a + b) / 2 - np.array(origin, float)
    distance = float(np.linalg.norm(mid - (mid @ d) * d))
    return (distance, angle) if distance <= offset and angle <= angle_deg else None


def _axial(points, origin, direction):
    d = np.array(direction, float) / np.linalg.norm(direction)
    t = sorted((np.array(p) - np.array(origin, float)) @ d for p in points)
    return t[0], t[-1]


def fastener(joint, parts):
    """One screw (and insert) from bd_warehouse against the declared parts.

    joint: name, screw (size, length, fastener_type), insert (size, fastener_type)
    or None, seat (point under the head on the axis), direction (into the joint),
    clamped (parts the screw passes through; the first carries the head),
    threaded (part holding the insert), insert_top (insert end on the axis) and
    driver (diameter and length of the tool path above the head).
    """
    import cadquery as cq
    import bd_warehouse.fastener as bw
    from build123d import Location, Plane
    seat, direction = np.array(joint['seat'], float), np.array(joint['direction'], float)
    direction /= np.linalg.norm(direction)
    spec = joint['screw']
    screw_part = bw.SocketHeadCapScrew(spec['size'], spec['length'], spec['fastener_type'], simple=True)
    d = screw_part.thread_diameter
    # The simple library screw models the thread core; add the major-diameter thread envelope.
    screw = _cq(screw_part.moved(Location(Plane(origin=tuple(seat), z_dir=tuple(-direction))))).fuse(
        _ring(seat, direction, 0, d / 2, 0.0, spec['length'])).clean()
    rows = dict(name=joint['name'], screw=screw_part.label, problems=[])
    problems = rows['problems']

    # The screw (head and shank) overlaps no part; the head sits on the first clamped part.
    for name, shape in parts.items():
        overlap = _volume(screw.intersect(shape)) if _boxes_touch(box(screw), box(shape)) else 0.0
        if overlap > TOLERANCE:
            problems.append(dict(kind='screw overlaps part', part=name, overlap_mm3=overlap, box=box(screw)))
    head_r = screw_part.head_diameter / 2
    clearance = screw_part.clearance_hole_diameters
    seat_ring = _ring(seat, direction, max(clearance.values()) / 2, head_r, 0.0, 0.1)
    fill = _volume(seat_ring.intersect(parts[joint['clamped'][0]])) / _volume(seat_ring)
    rows['head_seat_fill'] = fill
    if fill < 0.99:
        problems.append(dict(kind='head not seated', fill=fill, box=box(seat_ring)))

    # Each clamped part has a coaxial clearance hole of a library size over the screw.
    rows['clearance_holes'] = []
    for name in joint['clamped']:
        found = [(h, _on_axis(h, seat, direction)) for h in holes(parts[name])]
        found = [(h, fit) for h, fit in found if fit and _spans(h, seat, direction, 0.0, spec['length'])]
        if not found:
            problems.append(dict(kind='no coaxial clearance hole', part=name, box=box(screw)))
            continue
        for h, (offset, angle) in found:
            fits = [k for k, v in clearance.items() if abs(v - h['diameter_mm']) <= 0.01]
            row = dict(part=name, diameter_mm=h['diameter_mm'], fit=fits[0] if fits else None, axis_offset_mm=offset,
                       axis_angle_deg=angle, radial_clearance_mm=(h['diameter_mm'] - d) / 2)
            rows['clearance_holes'].append(row)
            if not fits or row['radial_clearance_mm'] < 0.1:
                problems.append(dict(kind='clearance hole not a library size', part=name, diameter_mm=h['diameter_mm'], box=h['box']))

    # Insert: a coaxial pilot of the library diameter, deep enough, with the library wall round it.
    insert = joint.get('insert')
    if insert:
        nut = bw.HeatSetNut(insert['size'], insert['fastener_type'])
        length, pilot_d, wall = nut.nut_data['m'], nut.nut_data['dc'], nut.nut_data['material_thickness']
        top = np.array(joint['insert_top'], float)
        t_top = (top - seat) @ direction
        rows['insert'] = dict(label=nut.label, length_mm=length, pilot_diameter_mm=pilot_d, wall_mm=wall)
        threaded = parts[joint['threaded']]
        pilots = [(h, _on_axis(h, seat, direction)) for h in holes(threaded)]
        pilots = [(h, fit) for h, fit in pilots if fit and abs(h['diameter_mm'] - pilot_d) <= 0.01]
        depth = max((_axial(h['ends'], top, direction)[1] for h, _ in pilots), default=0.0)
        rows['insert'].update(pilot_found=bool(pilots), pilot_depth_mm=depth)
        if not pilots:
            problems.append(dict(kind='no coaxial insert pilot of the library diameter', part=joint['threaded'],
                                 box=box(_ring(top, direction, 0, pilot_d / 2, 0, length))))
        elif depth < length:
            problems.append(dict(kind='insert pilot too shallow', depth_mm=depth, box=box(_ring(top, direction, 0, pilot_d / 2, 0, length))))
        ring = _ring(top, direction, pilot_d / 2, pilot_d / 2 + wall, 0.0, length)
        wall_fill = _volume(ring.intersect(threaded)) / _volume(ring)
        rows['insert']['wall_fill'] = wall_fill
        if wall_fill < 0.99:
            problems.append(dict(kind='insert wall thinner than library minimum', fill=wall_fill, box=box(ring)))
        # Thread engagement: the screw thread inside the insert.
        engaged = max(0.0, min(spec['length'], t_top + length) - max(0.0, t_top))
        required = min(length, 1.5 * d)
        rows['engagement'] = dict(engaged_mm=engaged, required_mm=required, insert_span_mm=[t_top, t_top + length])
        if engaged < required - 1e-6:
            problems.append(dict(kind='thread engagement too short', engaged_mm=engaged, required_mm=required,
                                 box=box(_ring(top, direction, 0, pilot_d / 2, 0, length))))
        # The tip stays clear of the threaded part when driven 0.3 mm further.
        deeper = screw.translate(cq.Vector(*(direction * 0.3)))
        bottom = _volume(deeper.intersect(threaded))
        rows['tip_overlap_when_0_3_deeper_mm3'] = bottom
        if bottom > TOLERANCE:
            problems.append(dict(kind='screw tip bottoms out', overlap_mm3=bottom, box=box(deeper)))

    # Driver path above the head.
    driver = joint.get('driver')
    if driver:
        head_top = seat - direction * screw_part.head_height
        path = _ring(head_top, direction, 0, driver['diameter'] / 2, -driver['length'], -0.01)
        rows['driver_blocked_by'] = {}
        for name, shape in parts.items():
            if _boxes_touch(box(path), box(shape)):
                overlap = _volume(path.intersect(shape))
                if overlap > TOLERANCE:
                    rows['driver_blocked_by'][name] = overlap
                    problems.append(dict(kind='driver path blocked', part=name, overlap_mm3=overlap, box=box(path)))
    rows['passed'] = not problems
    return rows


def _boxes_touch(a, b):
    return all(a[i] <= b[i + 3] + 1e-7 and b[i] <= a[i + 3] + 1e-7 for i in range(3))


def _spans(hole, origin, direction, t0, t1):
    a, b = _axial(hole['ends'], origin, direction)
    return a < t1 and b > t0


def _ring(origin, direction, r0, r1, t0, t1):
    """Ring between radii r0 and r1 from t0 to t1 along direction from origin (r0 = 0 gives a cylinder)."""
    import cadquery as cq
    d = np.array(direction, float)
    base = cq.Vector(*(np.array(origin, float) + d * t0))
    outer = cq.Solid.makeCylinder(r1, t1 - t0, base, cq.Vector(*d))
    return outer.cut(cq.Solid.makeCylinder(r0, t1 - t0 + 2, base - cq.Vector(*d), cq.Vector(*d))) if r0 > 0 else outer


def hole_owners(found, joints):
    """Each hole of the part matched to a coaxial fastener whose screw or insert reaches it."""
    for h in found:
        h['fastener'] = None
        for j in joints:
            reach = j['screw']['length']
            if _on_axis(h, j['seat'], j['direction']) and _spans(h, j['seat'], j['direction'], 0.0, reach + 5.0):
                h['fastener'] = j['name']
                break
    return [dict(h, kind='hole without fastener') for h in found if not h['fastener']]


# Accepted problems

def judge(check, findings, accepted, max_side=AREA_MAX_SIDE):
    """Findings of one check against its named accepted areas.

    A finding is accepted when its box lies inside an area of the same check.
    Every area must cover a finding, name a reason, stay within AREA_MARGIN of
    the findings it covers and keep each side under max_side (AREA_MAX_SIDE
    unless a part's configuration states a longer limit for long straight edges).
    """
    areas = [a for a in accepted if a['check'] == check]
    for f in findings:
        f['accepted_as'] = next((a['name'] for a in areas if _inside(f['box'], a['box'])), None)
    rows = []
    for a in areas:
        covered = [f for f in findings if f['accepted_as'] == a['name']]
        tight = bool(covered) and _inside(a['box'], _grow(_union([f['box'] for f in covered]), AREA_MARGIN))
        small = max(a['box'][i + 3] - a['box'][i] for i in range(3)) <= max_side
        rows.append(dict(name=a['name'], reason=a.get('reason', ''), box=a['box'], covers=len(covered), tight=tight,
                         small=small, passed=bool(covered and tight and small and a['name'] and a.get('reason'))))
    open_findings = [f for f in findings if not f['accepted_as']]
    return dict(findings=findings, open=len(open_findings), areas=rows,
                passed=not open_findings and all(r['passed'] for r in rows))


# Known-error fixtures: each checker must find its error before it judges a part

def _fixtures():
    import build123d as bd
    base = bd.Box(30, 20, 10, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    fin = bd.Pos(8, 0, 10) * bd.Box(0.8, 20, 8, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    bad = base + fin
    corner = bad.edges().filter_by(bd.Axis.Y).sort_by(bd.Axis.X)[0]
    bad = bd.chamfer(corner, 0.03)
    plain = bd.Box(20, 20, 10, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    sharp = bd.fillet(plain.edges().filter_by(bd.Axis.Z), 1.0)
    small = bd.fillet(plain.edges(), 0.3)
    round_ = bd.fillet(plain.edges(), 1.0)
    # Inside corners only: two bosses rounded on top, one with a sharp foot and one with a 0.3 mm inside fillet.
    for x in (-5, 5):
        round_ += bd.Pos(x, 0, 10) * bd.Cylinder(3, 4, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    round_ = bd.fillet(round_.edges().filter_by(bd.GeomType.CIRCLE).filter_by_position(bd.Axis.Z, 13.9, 14.1), 1.0)
    foot = round_.edges().filter_by(bd.GeomType.CIRCLE).filter_by_position(bd.Axis.Z, 9.9, 10.1)
    round_ = bd.fillet(foot.sort_by(bd.Axis.X)[-1], 0.3)
    shelf = plain + bd.Pos(15, 0, 2) * bd.Box(10, 20, 3, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    cube = bd.Box(20, 20, 20, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    hollow = cube - bd.Pos(0, 0, 10) * bd.Sphere(3)
    # A 60 degree wedge: a ray from one face crosses the other at 30 degrees, so the graze rule must keep it.
    wedge = bd.extrude(bd.Polygon((0, 0), (20, -20 * math.tan(math.radians(30))), (20, 20 * math.tan(math.radians(30))),
                                  align=None), 10)
    # A half pipe (radii 1.5 and 4): its square cut faces meet the outer cylinder, which curves away from them.
    pipe = bd.Cylinder(4, 10) - bd.Cylinder(1.5, 12)
    pipe = pipe - bd.Box(10, 10, 12, align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.CENTER))
    return dict(geometry_bad=bad, geometry_good=plain, geometry_shifted=bd.Pos(0.1, 0, 0) * plain, geometry_wedge=wedge, geometry_pipe=pipe, edges_sharp=sharp,
                edges_small=small, edges_good=round_,
                print_bad=shelf, print_void=hollow, print_good=plain)


def _joint_fixture(hole_d=3.4, pilot_offset=0.0, length=10, second_hole=False):
    import build123d as bd
    plate = bd.Pos(0, 0, 10) * bd.Box(20, 20, 4, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    plate -= bd.Pos(0, 0, 9) * bd.Cylinder(hole_d / 2, 6, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    block = bd.Box(20, 20, 10, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    block -= bd.Pos(pilot_offset, 0, 4) * bd.Cylinder(2.0, 7, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    block -= bd.Pos(pilot_offset, 0, 0.9) * bd.Cylinder(1.7, 3.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    if second_hole:
        block -= bd.Pos(6, 6, -1) * bd.Cylinder(1.7, 12, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    joint = dict(name='fixture screw', screw=dict(size='M3-0.5', length=length, fastener_type='iso4762'),
                 insert=dict(size='M3-0.5-5.7', fastener_type='CNCKitchen'), seat=(0, 0, 14), direction=(0, 0, -1),
                 clamped=['plate'], threaded='block', insert_top=(0, 0, 10), driver=dict(diameter=6.0, length=30.0))
    return joint, dict(plate=_cq(plate), block=_cq(block))


def self_test():
    """Run every checker on fixtures with a known error and on a clean fixture."""
    fx = {k: _cq(v) for k, v in _fixtures().items()}
    rows = []

    def expect(checker, fixture, error, found, clean=None):
        rows.append(dict(checker=checker, fixture=fixture, known_error=error, found=bool(found),
                         clean_fixture_quiet=clean, passed=bool(found) and clean is not False))

    lim = dict(short_edge=0.05, small_face=0.05, narrow_face=0.1)
    bad = topology(fx['geometry_bad'], **lim)
    good = topology(fx['geometry_good'], **lim)
    expect('geometry.topology', 'block with a 0.03 mm chamfer', 'short edges and a narrow face',
           any(f['kind'] == 'short edge' for f in bad) and any(f['kind'] == 'narrow face' for f in bad), not good)
    thin, _ = thickness(mesh(fx['geometry_bad']), 1.2)
    quiet, _ = thickness(mesh(fx['geometry_good']), 1.2)
    expect('geometry.thickness', 'block with a 0.8 mm fin', 'thin wall near 0.8 mm',
           any(abs(f['min_mm'] - 0.8) < 0.05 and f['box'][0] > 7 for f in thin), not quiet)
    knife, _ = thickness(mesh(fx['geometry_wedge']), 1.2)
    pipe, _ = thickness(mesh(fx['geometry_pipe']), 1.2)
    expect('geometry.thickness', '60 degree wedge; half pipe 2.5 mm thick', 'thin knife edge; pipe quiet beside its cuts',
           any(f['min_mm'] < 0.5 and f['box'][0] < 1 for f in knife), not pipe)
    asym = symmetry(mesh(fx['geometry_bad']))
    expect('geometry.symmetry', 'fin on one side of the YZ plane', 'about 256 mm3 differing',
           abs(sum(f['volume_mm3'] for f in asym) - 2 * 0.8 * 20 * 8) < 1.0, not symmetry(mesh(fx['geometry_good'])))
    expect('geometry.symmetry', 'symmetric block moved 0.1 mm along X', 'both X faces differ',
           len(symmetry(mesh(fx['geometry_shifted']))) == 2)
    # A prism whose section is an uneven bow tie: its outline crosses itself at
    # (1.2, 1.2), away from every mesh node, leaving 1.8 and 0.8 mm2 lobes.
    tie = np.array([(0, 0), (2, 2), (2, 0), (0, 3)], float)
    tv = np.array([(x, y, z) for z in (0.0, 1.0) for x, y in tie])
    tf = np.array([t for i in range(4) for t in ((i, (i + 1) % 4, 4 + i), ((i + 1) % 4, 4 + (i + 1) % 4, 4 + i))])
    try:
        _slice(dict(v=tv, f=tf), 0.5)
        fold = False
    except AssertionError as error:
        fold = 'self-crossing' in str(error)
    expect('geometry.sections', 'uneven bow-tie prism, 1.8 and 0.8 mm2 lobes', 'self-crossing outline refused',
           fold, not _slice(mesh(fx['geometry_good']), 1.0).is_empty)
    sharp, _ = edges(fx['edges_sharp'], 0.5)
    good, corner = edges(fx['edges_good'], 0.5)
    expect('edges.sharp', 'block rounded on its vertical edges only', 'sixteen sharp rim edges',
           sum(f['kind'] == 'sharp edge' for f in sharp) == 16, not good)
    small, _ = edges(fx['edges_small'], 0.5)
    expect('edges.radius', 'block rounded 0.3 mm', 'radius under 0.5 mm',
           any(f['kind'] == 'small radius' and abs(f['radius_mm'] - 0.3) < 0.01 for f in small) and
           not any(f['kind'] == 'sharp edge' for f in small))
    expect('edges.inside', 'rounded block with two bosses: a sharp foot and a 0.3 mm inside fillet',
           'inside corners counted, not reported', corner['sharp_inside_edges'] >= 1 and
           abs(corner['smallest_inside_radius_mm'] - 0.3) < 0.01, not good)
    shelf = support(mesh(fx['print_bad']), (0, 0, 1))
    expect('print.support', '10 x 20 mm shelf 2 mm over the bed', '200 mm2 to support',
           abs(shelf['footprint_mm2'] - 200.0) < 1.0, support(mesh(fx['print_good']), (0, 0, 1))['footprint_mm2'] < 1e-6)
    void = voids(fx['print_void'])
    expect('print.voids', 'cube with a closed 3 mm sphere', 'one closed void of 113 mm3',
           len(void) == 1 and abs(void[0]['volume_mm3'] - 4 / 3 * math.pi * 27) < 0.5, not voids(fx['print_good']))

    good_joint, good_parts = _joint_fixture()
    clean = fastener(good_joint, good_parts)['passed']
    for case, kwargs, kind in (('plate hole 2.8 mm', dict(hole_d=2.8), 'screw overlaps part'),
                               ('pilot 0.3 mm off axis', dict(pilot_offset=0.3), 'no coaxial insert pilot of the library diameter'),
                               ('M3 x 6 screw', dict(length=6), 'thread engagement too short')):
        joint, parts = _joint_fixture(**kwargs)
        found = [p['kind'] for p in fastener(joint, parts)['problems']]
        expect('fasteners', case, kind, kind in found, clean)
    joint, parts = _joint_fixture(second_hole=True)
    lone = hole_owners(holes(parts['block']), [joint])
    expect('holes', 'block with a second 3.4 mm hole', 'one hole without fastener',
           len(lone) == 1 and abs(lone[0]['ends'][0][0] - 6) < 1e-3,
           not hole_owners(holes(good_parts['block']), [good_joint]))

    finding = [dict(kind='x', box=[0, 0, 0, 1, 1, 1])]
    big = judge('x', [dict(f) for f in finding], [dict(check='x', name='big', reason='r', box=[-30, -30, -30, 30, 30, 30])])
    stale = judge('x', [], [dict(check='x', name='stale', reason='r', box=[0, 0, 0, 1, 1, 1])])
    outside = judge('x', [dict(f) for f in finding], [dict(check='x', name='near', reason='r', box=[2, 2, 2, 3, 3, 3])])
    fine = judge('x', [dict(f) for f in finding], [dict(check='x', name='tight', reason='r', box=[-0.5, -0.5, -0.5, 1.5, 1.5, 1.5])])
    expect('accepted areas', 'oversized, stale and missed areas', 'each rejected',
           not big['passed'] and not stale['passed'] and not outside['passed'], fine['passed'])
    return rows


# The Zeee 2S holder

def _holder_joints(lm, sp):
    """The four arm screws: M3 x 10 down through the trunk plate into the holder inserts."""
    return [dict(name=f'holder screw x={x:+.0f}', screw=dict(lm.SCREW), insert=dict(sp.INSERT),
                 seat=(x, y, lm.SCREW_SEAT_Z), direction=(0.0, 0.0, -1.0), clamped=['trunk_base'], threaded='lipo_pod',
                 insert_top=(x, y, lm.ARM_TOP), driver=dict(diameter=3.0, length=50.0)) for x, y in lm.SCREWS]


def _sections(holder, ls):
    """Designed wall dimensions measured along rays: front panel, rear wall, floor and arms."""
    import cadquery as cq

    def depth(a, b):
        return sum(e.Length() for e in cq.Edge.makeLine(cq.Vector(*a), cq.Vector(*b)).intersect(holder).Edges())
    rows = []
    front = ls.COWL_REAR_Y
    # z 150 stays between the arms: their 3 mm root curves thicken the panel at X 12-29 mm.
    for z, xs in ((100, (-17, 0, 17)), (140, (-25, 0, 25)), (150, (-8, 0, 8)), (159, (-22, -11, 0, 11, 22)), (164, (-15, 0, 15))):
        for x in xs:
            rows.append(dict(feature='front panel', at_mm=[x, z], nominal_mm=ls.COWL_WALL,
                             measured_mm=depth((x, front - 3, z), (x, front + 0.1, z))))
    for x, y in ((0, ls.PACK_CENTER_Y), (-15, 40.5), (15, 56.5)):   # inside the pack cavity
        rows.append(dict(feature='floor', at_mm=[x, y], nominal_mm=ls.FLOOR_TOP - ls.BASE_BOTTOM,
                         measured_mm=depth((x, y, ls.BASE_BOTTOM - 0.5), (x, y, ls.FLOOR_TOP + 0.01))))
    for x, y in ((-27, 29), (-20.5, 33), (-14.5, 29), (14.5, 29), (20.5, 33), (27, 29)):
        rows.append(dict(feature='arm', at_mm=[x, y], nominal_mm=8.0, measured_mm=depth((x, y, 150.4), (x, y, 158.5))))
    for row in rows:
        row['passed'] = abs(row['measured_mm'] - row['nominal_mm']) < 0.01
    return rows


def lipo_pod():
    """The printed Zeee 2S holder against the quality checks, after the checkers prove themselves."""
    import cadquery as cq
    import sys
    sys.path[:0] = [str(ROOT / 'src')]
    from checks.common import read_scene
    from lib import lipo_shoulders as ls, lipo_mount as lm, stick_pod as sp
    started = time.monotonic()
    canaries = self_test()
    assert all(r['passed'] for r in canaries), [r for r in canaries if not r['passed']]
    config = json.loads((ROOT / 'source/lipo_pod_quality.json').read_text())
    limits, accepted = config['limits'], config['accepted']
    side = config.get('area_max_side_mm', AREA_MAX_SIDE)
    scene = read_scene(ROOT / 'STEP/frankenduck.step')
    shapes = {p.label: cq.Shape.cast(p.shape().wrapped) for p in scene.roots[0].children}
    holder = shapes['lipo_pod']
    assert holder.isValid() and len(holder.Solids()) == 1, 'holder must be one valid solid'
    m = mesh(holder)

    results = {}
    geometry = topology(holder, limits['short_edge_mm'], limits['small_face_mm2'], limits['narrow_face_mm'])
    thin, wall = thickness(m, limits['min_wall_mm'])
    results['geometry'] = judge('geometry', geometry, accepted, side)
    results['wall'] = judge('wall', thin, accepted, side)
    results['wall']['summary'] = wall
    s = config['symmetry']
    results['symmetry'] = judge('symmetry', symmetry(m, s['step_mm'], s['tolerance_mm']), accepted, side)
    edge_findings, inside = edges(holder, limits['min_edge_radius_mm'])
    results['edges'] = judge('edges', edge_findings, accepted, side)
    results['edges']['inside_corners'] = inside
    results['voids'] = judge('voids', voids(holder), accepted, side)
    supports = {name: support(m, up, limits['support_angle_deg']) for name, up in ORIENTATIONS.items()}
    joints = _holder_joints(lm, sp)
    # The holder is screwed to the trunk before the body shells go on, so the
    # shells are not in the driver path; the modeled screws and inserts are the
    # parts under test and are left out.
    parts = {name: shape for name, shape in shapes.items()
             if name not in ('left_shell_native', 'right_shell_native', 'lipo_mount_screws', 'lipo_mount_inserts')}
    fasteners = [fastener(j, parts) for j in joints]
    found_holes = holes(holder)
    results['holes'] = judge('holes', hole_owners(found_holes, joints), accepted, side)
    sections = _sections(holder, ls)
    unused = [a['name'] for a in accepted if a['check'] not in results]
    report = dict(document_sha256=scene.document_hash, units='mm', part='lipo_pod (lipo_pod_native)',
                  frame='world; the holder is placed with an identity transform', limits=limits, area_max_side_mm=side,
                  self_test=canaries, mesh=dict((k, m[k]) for k in ('mesh_volume_mm3', 'solid_volume_mm3')),
                  support_by_build_direction=supports, holes_found=found_holes, fasteners=fasteners,
                  sections=sections, unknown_accepted_checks=unused, **{k: v for k, v in results.items()},
                  seconds=time.monotonic() - started,
                  coverage='Zeee 2S holder only, as saved in the assembly. Checkers prove themselves on known-error fixtures '
                           'first. Short edges, small and narrow faces, ray-cast wall thickness, mirror symmetry on 0.25 mm '
                           'slices, sharp edges and curvature radius, closed voids, support area for six build directions '
                           '(reported), every hole owned by a fastener or a named reason, the four bd_warehouse M3 x 10 '
                           'screws with CNC Kitchen inserts (clearance, seat, alignment, pilot, wall, engagement, tip and '
                           'driver path with the shells off), and the designed front panel, floor and arm sections. '
                           'No slicing, strength or physical fit test.')
    report['passed'] = (all(r['passed'] for r in results.values()) and all(f['passed'] for f in fasteners)
                        and all(r['passed'] for r in sections) and not unused)
    (ROOT / 'reports/latest/lipo_pod_quality.json').write_text(json.dumps(report, indent=2, default=float) + '\n')
    print({k: (v['open'], len(v['areas'])) for k, v in results.items()}, [f['problems'] for f in fasteners],
          [r for r in sections if not r['passed']])
    assert report['passed'], 'holder quality findings; see reports/latest/lipo_pod_quality.json'
    return report
