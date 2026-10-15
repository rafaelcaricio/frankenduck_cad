"""Produce a tracked, atomic 3MF print bundle from classified saved geometry."""
from collections import Counter
import ast
import importlib
from concurrent.futures import ThreadPoolExecutor
import subprocess
from datetime import datetime, timezone
import csv
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile

from tools.components import validate

ROOT = Path(__file__).resolve().parents[1]
MESH_TOLERANCE = 0.01  # Absolute millimetres; resolves thin foot faces.
ANGULAR_TOLERANCE = 0.1
MAX_RELATIVE_VOLUME_ERROR = 0.02
MESH_ALGORITHM = "Watson"
MESH_SURFACE_CONTROL = False
MESH_INTERNAL_VERTICES = False
MESH_MIN_SIZE = 1e-5


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def solid_volume(shape):
    """Volume by adaptive integration, the reference for mesh volumes.

    The fixed-order default reads about 2 % high on dense B-spline faces (the
    stick pod), which is as large as the mesh tolerance it is compared with.
    """
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(getattr(shape, "wrapped", shape), props, 1e-6)
    return props.Mass()


def transform(point, text):
    if not text:
        return point
    m = list(map(float, text.split()))
    if len(m) != 12 or not all(math.isfinite(x) for x in m):
        raise ValueError("Invalid 3MF transform")
    x, y, z = point
    return (m[0]*x+m[3]*y+m[6]*z+m[9], m[1]*x+m[4]*y+m[7]*z+m[10], m[2]*x+m[5]*y+m[8]*z+m[11])


def inspect_3mf(path, allow_numerical_overlaps=False):
    """Validate the actual package, topology, winding, volume and placed bounds."""
    with zipfile.ZipFile(path) as archive:
        if archive.testzip() is not None:
            raise ValueError("Corrupt 3MF ZIP member")
        model = ET.fromstring(archive.read("3D/3dmodel.model"))
    ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
    if model.get("unit") != "millimeter":
        raise ValueError("Print 3MF must explicitly use millimeters")
    objects = {o.get("id"): o for o in model.findall("m:resources/m:object", ns)}
    meshes = []
    def visit(identifier, transforms, seen):
        if identifier in seen or identifier not in objects:
            raise ValueError("Cyclic or missing 3MF object")
        obj = objects[identifier]
        mesh = obj.find("m:mesh", ns)
        if mesh is not None:
            points = [tuple(float(v.get(axis)) for axis in ("x", "y", "z")) for v in mesh.findall("m:vertices/m:vertex", ns)]
            points = [apply_chain(v, transforms) for v in points]
            # Face-local duplicates at seams have the same exact coordinates.
            unique, indices = {}, []
            for point in points:
                if not all(math.isfinite(v) for v in point):
                    raise ValueError("Non-finite 3MF vertex")
                indices.append(unique.setdefault(point, len(unique)))
            vertices = list(unique)
            triangles = []
            for triangle in mesh.findall("m:triangles/m:triangle", ns):
                raw = [int(triangle.get(k)) for k in ("v1", "v2", "v3")]
                if any(i < 0 or i >= len(points) for i in raw):
                    raise ValueError("Triangle index outside vertex array")
                t = tuple(indices[i] for i in raw)
                if len(set(t)) != 3:
                    raise ValueError("Degenerate 3MF triangle")
                triangles.append(t)
            if not vertices or not triangles:
                raise ValueError("Empty 3MF mesh")
            edges, directions = Counter(), Counter()
            terms = []
            for a, b, c in triangles:
                for u, v in ((a,b), (b,c), (c,a)):
                    edge = tuple(sorted((u,v)))
                    edges[edge] += 1
                    directions[edge] += 1 if u < v else -1
                x, y, z = vertices[a], vertices[b], vertices[c]
                terms.append((x[0]*(y[1]*z[2]-y[2]*z[1])+x[1]*(y[2]*z[0]-y[0]*z[2])+x[2]*(y[0]*z[1]-y[1]*z[0]))/6)
            bad_edges = sum(n != 2 for n in edges.values())
            bad_winding = sum(n != 0 for n in directions.values())
            volume = math.fsum(terms)
            balanced_overlaps = bad_edges and all(n >= 2 and n % 2 == 0 for n in edges.values()) and not bad_winding
            if (bad_edges and not (allow_numerical_overlaps and balanced_overlaps)) or bad_winding or volume <= 0:
                raise ValueError(f"Invalid closed mesh: {bad_edges} edges, {bad_winding} winding errors, volume {volume}")
            meshes.append(dict(vertices=len(vertices), triangles=len(triangles), volume_mm3=volume,
                bounds_mm=[*[min(p[i] for p in vertices) for i in range(3)], *[max(p[i] for p in vertices) for i in range(3)]],
                nonmanifold_edges=bad_edges, inconsistent_winding_edges=bad_winding,
                open_edges=sum(n == 1 for n in edges.values()),
                warnings=["Balanced internal seam overlaps; review in slicer"] if balanced_overlaps else []))
        children = obj.findall("m:components/m:component", ns)
        if mesh is None and not children:
            raise ValueError("3MF object has no geometry")
        for component in children:
            visit(component.get("objectid"), [component.get("transform")] + transforms, seen | {identifier})
    def apply_chain(point, transforms):
        for value in transforms:
            point = transform(point, value)
        return point
    for item in model.findall("m:build/m:item", ns):
        visit(item.get("objectid"), [item.get("transform")], set())
    if len(meshes) != 1:
        raise ValueError(f"Expected one printed solid mesh, got {len(meshes)}")
    return meshes[0]


def source_shape(model, occurrence):
    """Rebuild a STEP-damaged solid using its current factory and exact placement."""
    assembly = importlib.import_module("frankenduck")
    tree = ast.parse((ROOT/"src/frankenduck.py").read_text())
    matches = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and
        isinstance(n.func, ast.Name) and n.func.id == "place" and
        isinstance(n.args[0], ast.Call) and isinstance(n.args[0].func, ast.Name) and
        n.args[0].func.id == model and ast.literal_eval(n.args[1]) == occurrence["name"]]
    if len(matches) != 1:
        raise ValueError(f"Cannot resolve unique source placement: {model}")
    pose_node = matches[0].args[2]
    pose = getattr(assembly, pose_node.id) if isinstance(pose_node, ast.Name) else ast.literal_eval(pose_node)
    return assembly.place(importlib.import_module("parts."+model).make(), occurrence["name"], pose)


def complete_planar_boundaries(vertices, triangles, shape):
    """Triangulate omitted planar regions only where native CAD has material."""
    import numpy as np
    from shapely import Polygon, constrained_delaunay_triangles
    from cadgen import build123d as bd
    counts = Counter(tuple(sorted((a,b))) for t in triangles for a,b in zip(t,t[1:]+t[:1]))
    border = [(b,a) for t in triangles for a,b in zip(t,t[1:]+t[:1]) if counts[tuple(sorted((a,b)))] == 1]
    if not border: return []
    successors = dict(border)
    if len(successors) != len(border) or set(successors) != set(successors.values()):
        raise ValueError("Planar mesh boundary is branched")
    remaining = set(successors)
    faces = list(shape.faces())
    receipts = []
    while remaining:
        ring = [min(remaining)]
        while successors[ring[-1]] != ring[0]:
            following = successors[ring[-1]]
            if following in ring: raise ValueError("Mesh boundary is not a simple loop")
            ring.append(following)
        remaining.difference_update(ring)
        accepted = False
        for face in faces:
            origin = np.array(tuple(face.center()))
            normal = np.array(tuple(face.normal_at()))
            points = [np.array(vertices[i])-origin for i in ring]
            if max(abs(np.dot(p,normal)) for p in points) > 1e-5: continue
            axis = np.array([1.,0.,0.]) if abs(normal[0]) < .9 else np.array([0.,1.,0.])
            u = np.cross(normal,axis); u /= np.linalg.norm(u)
            v = np.cross(normal,u)
            xy = [(float(np.dot(p,u)),float(np.dot(p,v))) for p in points]
            polygon = Polygon(xy)
            if not polygon.is_valid or polygon.area <= 1e-12: continue
            indices = dict(zip(xy,ring))
            signed_area = sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(xy,xy[1:]+xy[:1]))
            new_triangles = []
            for cell in constrained_delaunay_triangles(polygon).geoms:
                coords = list(cell.exterior.coords)[:3]
                triangle = [indices[tuple(p)] for p in coords]
                pts = [np.array(vertices[i]) for i in triangle]
                samples = [sum(pts)/3, *[(a+b)/2 for a,b in zip(pts,pts[1:]+pts[:1])]]
                if any(face.distance_to(tuple(point)) > .001 for point in samples):
                    new_triangles = []; break
                direction = sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(coords,coords[1:]+coords[:1]))
                if direction*signed_area < 0: triangle.reverse()
                new_triangles.append(triangle)
            if not new_triangles: continue
            triangles.extend(new_triangles)
            receipts.append(dict(boundary_vertices=len(ring), area_mm2=polygon.area,
                native_face_area_mm2=face.area, native_surface_tolerance_mm=.001))
            accepted = True; break
        if not accepted: raise ValueError("Open mesh boundary does not match a native planar surface")
    return receipts


def write_native_3mf(shape, path):
    """Triangulate native faces with OCCT; package using pinned build123d/Lib3MF."""
    from cadgen import build123d as bd
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IMeshTools import IMeshTools_Parameters
    import OCP.IMeshTools as mesh_tools
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopAbs import TopAbs_REVERSED
    parameters = IMeshTools_Parameters()
    parameters.Deflection = MESH_TOLERANCE
    parameters.Angle = ANGULAR_TOLERANCE
    parameters.Relative = False
    parameters.ControlSurfaceDeflection = MESH_SURFACE_CONTROL
    parameters.InternalVerticesMode = MESH_INTERNAL_VERTICES
    parameters.InParallel = False
    parameters.MinSize = MESH_MIN_SIZE
    parameters.MeshAlgo = getattr(mesh_tools, "IMeshTools_MeshAlgoType_"+MESH_ALGORITHM)
    BRepMesh_IncrementalMesh(shape.wrapped, parameters)
    faces = shape.faces()
    missing_indices = [i for i,f in enumerate(faces)
                       if BRep_Tool.Triangulation_s(f.wrapped, TopLoc_Location()) is None]
    actual_deflection = MESH_TOLERANCE
    if any(faces[i].geom_type != bd.GeomType.PLANE for i in missing_indices):
        raise ValueError(f"Unsupported unmeshable faces in {shape.label}")
    vertices, triangles, missing = [], [], []
    for face_index,face in enumerate(faces):
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face.wrapped, location)
        if mesh is None or face_index in missing_indices:
            missing.append(face)
            continue
        offset = len(vertices)
        for index in range(1, mesh.NbNodes()+1):
            point = mesh.Node(index).Transformed(location.Transformation())
            vertices.append((point.X(), point.Y(), point.Z()))
        order = (1,3,2) if face.wrapped.Orientation() == TopAbs_REVERSED else (1,2,3)
        triangles.extend([triangle.Value(k)+offset-1 for k in order] for triangle in mesh.Triangles())
    from scipy.spatial import cKDTree
    unique = {}
    remap = [unique.setdefault(tuple(round(x,6) for x in p), len(unique)) for p in vertices]
    vertices = list(unique)
    parents = list(range(len(vertices)))
    def find(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    for a,b in cKDTree(vertices).query_pairs(.0001):
        a,b = find(a),find(b)
        parents[max(a,b)] = min(a,b)
    remap = [find(i) for i in remap]
    triangles = [[remap[i] for i in t] for t in triangles if len({remap[i] for i in t}) == 3]
    planar_receipt = complete_planar_boundaries(vertices, triangles, shape)
    package = bd.Mesher(unit=bd.Unit.MM)
    mesh_object = package.model.AddMeshObject()
    mesh_object.SetGeometry(*bd.Mesher._create_3mf_mesh(vertices, triangles))
    mesh_object.SetType(bd.Mesher._map_b3d_mesh_type_3mf[bd.MeshType.MODEL])
    mesh_object.SetName(shape.label)
    library_valid = mesh_object.IsValid()
    library_manifold = mesh_object.IsManifoldAndOriented()
    package.model.AddBuildItem(mesh_object, package.wrapper.GetIdentityTransform())
    package.write(path)
    return dict(linear_deflection_mm=actual_deflection,
                algorithm=MESH_ALGORITHM, native_parallel=False, control_surface_deflection=MESH_SURFACE_CONTROL, internal_vertices=MESH_INTERNAL_VERTICES,
                min_size_mm=MESH_MIN_SIZE, angular_deflection_rad=ANGULAR_TOLERANCE,
                constrained_planar_faces=len(missing), planar_boundary_loops=planar_receipt,
                seam_weld_tolerance_mm=.0001, library_valid=library_valid, library_manifold=library_manifold)


def write_export_3mf(shape, path):
    """Keep topology cleanup confined to an export copy and verify its effect."""
    try:
        return write_native_3mf(shape, path)
    except ValueError as error:
        from cadgen import build123d as bd
        from OCP.ShapeFix import ShapeFix_FixSmallFace
        fixer = ShapeFix_FixSmallFace()
        fixer.Init(shape.wrapped)
        fixer.SetPrecision(.0001)
        fixer.SetMaxTolerance(.0001)
        fixer.Perform()
        cleaned = bd.Part(fixer.Shape()).fix()
        cleaned.label = shape.label
        before, after = shape.bounding_box(), cleaned.bounding_box()
        bounds_error = max(abs(a-b) for a,b in zip(
            (*before.min, *before.max), (*after.min, *after.max)))
        volume_error = abs(shape.volume-cleaned.volume)
        if (not cleaned.is_valid or len(cleaned.solids()) != 1 or
            bounds_error > 1e-5 or volume_error > max(.02,shape.volume*1e-6)):
            raise ValueError(f"Export topology cleanup failed geometry checks: {shape.label}") from error
        result = write_native_3mf(cleaned, path)
        result["topology_cleanup"] = dict(precision_mm=.0001, bounds_error_mm=bounds_error,
            volume_difference_mm3=volume_error, original_mesh_error=str(error))
        return result


def geometric_difference(a, b):
    """Resolve mass-integration disagreement with actual native solid differences."""
    values = []
    for first,second in ((a,b),(b,a)):
        difference = first.cut(second)
        if not difference.is_valid or not math.isfinite(difference.volume) or difference.volume < 0:
            raise ValueError("Invalid native geometry difference")
        values.append(difference.volume)
    return values


def convert_worker(record, stage_path, bundle_path):
    from cadgen import build123d as bd
    stage, bundle = Path(stage_path), Path(bundle_path)
    name = record["model"]
    print("Converting " + name, flush=True)
    native = source_shape(name, record["occurrences"][0])
    if not native.is_valid or len(native.solids()) != 1:
        raise ValueError(f"Invalid source factory: {name}")
    native = native.moved(bd.Location(tuple(record["translation_mm"])))
    native.label = name
    box = native.bounding_box()
    bounds = [box.min.X,box.min.Y,box.min.Z,box.max.X,box.max.Y,box.max.Z]
    bounds_error = max(abs(a-b) for a,b in zip(bounds,record["expected_bounds_mm"]))
    native_volume = solid_volume(native)
    volume_error = abs(native_volume-record["source_volume_mm3"])
    differences = None
    if volume_error > max(.02,record["source_volume_mm3"]*1e-6):
        saved = bd.import_brep(stage/(name+".brep"))
        differences = geometric_difference(native, saved)
    if bounds_error > 1e-5 or (differences is not None and sum(differences) > .02):
        raise ValueError(f"Source factory differs from saved assembly: {name}")
    record.update(geometry_source="current_source_factory",
        saved_step_comparison=dict(bounds_error_mm=bounds_error, volume_difference_mm3=volume_error,
            bidirectional_difference_mm3=differences,
            saved_step_valid=record["saved_step_comparison"].get("saved_step_valid", True)
                if record["saved_step_comparison"] else True),
        source_volume_mm3=native_volume)
    record["tessellation"] = write_export_3mf(native, bundle/record["filename"])
    measured = inspect_3mf(bundle/record["filename"], allow_numerical_overlaps=True)
    allowed = max(.01, 2*MESH_TOLERANCE+1e-5)
    bound_error = max(abs(a-b) for a,b in zip(measured["bounds_mm"],record["expected_bounds_mm"]))
    volume_error = abs(measured["volume_mm3"]-record["source_volume_mm3"])/record["source_volume_mm3"]
    if bound_error > allowed or volume_error > MAX_RELATIVE_VOLUME_ERROR:
        raise ValueError(f"Mesh does not match saved geometry: {name}; bounds {bound_error}, volume fraction {volume_error}")
    record.update(mesh=measured, max_bound_error_mm=bound_error, allowed_bound_error_mm=allowed,
                  relative_volume_error=volume_error, sha256=sha(bundle/record["filename"]))
    print(f"Exported {name}: no open edges, {measured['triangles']} triangles, {measured['nonmanifold_edges']} seam overlaps", flush=True)
    return record


def export_inputs():
    from tools.cad import signature, BUILD_INPUTS
    step = ROOT / "STEP/frankenduck.step"
    runtime = {"python": sys.version.split()[0]}
    for name in ("cadgen", "cadquery", "build123d", "cadquery-ocp", "shapely", "scipy", "numpy"):
        runtime[name] = importlib.metadata.version(name)
    inputs = dict(schema_version=1, source_step_sha256=sha(step), catalog_sha256=sha(ROOT/"source/components.json"),
        implementation={name:sha(ROOT/name) for name in ("tools/print_export.py", "tools/components.py")},
        runtime=runtime, mesh_tolerance_absolute_mm=MESH_TOLERANCE, mesh_angular_tolerance_rad=ANGULAR_TOLERANCE,
        mesher="OCCT Watson then Delabella via build123d/Lib3MF", mesh_min_size_mm=1e-5,
        build_input_key=signature(BUILD_INPUTS)["key"],
        assembly_data_sha256=sha(ROOT/"source/assembly_data.json"))
    key = hashlib.sha256(json.dumps(inputs, sort_keys=True).encode()).hexdigest()
    return inputs,key


def publish_bundle(records, bundle, stage, inputs, key):
    from tools.cad import build_status
    if build_status()[0] != "PASS" or export_inputs()[1] != key:
        raise ValueError("Model/export inputs changed; bundle not published")
    catalog,bindings = validate(ROOT)
    expected = {e["model"] for e in catalog["models"] if e["export"]}
    if {r["model"] for r in records} != expected or len(records) != len(expected):
        raise ValueError("Incomplete print bundle")
    for record in records:
        path = bundle/record["filename"]
        measured = inspect_3mf(path, allow_numerical_overlaps=True)
        if sha(path) != record["sha256"]:
            raise ValueError("Print file changed after conversion")
        bounds_error = max(abs(a-b) for a,b in zip(measured["bounds_mm"],record["expected_bounds_mm"]))
        volume_error = abs(measured["volume_mm3"]-record["source_volume_mm3"])/record["source_volume_mm3"]
        if bounds_error > record["allowed_bound_error_mm"] or volume_error > MAX_RELATIVE_VOLUME_ERROR:
            raise ValueError("Print file differs from native geometry")
        record["mesh"] = measured
    destination = ROOT/"printable"
    with (bundle/"PARTS.csv").open("w", newline="") as stream:
        fields = ["filename", "model", "quantity", "material_hint", "print_release", "occurrences"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            row = {k:record[k] for k in fields}
            row["occurrences"] = ";".join(o["ref"]+":"+o["name"] for o in record["occurrences"])
            writer.writerow(row)
    lines = ["# Frankenduck printable parts", "", f"This directory contains **{len(records)} individual 3MF files** for the printed components used in the current assembly. Purchased electronics, servos, horns, batteries, the heatsink, unused variants and review volumes are excluded.", "", "Open the files in `3mf/` directly in your slicer. Each file represents one native CAD solid and explicitly uses millimetres. Any numerical seam-overlap warnings are listed in the manifest; inspect affected meshes in your slicer. Left/right variants have separate filenames. Quantities and material hints are in [PARTS.csv](PARTS.csv).", "", "Print-release status is **unreviewed**. The exports have passed mesh integrity and geometry-conversion checks; these do not establish printability or physical fit. Choose material, orientation, supports and slicer settings for your printer. Material hints are listed in PARTS.csv; choose the final material and slicer settings for your printer.", "", "Geometry keeps its saved rest-pose axes, translated to centre X/Y and put its lowest Z at zero. No automatic print orientation is selected.", "", "The [manifest](manifest.json) records source/catalog hashes, dimensions, volumes, mesh checks and export settings. Regenerate from the repository root with `./cad build` followed by `./cad export print`. An unchanged bundle is reused; `--force` regenerates it. This directory is generated and committed. Edit the model/catalog and regenerate rather than editing these files.", "", "| File | Quantity | Material hint |", "| --- | ---: | --- |"]
    lines.extend(f"| [{r['model']}.3mf]({r['filename']}) | {r['quantity']} | {r['material_hint']} |" for r in records)
    (bundle/"README.md").write_text("\n".join(lines)+"\n")
    files = {str(p.relative_to(bundle)):sha(p) for p in sorted(bundle.rglob("*")) if p.is_file()}
    manifest = dict(generated_utc=datetime.now(timezone.utc).isoformat(), input_key=key, inputs=inputs,
        units="mm", source="STEP/frankenduck.step", export_count=len(records),
        orientation="Saved rest-pose axes; translation only, XY centred and min Z=0",
        max_relative_volume_error=MAX_RELATIVE_VOLUME_ERROR, parts=records, files=files)
    manifest["manifest_content_sha256"] = hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest()
    (bundle/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    previous = stage/"previous"
    if destination.exists():
        destination.rename(previous)
    try:
        bundle.rename(destination)
    except Exception:
        if previous.exists(): previous.rename(destination)
        raise
    print(f"Print bundle ready: {len(records)} validated 3MF files in printable/3mf/")
    return manifest


def export(force=False):
    from tools.cad import build_status, signature, BUILD_INPUTS
    if build_status()[0] != "PASS":
        raise ValueError("Print export requires a current build")
    catalog, bindings = validate(ROOT)
    selected = [e for e in catalog["models"] if e["export"]]
    if not selected:
        raise ValueError("Empty print selection")
    step = ROOT / "STEP/frankenduck.step"
    inputs,key = export_inputs()
    destination = ROOT / "printable"
    try:
        old = json.loads((destination/"manifest.json").read_text())
        actual = {str(p.relative_to(destination)) for p in destination.rglob("*") if p.is_file()}
        expected = set(old["files"]) | {"manifest.json"}
        unsigned = {k:v for k,v in old.items() if k != "manifest_content_sha256"}
        content_hash = hashlib.sha256(json.dumps(unsigned, sort_keys=True).encode()).hexdigest()
        unchanged = (old["input_key"] == key and actual == expected and
                     old.get("manifest_content_sha256") == content_hash and
                     all((destination/name).is_file() and sha(destination/name) == value for name,value in old["files"].items()))
    except (OSError, ValueError, KeyError):
        unchanged = False
    if unchanged and not force:
        print(f"Print bundle current: {len(selected)} 3MF files in printable/3mf/")
        return old
    # Native geometry is loaded only for an explicit export, never fast checks.
    from cadgen import read_scene
    from cadgen import build123d as bd
    scene = read_scene(step)
    scratch = ROOT / "scratch"
    scratch.mkdir(exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix="print-export-", dir=scratch))
    bundle = stage / "bundle"
    (bundle/"3mf").mkdir(parents=True)
    records = []
    try:
        for entry in selected:
            occurrence = bindings[entry["model"]][0]
            selection = scene.resolve(occurrence["ref"])
            if selection.occurrence_ref != occurrence["ref"]:
                raise ValueError("Occurrence selection mismatch")
            shape = selection.shape()
            geometry_source = "saved_assembly_step"
            comparison = None
            if not shape.is_valid:
                saved = shape
                shape = source_shape(entry["model"], occurrence)
                a, b = saved.bounding_box(), shape.bounding_box()
                bounds_error = max(abs(x-y) for x,y in zip(
                    (a.min.X,a.min.Y,a.min.Z,a.max.X,a.max.Y,a.max.Z),
                    (b.min.X,b.min.Y,b.min.Z,b.max.X,b.max.Y,b.max.Z)))
                volume_error = abs(saved.volume-shape.volume)
                if bounds_error > 1e-5 or volume_error > max(.02, saved.volume*1e-6):
                    raise ValueError(f"Source fallback differs from saved assembly: {entry['model']}")
                geometry_source = "current_source_factory"
                comparison = dict(saved_step_valid=False, bounds_error_mm=bounds_error,
                                  volume_difference_mm3=volume_error)
            solids = shape.solids()
            if len(solids) != 1 or not shape.is_valid or solids[0].volume <= 0:
                raise ValueError(f"Print model must be one valid positive solid: {entry['model']}")
            box = shape.bounding_box()
            xyz = [(box.min.X+box.max.X)/2, (box.min.Y+box.max.Y)/2, box.min.Z]
            normalized = shape.moved(bd.Location(tuple(-v for v in xyz)))
            normalized.label = entry["model"]
            bd.export_brep(normalized, stage/(entry["model"]+".brep"))

            expected_bounds = [-box.size.X/2, -box.size.Y/2, 0, box.size.X/2, box.size.Y/2, box.size.Z]
            records.append(dict(model=entry["model"], filename=f"3mf/{entry['model']}.3mf", quantity=len(bindings[entry["model"]]),
                occurrences=bindings[entry["model"]], role=entry["role"], print_release=entry["print_release"],
                material_hint=entry["material_hint"], source_volume_mm3=solid_volume(solids[0]),
                geometry_source=geometry_source, saved_step_comparison=comparison,
                translation_mm=[-v for v in xyz], expected_bounds_mm=expected_bounds))
        def launch(record):
            task = stage/(record["model"]+".json")
            failures = []
            for algorithm,tolerance in (("Watson",.01),("WatsonControlled",.05),("Delabella",.0005),("Watson",.00001)):
                task.write_text(json.dumps(record))
                try:
                    result = subprocess.run([sys.executable,"-m","tools.print_export","--worker",
                    str(task),str(bundle),algorithm,str(tolerance)],cwd=ROOT,text=True,capture_output=True,timeout=180)
                except subprocess.TimeoutExpired:
                    failures.append(dict(algorithm=algorithm, tolerance_mm=tolerance, exit_code="timeout", error="180 second conversion limit"))
                    continue
                print(result.stdout, end="", flush=True)
                if result.returncode == 0:
                    completed = json.loads(task.read_text())
                    completed["tessellation"]["prior_attempt_failures"] = failures
                    return completed,None
                failures.append(dict(algorithm=algorithm, tolerance_mm=tolerance, exit_code=result.returncode,
                    error=result.stderr[-1200:]))
            error = f"{record['model']}: all bounded native meshing attempts failed: {failures}"
            print("EXPORT FAILED " + error, flush=True)
            return None,error
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(launch, records))
        errors = [error for _,error in results if error]
        if errors:
            raise ValueError("Print bundle not published:\n"+"\n".join(errors))
        records = [record for record,_ in results]
        if (sha(step) != inputs["source_step_sha256"] or sha(ROOT/"source/components.json") != inputs["catalog_sha256"]
            or build_status()[0] != "PASS" or signature(BUILD_INPUTS)["key"] != inputs["build_input_key"]):
            raise RuntimeError("Model or catalog changed during export; bundle not replaced")
        return publish_bundle(records, bundle, stage, inputs, key)
    finally:
        shutil.rmtree(stage, ignore_errors=True)


if __name__ == "__main__":
    if len(sys.argv) == 6 and sys.argv[1] == "--worker":
        MESH_ALGORITHM = sys.argv[4]
        MESH_TOLERANCE = float(sys.argv[5])
        if MESH_ALGORITHM == "WatsonControlled":
            MESH_ALGORITHM = "Watson"
            MESH_SURFACE_CONTROL = MESH_INTERNAL_VERTICES = True
            MESH_MIN_SIZE = .01
            ANGULAR_TOLERANCE = .2
        task = Path(sys.argv[2])
        record = convert_worker(json.loads(task.read_text()), task.parent, sys.argv[3])
        task.write_text(json.dumps(record))
    else:
        raise SystemExit("Use ./cad export print")
