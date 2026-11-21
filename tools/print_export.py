"""Produce a tracked, atomic 3MF and STL print bundle from classified saved geometry."""
from collections import Counter
import ast
import importlib
from concurrent.futures import ThreadPoolExecutor
import os
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
from tools.print_mesh import (sha, solid_volume, transform, inspect_3mf, source_shape, complete_planar_boundaries, complete_small_surface_seams, check_face_triangles, write_native_3mf, write_gmsh_3mf, validate_gmsh_mesh, write_export_3mf, geometric_difference, convert_worker, write_verified_stl, MESH_TOLERANCE, ANGULAR_TOLERANCE, MAX_RELATIVE_VOLUME_ERROR, ROOT)


# Ordered mesh recipes. A part tries the recipe that passed for it last time
# first, then the others in this order; every recipe meets the same checks.
MESH_RECIPES = (("Watson",.01),("WatsonControlled",.005),("Delabella",.0005),("Watson",.00001),
    ("WatsonFineControlled",.01),("DEFAULT",.01),("DEFAULT",.0005),("Gmsh",.4),("Gmsh",.1))
# Parts that try their own recipes before the general ladder. The soles are
# lofts through splines with over 800 knots around; Watson 0.01 mm times out on
# them, so they start at the recipe that passes. A wider angle (0.35 rad) cut
# them to 0.8 M triangles but moved sampled points up to 0.38 mm off the CAD
# surface (0.07 mm at this recipe), so it was rejected.
PART_RECIPES = {"sole_left_native": (("WatsonControlled", .005),),
                "sole_left_native_right": (("WatsonControlled", .005),)}
# Source of the worker and mesher; a cached mesh is valid only for these exact files.
MESH_IMPLEMENTATION = ("tools/print_mesh.py", "tools/gmsh_mesh.py")
# Workers are single-threaded processes; leave cores for the system and the parent.
WORKERS = max(2, min(8, (os.cpu_count() or 4) - 2))
# Time bound for a part's previously passing recipe; others keep 180 s (Gmsh 300 s).
PASSED_RECIPE_SECONDS = 600


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def export_inputs():
    from tools.cad import signature, BUILD_INPUTS
    step = ROOT / "STEP/frankenduck.step"
    runtime = {"python": sys.version.split()[0]}
    for name in ("cadgen", "cadquery", "build123d", "cadquery-ocp", "shapely", "scipy", "numpy"):
        runtime[name] = importlib.metadata.version(name)
    # Everything that can change a part's mesh, apart from the part's own geometry.
    mesh = dict(implementation={name:sha(ROOT/name) for name in MESH_IMPLEMENTATION},
        runtime=runtime, mesh_tolerance_absolute_mm=MESH_TOLERANCE, mesh_angular_tolerance_rad=ANGULAR_TOLERANCE,
        mesher="Surface-controlled OCCT with sampled CAD-face validation; isolated Gmsh fallback", mesh_min_size_mm=1e-5,
        recipes=[list(r) for r in MESH_RECIPES],
        part_recipes={k:[list(r) for r in v] for k,v in sorted(PART_RECIPES.items())})
    inputs = dict(schema_version=2, source_step_sha256=sha(step), catalog_sha256=sha(ROOT/"source/components.json"),
        implementation={name:sha(ROOT/name) for name in ("tools/print_export.py", "tools/components.py")},
        mesh=mesh, mesh_key=digest(mesh),
        build_input_key=signature(BUILD_INPUTS)["key"],
        assembly_data_sha256=sha(ROOT/"source/assembly_data.json"),
        license_files={name:sha(ROOT/name) for name in ("LICENSE", "NOTICE.md")})
    return inputs,digest(inputs)


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
        if record["mesh_reused"]:
            # Byte-identical to files that passed every check for this exact
            # geometry and mesh implementation; re-inspection would repeat it.
            if (sha(path) != record["sha256"] or record["stl_filename"] != f"stl/{record['model']}.stl"
                or sha(bundle/record["stl_filename"]) != record["stl"]["sha256"]):
                raise ValueError(f"Reused print file changed: {record['model']}")
            continue
        measured = inspect_3mf(path, allow_numerical_overlaps=True)
        if sha(path) != record["sha256"]:
            raise ValueError("Print file changed after conversion")
        bounds_error = max(abs(a-b) for a,b in zip(measured["bounds_mm"],record["expected_bounds_mm"]))
        volume_error = abs(measured["volume_mm3"]-record["source_volume_mm3"])/record["source_volume_mm3"]
        if bounds_error > record["allowed_bound_error_mm"] or volume_error > MAX_RELATIVE_VOLUME_ERROR:
            raise ValueError("Print file differs from native geometry")
        record["mesh"] = measured
        record["stl_filename"] = f"stl/{record['model']}.stl"
        record["stl"] = write_verified_stl(path, bundle/record["stl_filename"], measured)
    destination = ROOT/"printable"
    with (bundle/"PARTS.csv").open("w", newline="") as stream:
        fields = ["filename", "stl_filename", "model", "quantity", "material_hint", "print_release", "occurrences"]
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for record in records:
            row = {k:record[k] for k in fields}
            row["occurrences"] = ";".join(o["ref"]+":"+o["name"] for o in record["occurrences"])
            writer.writerow(row)
    lines = ["# Frankenduck printable parts", "", f"This directory contains **{len(records)} printable models, each supplied as 3MF and STL** for the printed components used in the current assembly. Purchased electronics, servos, horns, batteries, the heatsink, unused variants and review volumes are excluded.", "", "Open the files in `3mf/` directly in your slicer. Each file represents one native CAD solid and explicitly uses millimetres. Any numerical seam-overlap warnings are listed in the manifest; inspect affected meshes in your slicer. Left/right variants have separate filenames. Quantities and material hints are in [PARTS.csv](PARTS.csv).", "", "These parts are an adaptation of the Microduck by Pollen Robotics (https://github.com/pollen-robotics/microduck_rl), derived from its 3D model files and modified. Copyright 2026 Rafael Caricio. Licensed under CC BY-NC-SA 4.0 (https://creativecommons.org/licenses/by-nc-sa/4.0/): credit Pollen Robotics and Rafael Caricio, no commercial use, share adaptations under the same license. The legal code is in [LICENSE](LICENSE) and the attribution in [NOTICE.md](NOTICE.md). Pollen Robotics does not endorse this project.", "", "Print-release status is **unreviewed**. The exports have passed mesh integrity and geometry-conversion checks; these do not establish printability or physical fit. Choose material, orientation, supports and slicer settings for your printer. Material hints are listed in PARTS.csv; choose the final material and slicer settings for your printer.", "", "Geometry keeps its saved rest-pose axes, translated to centre X/Y and put its lowest Z at zero. No automatic print orientation is selected.", "", "The [manifest](manifest.json) records source/catalog hashes, dimensions, volumes, mesh checks and export settings. Regenerate from the repository root with `./cad build` followed by `./cad export print`. An unchanged bundle is reused; `--force` regenerates it. This directory is generated and committed. Edit the model/catalog and regenerate rather than editing these files.", "", "| File | Quantity | Material hint |", "| --- | ---: | --- |"]
    lines.extend(f"| [{r['model']}.3mf]({r['filename']}) | {r['quantity']} | {r['material_hint']} |" for r in records)
    lines.extend(["", "## STL copies", "", "Matching binary STL files are in `stl/`. Import them as millimetres: STL cannot encode units or colors. They contain the same validated triangles as the 3MF files, checked again after float32 conversion. The CAD assembly carries the viewer colors. See `docs/bearing-revision.md` in the project for the bearing revision's unresolved movement and fit limitations."])
    (bundle/"README.md").write_text("\n".join(lines)+"\n")
    for name in ("LICENSE", "NOTICE.md"):
        shutil.copyfile(ROOT/name, bundle/name)
    files = {str(p.relative_to(bundle)):sha(p) for p in sorted(bundle.rglob("*")) if p.is_file()}
    manifest = dict(generated_utc=datetime.now(timezone.utc).isoformat(), input_key=key, inputs=inputs,
        units="mm", source="STEP/frankenduck.step", export_count=len(records),
        orientation="Saved rest-pose axes; translation only, XY centred and min Z=0",
        max_relative_volume_error=MAX_RELATIVE_VOLUME_ERROR, conversion_workers=WORKERS,
        reused_meshes=sum(r["mesh_reused"] for r in records), parts=records, files=files)
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
    reused = sum(r["mesh_reused"] for r in records)
    print(f"Print bundle ready: {len(records)} validated 3MF/STL pairs in printable/ ({reused} reused unchanged)")
    return manifest


def export(force=False):
    from tools.cad import build_status, signature, BUILD_INPUTS
    if build_status()[0] != "PASS":
        raise ValueError("Print export requires a current build")
    catalog, bindings = validate(ROOT)
    selected = [e for e in catalog["models"] if e["export"]]
    if not selected:
        raise ValueError("Empty print selection")
    unknown = set(PART_RECIPES) - {e["model"] for e in selected}
    if unknown:
        raise ValueError(f"Part mesh recipes name parts outside the print selection: {sorted(unknown)}")
    step = ROOT / "STEP/frankenduck.step"
    inputs,key = export_inputs()
    destination = ROOT / "printable"
    try:
        old = json.loads((destination/"manifest.json").read_text())
        actual = {str(p.relative_to(destination)) for p in destination.rglob("*") if p.is_file()}
        expected = set(old["files"]) | {"manifest.json"}
        unsigned = {k:v for k,v in old.items() if k != "manifest_content_sha256"}
        content_hash = hashlib.sha256(json.dumps(unsigned, sort_keys=True).encode()).hexdigest()
        intact = (actual == expected and old.get("manifest_content_sha256") == content_hash and
                  all((destination/name).is_file() and sha(destination/name) == value for name,value in old["files"].items()))
        unchanged = intact and old["input_key"] == key
        previous_parts = {p["model"]:p for p in old["parts"]} if intact else {}
    except (OSError, ValueError, KeyError):
        unchanged, previous_parts = False, {}
    if unchanged and not force:
        print(f"Print bundle current: {len(selected)} 3MF/STL pairs in printable/")
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
    def reuse(entry, geometry):
        """Previous verified files for byte-identical saved geometry and mesh inputs.

        The previous run checked this exact geometry (validity, rebuild agreement,
        volume, bounds, mesh and STL), so none of that is recomputed here.
        """
        prior = previous_parts.get(entry["model"])
        if (force or not prior or prior.get("geometry_sha256") != geometry
            or prior.get("mesh_key") != inputs["mesh_key"]):
            return None
        for name,value in ((prior["filename"],prior["sha256"]),(prior["stl_filename"],prior["stl"]["sha256"])):
            (bundle/name).parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(destination/name, bundle/name)
            if sha(bundle/name) != value:
                return None
        reused = dict(prior)
        reused.update(quantity=len(bindings[entry["model"]]), occurrences=bindings[entry["model"]],
                      role=entry["role"], print_release=entry["print_release"],
                      material_hint=entry["material_hint"], mesh_reused=True)
        return reused
    try:
        for entry in selected:
            occurrence = bindings[entry["model"]][0]
            selection = scene.resolve(occurrence["ref"])
            if selection.occurrence_ref != occurrence["ref"]:
                raise ValueError("Occurrence selection mismatch")
            shape = selection.shape()
            # Cache identity is the saved STEP geometry: its BREP is stable between
            # runs, unlike a rebuilt fallback solid. A current build guarantees the
            # saved STEP matches the part sources.
            saved_box = shape.bounding_box()
            saved_origin = [(saved_box.min.X+saved_box.max.X)/2, (saved_box.min.Y+saved_box.max.Y)/2, saved_box.min.Z]
            bd.export_brep(shape.moved(bd.Location(tuple(-v for v in saved_origin))), stage/(entry["model"]+".saved.brep"))
            geometry = sha(stage/(entry["model"]+".saved.brep"))
            cached = reuse(entry, geometry)
            if cached:
                print(f"Reused {entry['model']}: geometry and mesh inputs unchanged", flush=True)
                records.append(cached)
                continue
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
            box = saved_box if geometry_source == "saved_assembly_step" else shape.bounding_box()
            xyz = [(box.min.X+box.max.X)/2, (box.min.Y+box.max.Y)/2, box.min.Z]
            normalized = shape.moved(bd.Location(tuple(-v for v in xyz)))
            normalized.label = entry["model"]
            bd.export_brep(normalized, stage/(entry["model"]+".brep"))

            expected_bounds = [-box.size.X/2, -box.size.Y/2, 0, box.size.X/2, box.size.Y/2, box.size.Z]
            records.append(dict(model=entry["model"], filename=f"3mf/{entry['model']}.3mf", quantity=len(bindings[entry["model"]]),
                occurrences=bindings[entry["model"]], role=entry["role"], print_release=entry["print_release"],
                material_hint=entry["material_hint"], source_volume_mm3=solid_volume(solids[0]),
                geometry_source=geometry_source, saved_step_comparison=comparison,
                translation_mm=[-v for v in xyz], expected_bounds_mm=expected_bounds,
                geometry_sha256=geometry, mesh_key=inputs["mesh_key"], mesh_reused=False))
        def recipe_order(record):
            """Previously passing recipe first; the rest keep their order."""
            prior = previous_parts.get(record["model"]) or {}
            passed = prior.get("mesh_recipe")
            if passed is None and "tessellation" in prior:
                # Older manifests: the ladder stops at its first success.
                failed = [(f["algorithm"],f["tolerance_mm"]) for f in prior["tessellation"].get("prior_attempt_failures",[])]
                if failed == [tuple(r) for r in MESH_RECIPES[:len(failed)]] and len(failed) < len(MESH_RECIPES):
                    passed = MESH_RECIPES[len(failed)]
            preferred = list(PART_RECIPES.get(record["model"], ()))
            passed = tuple(passed) if passed else None
            passed = passed if passed in MESH_RECIPES or passed in preferred else None
            first = preferred + ([passed] if passed and passed not in preferred else [])
            # Part-specific and previously passing recipes get the longer bound.
            return first + [r for r in MESH_RECIPES if r not in first], set(first)
        def launch(record):
            task = stage/(record["model"]+".json")
            failures = []
            order, known = recipe_order(record)
            for algorithm,tolerance in order:
                # Part-specific and last-passing recipes get a longer bound, so load from
                # parallel workers cannot push a part onto a different recipe.
                limit = PASSED_RECIPE_SECONDS if (algorithm,tolerance) in known else (300 if algorithm=="Gmsh" else 180)
                task.write_text(json.dumps(record))
                try:
                    result = subprocess.run([sys.executable,"-m","tools.print_mesh","--worker",
                    str(task),str(bundle),algorithm,str(tolerance)],cwd=ROOT,text=True,capture_output=True,timeout=limit)
                except subprocess.TimeoutExpired:
                    failures.append(dict(algorithm=algorithm, tolerance_mm=tolerance, exit_code="timeout", error=f"{limit} second conversion limit"))
                    continue
                print(result.stdout, end="", flush=True)
                if result.returncode and "Exported " in result.stdout:
                    # A worker that meshes and then fails must not fall through to other recipes.
                    raise RuntimeError(f"Mesh worker exited {result.returncode} after exporting {record['model']}: {result.stderr[-600:]}")
                if result.returncode == 0:
                    completed = json.loads(task.read_text())
                    completed["tessellation"]["prior_attempt_failures"] = failures
                    completed.update(mesh_recipe=[algorithm,tolerance], mesh_reused=False)
                    return completed,None
                failures.append(dict(algorithm=algorithm, tolerance_mm=tolerance, exit_code=result.returncode,
                    error=result.stderr[-1200:]))
            error = f"{record['model']}: all bounded native meshing attempts failed: {failures}"
            print("EXPORT FAILED " + error, flush=True)
            return None,error
        # Start the heaviest previous meshes first so they do not finish last.
        weight = lambda r: (previous_parts.get(r["model"]) or {}).get("mesh",{}).get("triangles",0)
        with ThreadPoolExecutor(max_workers=WORKERS) as pool:
            pending = [r for r in records if not r["mesh_reused"]]
            futures = {r["model"]:pool.submit(launch, r) for r in sorted(pending, key=weight, reverse=True)}
            results = [futures[r["model"]].result() if not r["mesh_reused"] else (r,None) for r in records]
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
    raise SystemExit("Use ./cad export print")
