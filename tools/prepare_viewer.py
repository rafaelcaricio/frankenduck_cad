"""Prepare a project-local client with the cadgen 0.6.5 face-colour fix.

The installed runtime stays intact. view.sh selects this generated client;
its kernel, server, geometry and caches still come from the pinned environment.
Exact anchors fail closed if a future viewer needs a different patch.
"""
from __future__ import annotations

import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
MODULE = "assets/surfMeshData-Bz3U9DZp.js"
WORKER = "assets/surfWorker-BExKymwi.js"
HELPER = ROOT / "tools/viewer/face_colours.mjs"
OUTPUT_HELPER = "assets/frankenduck-face-colours.js"


def patch_mesh_module(source: str) -> str:
    substitutions = [
        ("g=h?n(h):null,_={id:", "g=h?n(h):null,v=faceColorData(s,h),_={id:"),
        ("hasSourceColors:!!g,", "hasSourceColors:!!g||v.hasFaceColors,"),
        ("colors:new Float32Array,", "colors:v.colors,"),
        ("has_source_colors:!!g,", "has_source_colors:!!g||v.hasFaceColors,"),
    ]
    for before, after in substitutions:
        if source.count(before) != 1:
            raise RuntimeError(f"Viewer changed; expected one patch anchor: {before}")
        source = source.replace(before, after)
    return ('import{buildFaceColorData as faceColorData}'
            'from"./frankenduck-face-colours.js";\n' + source)


def patch_mesh_worker(source: str) -> str:
    substitutions = [
        ("f=d?Ln(d):null,p={id:", "f=d?Ln(d):null,faceData=faceColorData(r,d),p={id:"),
        ("hasSourceColors:!!f,", "hasSourceColors:!!f||faceData.hasFaceColors,"),
        ("colors:/* @__PURE__ */ new Float32Array,", "colors:faceData.colors,"),
        ("has_source_colors:!!f,", "has_source_colors:!!f||faceData.hasFaceColors,"),
    ]
    for before, after in substitutions:
        if source.count(before) != 1:
            raise RuntimeError(f"Viewer worker changed; expected one patch anchor: {before}")
        source = source.replace(before, after)
    return ('import{buildFaceColorData as faceColorData}'
            'from"./frankenduck-face-colours.js";\n' + source)


def prepare() -> Path:
    import cadgen
    version = importlib.metadata.version("cadgen")
    if version != "0.6.5":
        raise RuntimeError(f"Review the viewer patch for cadgen {version}; expected 0.6.5")
    bundled = Path(cadgen.__file__).resolve().parent / "_runtime/viewer"
    original = (bundled / MODULE).read_text()
    patched = patch_mesh_module(original)
    original_worker = (bundled / WORKER).read_text()
    patched_worker = patch_mesh_worker(original_worker)
    target = ROOT / ".cadgen/viewer-dist"
    receipt_path = target / "frankenduck-viewer-patch.json"
    digest = lambda data: hashlib.sha256(data).hexdigest()
    receipt = dict(cadgen_version=version,
                   bundled_module_sha256=digest(original.encode()),
                   patched_module_sha256=digest(patched.encode()),
                   bundled_worker_sha256=digest(original_worker.encode()),
                   patched_worker_sha256=digest(patched_worker.encode()),
                   helper_sha256=digest(HELPER.read_bytes()),
                   patcher_sha256=digest(Path(__file__).read_bytes()),
                   purpose="Preserve SURF face colours on one CAD component")
    suffix = digest(json.dumps(receipt, sort_keys=True).encode())[:12]
    # Bundled assets use immutable cache headers. Give changed entrypoints new
    # URLs so an already-open tab cannot retain the unpatched converter.
    names = {asset.name: f"{asset.stem}-frankenduck-{suffix}.js"
             for asset in (bundled / "assets").glob("*.js")}
    if receipt_path.exists() and json.loads(receipt_path.read_text()) == receipt:
        if ((target / MODULE).read_text() == patched
                and (target / WORKER).read_text() == patched_worker
                and (target / OUTPUT_HELPER).read_bytes() == HELPER.read_bytes()
                and (target / "index.html").exists()):
            return target
    shutil.copytree(bundled, target, dirs_exist_ok=True)
    for source in bundled.rglob("*"):
        if source.suffix not in {".js", ".html"} or not source.is_file():
            continue
        relative = source.relative_to(bundled).as_posix()
        text = {MODULE: patched, WORKER: patched_worker}.get(relative, source.read_text())
        for before, after in names.items():
            text = text.replace(before, after)
        output = target / source.relative_to(bundled)
        output = output.with_name(names.get(output.name, output.name))
        output.write_text(text)
    (target / MODULE).write_text(patched)
    (target / WORKER).write_text(patched_worker)
    shutil.copy2(HELPER, target / OUTPUT_HELPER)
    receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    return target


if __name__ == "__main__":
    print(prepare())
