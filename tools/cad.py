"""Local CAD workflow: explicit builds, dependency-keyed checks and readable status."""
from pathlib import Path
from datetime import datetime, timezone
from contextlib import redirect_stdout, redirect_stderr
import argparse
import hashlib
import fcntl
import importlib
import importlib.metadata
import json
import os
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
LATEST = ROOT / "reports/latest"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))
BUILD_INPUTS = ["src/**/*.py", "src/**/*.js", "source/*.json",
                "STEP/imported/*.step", "requirements.txt", "build.sh"]
BUILD_OUTPUTS = ["STEP/frankenduck.step", "STEP/frankenduck.step.json"]


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".new")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def runtime():
    versions = {"python": sys.version.split()[0]}
    for package in ["cadgen", "cadquery", "build123d", "cadquery-ocp"]:
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = "missing"
    return versions


def signature(patterns):
    files = {}
    missing = []
    for pattern in patterns:
        matches = sorted(p for p in ROOT.glob(pattern) if p.is_file())
        if not matches:
            missing.append(pattern)
        for path in matches:
            files[str(path.relative_to(ROOT))] = digest(path)
    value = dict(files=files, missing=missing, runtime=runtime())
    value["key"] = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
    return value


def registry():
    entries = json.loads((ROOT / "checks/registry.json").read_text())["checks"]
    ids = [e["id"] for e in entries]
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate check IDs")
    return entries


def build_status():
    receipt = read_json(LATEST / "build.json")
    inputs, outputs = signature(BUILD_INPUTS), signature(BUILD_OUTPUTS)
    if inputs["missing"] or outputs["missing"]:
        return "STALE", "missing build inputs or outputs"
    if receipt is None:
        return "STALE", "no build receipt; run ./cad build"
    if receipt.get("status") != "PASS":
        return "FAILED", "last build did not complete successfully"
    if receipt.get("input_key") != inputs["key"]:
        return "STALE", "model source, inputs or runtime changed"
    if receipt.get("output_key") != outputs["key"]:
        return "STALE", "saved STEP or sidecar changed"
    return "PASS", "current"


def check_signature(entry):
    value = signature(entry["inputs"] + ["checks/registry.json", "checks/common.py",
                                        "checks/__init__.py", "tools/cad.py"])
    value["key"] = hashlib.sha256((value["key"] + json.dumps(entry, sort_keys=True)).encode()).hexdigest()
    return value


def check_status(entry, build):
    receipt = read_json(LATEST / (entry["id"] + ".receipt.json"))
    sig = check_signature(entry)
    if build != "PASS":
        return "STALE", "build is not current"
    if sig["missing"]:
        return "STALE", "missing: " + ", ".join(sig["missing"])
    if receipt is None:
        return "NOT RUN", "no current workflow receipt"
    if receipt.get("input_key") != sig["key"]:
        return "STALE", "check inputs or implementation changed"
    report = ROOT / entry["report"]
    if not report.exists() or receipt.get("report_sha256") != digest(report):
        return "STALE", "report missing or changed"
    return receipt["status"], f"{receipt['seconds']:.2f}s; current"


def status(save=True):
    build, reason = build_status()
    rows = []
    print(f"{'Build':24} {build:9} {reason}")
    for entry in registry():
        state, detail = check_status(entry, build)
        rows.append(dict(id=entry["id"], status=state, detail=detail,
                         purpose=entry["purpose"], coverage=entry["coverage"]))
        print(f"{entry['id']:24} {state:9} {detail}")
    print("Motion sweep             NOT IMPLEMENTED")
    result = dict(generated_utc=now(), build=dict(status=build, detail=reason),
                  checks=rows, all_maintained_checks_passed=build == "PASS" and
                  all(r["status"] == "PASS" for r in rows),
                  unimplemented=["motion collision sweep", "tool/connector insertion paths",
                                 "physical cable bending", "thermal performance"])
    if save:
        write_json(LATEST / "summary.json", result)
    return result


def build(force=False):
    LATEST.mkdir(parents=True, exist_ok=True)
    inputs = signature(BUILD_INPUTS)
    if inputs["missing"]:
        raise ValueError("Missing build inputs: " + ", ".join(inputs["missing"]))
    start = time.monotonic()
    write_json(LATEST / "build.json", dict(status="RUNNING", generated_utc=now()))
    command = [str(ROOT / "build.sh")] + (["--force"] if force else [])
    print("Building saved STEP and sidecar; log: reports/latest/build.log", flush=True)
    with (LATEST / "build.log").open("w") as log:
        code = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT).returncode
    after, outputs = signature(BUILD_INPUTS), signature(BUILD_OUTPUTS)
    passed = code == 0 and inputs["key"] == after["key"] and not outputs["missing"]
    write_json(LATEST / "build.json", dict(status="PASS" if passed else "FAILED",
        generated_utc=now(), seconds=time.monotonic()-start, input_key=inputs["key"],
        output_key=outputs["key"], inputs=inputs, outputs=outputs, returncode=code))
    if not passed:
        raise RuntimeError("Build failed or inputs changed during build; see reports/latest/build.log")
    print(f"Build PASS ({time.monotonic()-start:.2f}s)", flush=True)


def run_checks(suite, force=False):
    build_state, reason = build_status()
    if build_state != "PASS":
        print(f"Checks blocked: {reason}. Run ./cad build.", file=sys.stderr)
        status()
        return 1
    entries = [e for e in registry() if suite == "full" or suite in e["suites"]]
    failed = False
    for entry in entries:
        state, detail = check_status(entry, build_state)
        if state == "PASS" and not force:
            print(f"{entry['id']:24} PASS      cached ({detail})", flush=True)
            continue
        sig = check_signature(entry)
        if sig["missing"]:
            print(f"{entry['id']}: missing inputs {sig['missing']}", file=sys.stderr)
            failed = True
            continue
        start = time.monotonic()
        receipt_path = LATEST / (entry["id"] + ".receipt.json")
        write_json(receipt_path, dict(status="RUNNING", input_key=sig["key"], seconds=0))
        log_path = LATEST / (entry["id"] + ".log")
        print(f"{entry['id']:24} RUNNING   {entry['purpose']}", flush=True)
        error = None
        try:
            with log_path.open("w") as log, redirect_stdout(log), redirect_stderr(log):
                module, name = entry["call"].split(":")
                report = getattr(importlib.import_module(module), name)()
                if report.get("passed") is False:
                    raise AssertionError("Check reported failure")
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            with log_path.open("a") as log:
                log.write(traceback.format_exc())
        if sig["key"] != check_signature(entry)["key"] or build_status()[0] != "PASS":
            error = "Inputs changed during check; result invalidated"
        report_path = ROOT / entry["report"]
        if not report_path.exists():
            error = error or "Check did not write its report"
        elapsed = time.monotonic()-start
        write_json(receipt_path, dict(status="FAILED" if error else "PASS", input_key=sig["key"],
            generated_utc=now(), seconds=elapsed, error=error,
            report_sha256=digest(report_path) if report_path.exists() else None,
            inputs=sig, coverage=entry["coverage"], report=entry["report"]))
        print(f"{entry['id']:24} {'FAILED' if error else 'PASS':9} {elapsed:.2f}s" +
              (f"; {error}; see {log_path.relative_to(ROOT)}" if error else ""), flush=True)
        failed |= error is not None
    result = status()
    if suite == "full" and not failed and result["all_maintained_checks_passed"]:
        manifest()
    return int(failed)


def manifest():
    """A current receipt, without importing historic reports as current passes."""
    patterns = ["src/**/*.py", "src/**/*.js", "source/**/*", "STEP/imported/*",
                "checks/*.py", "checks/registry.json", "tools/*.py", "cad", "build.sh",
                "requirements.txt", "STEP/frankenduck.step*", "reports/latest/*.json"]
    paths = {p for pattern in patterns for p in ROOT.glob(pattern) if p.is_file()
             and p.name not in {"provenance_manifest.json", "summary.json", "lan_viewer.json"}
             and "__pycache__" not in p.parts}
    write_json(LATEST / "provenance_manifest.json", dict(generated_utc=now(), runtime=runtime(),
        validation=read_json(LATEST / "summary.json"), files={str(p.relative_to(ROOT)):
        dict(bytes=p.stat().st_size, sha256=digest(p)) for p in sorted(paths)}))
    print(f"Provenance: {len(paths)} hashed files")


def review(target):
    if build_status()[0] != "PASS":
        raise RuntimeError("Build is stale; run ./cad build before review")
    if target == "pod":
        subprocess.run([sys.executable, "tools/part_review.py"], cwd=ROOT, check=True)
        return
    command = [sys.executable, "src/radxa_fit_review.py"] if target == "radxa" else None
    if command:
        subprocess.run(command, cwd=ROOT, check=True)
    path = "STEP/radxa_fit_review.step" if target == "radxa" else "STEP/frankenduck.step"
    print(f"Review artifact: {ROOT / path}\nOpen with ./cad view")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("build", help="build STEP + sidecar and record freshness")
    p.add_argument("--force", action="store_true")
    p = sub.add_parser("check", help="run named maintained suites")
    p.add_argument("suite", choices=["fast", "radxa", "full"])
    p.add_argument("--force", action="store_true", help="ignore passing check cache")
    p = sub.add_parser("parts", help="list explicit component roles and print selection")
    p.add_argument("--json", action="store_true")
    p = sub.add_parser("export", help="export a tracked bundle of classified printed parts")
    p.add_argument("target", choices=["print"])
    p.add_argument("--force", action="store_true", help="regenerate an unchanged print bundle")
    sub.add_parser("status", help="show current, stale and missing results")
    p = sub.add_parser("review", help="prepare the saved assembly or Radxa review model, or the stick pod pictures")
    p.add_argument("target", choices=["assembly", "radxa", "pod"])
    sub.add_parser("view", help="start/reuse the LAN viewer")
    args = parser.parse_args()
    if sys.flags.optimize:
        raise RuntimeError("Checks require Python assertions; disable PYTHONOPTIMIZE")
    os.chdir(ROOT)
    LATEST.mkdir(parents=True, exist_ok=True)
    # The descriptor stays alive through the invocation. A second writer must
    # not replace reports, rebuild geometry or reuse a scene during another run.
    lock = (LATEST / ".workflow.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("Another CAD workflow command is running; retry when it finishes")
    if args.command == "build":
        build(args.force)
        status()
    elif args.command == "check":
        return run_checks(args.suite, args.force)
    elif args.command == "parts":
        from tools.components import list_parts
        list_parts(args.json)
    elif args.command == "export":
        if run_checks("fast"):
            return 1
        from tools.print_export import export
        export(args.force)
    elif args.command == "status":
        status()
    elif args.command == "review":
        review(args.target)
    elif args.command == "view":
        return subprocess.run([str(ROOT / "view.sh")], cwd=ROOT).returncode
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"CAD workflow: {exc}", file=sys.stderr)
        raise SystemExit(1)
