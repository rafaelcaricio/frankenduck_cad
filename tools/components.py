"""Explicit component intent and active assembly bindings; no CAD imports."""
from collections import Counter
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
ROLES = {"printed_part", "purchased_component", "fabricated_part"}
FIDELITIES = {"design_geometry", "imported_reference", "dimensioned_approximation", "verified_vendor_cad"}


def variant_key(component, params):
    return component, json.dumps(params, sort_keys=True)


def validate(root=ROOT):
    def read(name):
        return json.loads((root / name).read_text())
    catalog = read("source/components.json")
    if catalog.get("schema_version") != 1 or catalog.get("units") != "mm":
        raise ValueError("Component catalog requires schema 1 and mm units")
    models = catalog["models"]
    by_model = {}
    for entry in models:
        name = entry["model"]
        if not re.fullmatch(r"[a-z_][a-z0-9_]*", name) or name in by_model:
            raise ValueError(f"Invalid or duplicate model: {name}")
        if entry["role"] not in ROLES or entry["geometry_fidelity"] not in FIDELITIES:
            raise ValueError(f"Invalid role or fidelity: {name}")
        if type(entry["export"]) is not bool or type(entry["active_variant"]) is not bool:
            raise ValueError(f"Export/active flags must be booleans: {name}")
        if not isinstance(entry["params"], dict) or not entry["notes"] or not entry["material_hint"]:
            raise ValueError(f"Missing variant parameters or explanatory metadata: {name}")
        if entry["role"] == "printed_part":
            if entry["print_release"] not in {"unreviewed", "checked", "released"}:
                raise ValueError(f"Missing print-release status: {name}")
            if entry["export"] != entry["active_variant"]:
                raise ValueError(f"Active printed parts must be in the print set: {name}")
        elif entry["export"] or entry["print_release"] != "not_applicable":
            raise ValueError(f"Non-print components cannot be exported: {name}")
        by_model[name] = entry
    scripts = {p.stem for p in (root / "src/parts").glob("*.py") if p.stem != "__init__"}
    if set(by_model) != scripts:
        raise ValueError(f"Unclassified scripts: {sorted(scripts-set(by_model))}; missing scripts: {sorted(set(by_model)-scripts)}")
    source = read("source/assembly_data.json")
    variants = read("source/variants.json")
    active = {e["model"]: e for e in models if e["active_variant"]}
    if set(active) != {v["model"] for v in variants}:
        raise ValueError("Active catalog models differ from assembly variants")
    by_variant = {}
    for variant in variants:
        entry = active[variant["model"]]
        if entry["component"] != variant["component"] or entry["params"] != variant["params"]:
            raise ValueError(f"Variant binding differs: {entry['model']}")
        key = variant_key(entry["component"], entry["params"])
        if key in by_variant:
            raise ValueError(f"Ambiguous active variant: {key}")
        by_variant[key] = entry
    bindings = {e["model"]: [] for e in models}
    instances = source["instances"]
    for i, occurrence in enumerate(instances, 1):
        key = variant_key(occurrence["component"], occurrence.get("params", {}))
        if key not in by_variant:
            raise ValueError(f"Unclassified assembly occurrence: {occurrence['name']}")
        bindings[by_variant[key]["model"]].append(dict(ref=f"#o1.{i}", name=occurrence["name"]))
    review_ids = []
    for entry in catalog["review_geometry"]:
        if entry["role"] != "review_geometry" or not entry["provider"] or not entry["notes"]:
            raise ValueError("Invalid review-geometry classification")
        review_ids.append(entry["id"])
    if len(review_ids) != len(set(review_ids)):
        raise ValueError("Duplicate review geometry IDs")
    for entry in models:
        if entry["active_variant"] and not bindings[entry["model"]]:
            raise ValueError(f"Unused model marked active: {entry['model']}")
    return catalog, bindings


def summary(root=ROOT):
    catalog, bindings = validate(root)
    counts = Counter(e["role"] for e in catalog["models"])
    occurrences = Counter()
    for entry in catalog["models"]:
        occurrences[entry["role"]] += len(bindings[entry["model"]])
    return dict(passed=True, units="mm", model_count=len(catalog["models"]),
        model_roles=dict(counts), occurrence_roles=dict(occurrences),
        print_model_count=sum(e["export"] for e in catalog["models"]),
        review_geometry_count=len(catalog["review_geometry"]),
        models=[dict(model=e["model"], role=e["role"], export=e["export"],
                     geometry_fidelity=e["geometry_fidelity"], print_release=e["print_release"],
                     quantity=len(bindings[e["model"]]), occurrences=bindings[e["model"]])
                for e in catalog["models"]])


def list_parts(as_json=False):
    report = summary()
    if as_json:
        print(json.dumps(report, indent=2))
        return
    print(f"{'Model':44} {'Role':20} {'Qty':>3} {'Print export':12} Release")
    for row in report["models"]:
        print(f"{row['model']:44} {row['role']:20} {row['quantity']:3} {'yes' if row['export'] else 'no':12} {row['print_release']}")
    print(f"Print set: {report['print_model_count']} models; {report['review_geometry_count']} review volumes excluded")
