"""Review the saved mount together with explicit plug and cable allowances."""
from pathlib import Path
from cadgen import step, read_scene, build123d as bd
from lib.radxa_clearance import clearance_shapes
from lib.radxa_6530b_parameters import fan_inlet_allowance

ROOT = Path(__file__).resolve().parents[1]


@step(out="../STEP/radxa_fit_review.step")
def radxa_fit_review():
    scene = read_scene(ROOT / "STEP/frankenduck.step")
    children = []
    # Context remains in its actual assembly placement. The complete robot
    # output contains only physical CAD parts, without these review volumes.
    for ref, opacity in [("#o1.2", 0.15), ("#o1.3", 0.3), ("#o1.6", 1.0),
                         ("#o1.7", 0.35), ("#o1.14", 0.15), ("#o1.15", 1.0),
                         ("#o1.16", 0.35), ("#o1.20", 0.3), ("#o1.74", 1.0)]:
        shape = scene.resolve(ref).shape()
        if opacity < 1.0:
            shape.color = bd.Color(0.72, 0.74, 0.78, opacity)
            for solid in shape.solids():
                solid.color = shape.color
        children.append(shape)
    children.extend(clearance_shapes())
    children.append(fan_inlet_allowance())
    result = bd.Compound(children=children)
    result.label = "Radxa_mount_plugs_and_upward_cables_review"
    return result


if __name__ == "__main__":
    radxa_fit_review()
