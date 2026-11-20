"""Regression coverage for rounding-error recovery without filling real openings."""
import unittest

from tools.print_export import complete_planar_boundaries, geometric_difference, solid_volume


class NumericalGeometry(unittest.TestCase):
    def test_equal_solids_have_zero_bidirectional_difference(self):
        from cadgen import build123d as bd
        first = bd.Box(2, 3, 4)
        second = bd.Box(2, 3, 4)
        self.assertEqual(geometric_difference(first, second), [0, 0])

    def test_actual_geometry_change_remains_visible(self):
        from cadgen import build123d as bd
        values = geometric_difference(bd.Box(2, 3, 4), bd.Box(1, 3, 4))
        self.assertAlmostEqual(sum(values), 12)


    def test_reference_volume_is_exact_on_bspline_faces(self):
        import math
        from cadgen import build123d as bd
        from OCP.BRepBuilderAPI import BRepBuilderAPI_NurbsConvert
        sphere = BRepBuilderAPI_NurbsConvert(bd.Sphere(10).wrapped, True).Shape()
        self.assertAlmostEqual(solid_volume(sphere) / (4 / 3 * math.pi * 1000), 1, places=5)


class PlanarTriangulation(unittest.TestCase):
    vertices = [(0,0,0),(1,0,0),(1,1,0),(0,1,0),
                (0,0,1),(1,0,1),(1,1,1),(0,1,1)]
    # An outward-wound cube with its two top triangles omitted.
    sides = [[0,2,1],[0,3,2],[0,1,5],[0,5,4],[1,2,6],
             [1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]

    def test_missing_native_planar_surface_can_be_triangulated(self):
        from cadgen import build123d as bd
        shape = bd.Box(1,1,1,align=(bd.Align.MIN,)*3)
        triangles = [list(t) for t in self.sides]
        receipts = complete_planar_boundaries(self.vertices, triangles, shape)
        self.assertEqual(len(triangles), 12)
        self.assertEqual(len(receipts), 1)
        self.assertAlmostEqual(receipts[0]["area_mm2"], 1)

    def test_real_native_opening_is_not_filled(self):
        from cadgen import build123d as bd
        shape = bd.Box(1,1,1,align=(bd.Align.MIN,)*3)
        hole = bd.Cylinder(.25,3).moved(bd.Location((.5,.5,.5)))
        shape = shape.cut(hole)
        with self.assertRaisesRegex(ValueError, "does not match"):
            complete_planar_boundaries(self.vertices, [list(t) for t in self.sides], shape)


if __name__ == "__main__":
    unittest.main()


class MeshWorker(unittest.TestCase):
    """Run the real worker process on a small committed part."""

    def test_worker_exits_cleanly_and_reproduces_committed_mesh(self):
        import json, os, subprocess, sys, tempfile
        from pathlib import Path
        from tools.print_mesh import ROOT, inspect_3mf
        manifest = json.loads((ROOT/"printable/manifest.json").read_text())
        part = next(p for p in manifest["parts"] if p["model"] == "ankle_bearing_inner_cap_left_native")
        keys = ("model", "filename", "occurrences", "translation_mm", "expected_bounds_mm",
                "source_volume_mm3", "saved_step_comparison")
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder/"bundle/3mf").mkdir(parents=True)
            task = folder/"task.json"
            task.write_text(json.dumps({k:part[k] for k in keys}))
            env = dict(os.environ, PYTHONPATH=f"{ROOT/'src'}{os.pathsep}{ROOT}")
            algorithm, tolerance = part["mesh_recipe"]
            result = subprocess.run([sys.executable, "-m", "tools.print_mesh", "--worker", str(task),
                str(folder/"bundle"), algorithm, str(tolerance)], cwd=ROOT, env=env,
                capture_output=True, text=True, timeout=300)
            self.assertEqual(result.returncode, 0, result.stderr[-1200:])
            self.assertIn("Exported ankle_bearing_inner_cap_left_native", result.stdout)
            measured = inspect_3mf(folder/"bundle"/part["filename"], allow_numerical_overlaps=True)
            self.assertEqual(measured["triangles"], part["mesh"]["triangles"])
            self.assertEqual(measured["open_edges"], 0)
            self.assertAlmostEqual(measured["volume_mm3"], part["mesh"]["volume_mm3"], places=6)

    def test_module_refuses_direct_use(self):
        import subprocess, sys
        from tools.print_mesh import ROOT
        result = subprocess.run([sys.executable, "-m", "tools.print_mesh"], cwd=ROOT,
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
