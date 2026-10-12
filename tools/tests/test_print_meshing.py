"""Regression coverage for rounding-error recovery without filling real openings."""
import unittest

from tools.print_export import complete_planar_boundaries, geometric_difference


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
