"""Regression tests for pinned-kernel placement and print-copy fidelity."""
import sys
import tempfile
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))

class BearingRuntime(unittest.TestCase):
    def test_modified_existing_parts_keep_the_simulator_palette(self):
        from types import SimpleNamespace
        from lib.viewer_palette import apply_palette
        names=['hip_left','leg_right','ankle_left','bearing_carrier_left','bearing_6704_servo_leg']
        parts=[SimpleNamespace(label=n,color=None,children=[]) for n in names]
        apply_palette(SimpleNamespace(children=parts))
        self.assertEqual([p.color for p in parts],['#8a8a90','#8a8a90','#f96c0a','#1d1d1f','#bfc3c7'])

    def test_moved_compound_intersects_at_its_installed_position(self):
        from cadgen import build123d as bd
        from lib.bearing_revision.collision_bridge_v2 import place,volume
        source=bd.Compound(children=[bd.Box(2,2,2)])
        installed=place(source,((20,0,0),('Z',90)))
        self.assertAlmostEqual(volume(installed.intersect(bd.Box(2,2,2).translate((20,0,0)))),8)
        self.assertAlmostEqual(volume(installed.intersect(bd.Box(2,2,2))),0)

    def test_raw_common_preserves_real_overlap(self):
        from cadgen import build123d as bd
        from lib.bearing_revision.hip_cradle_thread_forming_v8 import common
        self.assertAlmostEqual(common(bd.Box(2,2,2),bd.Box(2,2,2)).volume,8)

    def test_surface_seam_requires_native_surface_and_small_opening(self):
        from cadgen import build123d as bd
        from tools.print_export import complete_small_surface_seams
        shape=bd.Box(4,4,2)
        vertices=[(0,0,1),(.2,0,1),(0,.2,1)]
        triangles=[[0,1,2]]
        receipt=complete_small_surface_seams(vertices,triangles,shape)
        self.assertEqual(len(triangles),2)
        self.assertLess(receipt[0]['maximum_sampled_surface_distance_mm'],1e-6)
        with self.assertRaisesRegex(ValueError,'deviation'):
            complete_small_surface_seams([(x,y,z+.1) for x,y,z in vertices],[[0,1,2]],shape)
        with self.assertRaisesRegex(ValueError,'size'):
            complete_small_surface_seams([(-1,-1,1),(1,-1,1),(1,1,1)],[[0,1,2]],shape)

    def test_stl_copy_keeps_closed_mesh_and_dimensions(self):
        from cadgen import build123d as bd
        from tools.print_export import write_export_3mf,inspect_3mf,write_verified_stl
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'part.3mf'
            write_export_3mf(bd.Box(2,3,4),source)
            result=write_verified_stl(source,root/'part.stl',inspect_3mf(source))
            self.assertEqual(result['open_edges'],0)
            self.assertEqual(result['inconsistent_winding_edges'],0)
            self.assertAlmostEqual(result['volume_mm3'],24)
            self.assertEqual(result['bounds_mm'],[-1,-1.5,-2,1,1.5,2])

if __name__=='__main__':unittest.main()
