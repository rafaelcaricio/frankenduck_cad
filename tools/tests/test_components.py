"""Classification and actual 3MF integrity regressions, without CAD builds."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from tools.components import summary, validate
from tools.print_export import inspect_3mf


class ComponentCatalog(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root/"src/parts").mkdir(parents=True)
        (self.root/"source").mkdir()
        for name in ("plate", "board", "plate_unused"):
            (self.root/"src/parts"/(name+".py")).write_text("# fixture")
        def entry(name, role, active):
            return dict(model=name,component="plate" if name=="plate_unused" else name,
                params={}, role=role,geometry_fidelity="design_geometry" if role=="printed_part" else "imported_reference",
                print_release="unreviewed" if role=="printed_part" else "not_applicable",
                material_hint="pla" if role=="printed_part" else "not_applicable",export=role=="printed_part" and active,
                active_variant=active,notes="fixture intent")
        self.catalog=dict(schema_version=1,units="mm",models=[entry("plate","printed_part",True),
            entry("board","purchased_component",True),entry("plate_unused","printed_part",False)],
            review_geometry=[dict(id="cable",role="review_geometry",provider="fixture",notes="empty space")])
        self.write("components.json",self.catalog)
        self.write("variants.json",[dict(model="plate",component="plate",params={}),dict(model="board",component="board",params={})])
        self.write("assembly_data.json",dict(instances=[dict(name="plate_a",component="plate",params={}),
            dict(name="plate_b",component="plate",params={}),dict(name="board",component="board",params={})]))

    def write(self,name,value):
        (self.root/"source"/name).write_text(json.dumps(value))

    def tearDown(self):
        self.temp.cleanup()

    def test_quantities_and_unused_variant_selection(self):
        report=summary(self.root)
        self.assertEqual(report["print_model_count"],1)
        self.assertEqual(report["occurrence_roles"]["printed_part"],2)
        self.assertEqual(report["occurrence_roles"]["purchased_component"],1)
        self.assertEqual(report["models"][2]["quantity"],0)

    def test_new_model_requires_classification(self):
        (self.root/"src/parts/new.py").write_text("# new")
        with self.assertRaisesRegex(ValueError,"Unclassified"):
            validate(self.root)

    def test_purchased_hardware_cannot_export(self):
        self.catalog["models"][1]["export"]=True
        self.write("components.json",self.catalog)
        with self.assertRaisesRegex(ValueError,"Non-print"):
            validate(self.root)

    def test_unused_variant_cannot_export(self):
        self.catalog["models"][2]["export"]=True
        self.write("components.json",self.catalog)
        with self.assertRaisesRegex(ValueError,"Active printed"):
            validate(self.root)

    def test_print_release_is_separate_from_role(self):
        self.catalog["models"][0]["print_release"]="not_applicable"
        self.write("components.json",self.catalog)
        with self.assertRaisesRegex(ValueError,"print-release"):
            validate(self.root)

    def test_review_geometry_cannot_pose_as_a_print_part(self):
        self.catalog["review_geometry"][0]["role"]="printed_part"
        self.write("components.json",self.catalog)
        with self.assertRaisesRegex(ValueError,"review-geometry"):
            validate(self.root)

    def test_model_paths_cannot_escape_catalog(self):
        self.catalog["models"][0]["model"]="../plate"
        self.write("components.json",self.catalog)
        with self.assertRaisesRegex(ValueError,"Invalid or duplicate"):
            validate(self.root)


class PrintPackage(unittest.TestCase):
    def test_balanced_rounding_overlap_warns_without_blocking_export(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"part.3mf"
            faces=[(0,2,1),(0,1,3),(0,3,2),(1,2,3),(0,2,1),(0,1,2)]
            self.package(path,faces=faces)
            with self.assertRaisesRegex(ValueError,"Invalid closed mesh"):
                inspect_3mf(path)
            measured=inspect_3mf(path,allow_numerical_overlaps=True)
            self.assertEqual(measured["open_edges"],0)
            self.assertEqual(measured["nonmanifold_edges"],3)
            self.assertTrue(measured["warnings"])
            self.assertAlmostEqual(measured["volume_mm3"],1/6)

    def test_rounding_policy_still_rejects_open_boundaries(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"part.3mf"
            self.package(path,faces=[(0,2,1),(0,1,3),(0,3,2)])
            with self.assertRaisesRegex(ValueError,"Invalid closed mesh"):
                inspect_3mf(path,allow_numerical_overlaps=True)

    def package(self, path, unit="millimeter", faces=None, transform=""):
        if faces is None:faces=[(0,2,1),(0,1,3),(0,3,2),(1,2,3)]
        triangles="".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a,b,c in faces)
        vertices="".join(f'<vertex x="{x}" y="{y}" z="{z}"/>' for x,y,z in [(0,0,0),(1,0,0),(0,1,0),(0,0,1)])
        xml=f'<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" unit="{unit}"><resources><object id="1" type="model"><mesh><vertices>{vertices}</vertices><triangles>{triangles}</triangles></mesh></object></resources><build><item objectid="1" transform="{transform}"/></build></model>'
        with zipfile.ZipFile(path,"w") as archive:archive.writestr("3D/3dmodel.model",xml)

    def test_closed_positive_mesh_with_build_transform(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"part.3mf"
            self.package(path,transform="1 0 0 0 1 0 0 0 1 2 3 4")
            measured=inspect_3mf(path)
            self.assertAlmostEqual(measured["volume_mm3"],1/6)
            self.assertEqual(measured["bounds_mm"],[2,3,4,3,4,5])

    def test_wrong_units_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"part.3mf";self.package(path,unit="inch")
            with self.assertRaisesRegex(ValueError,"millimeters"):inspect_3mf(path)

    def test_open_mesh_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"part.3mf";self.package(path,faces=[(0,2,1),(0,1,3),(0,3,2)])
            with self.assertRaisesRegex(ValueError,"closed mesh"):inspect_3mf(path)

    def test_reversed_winding_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"part.3mf";self.package(path,faces=[(0,1,2),(0,1,3),(0,3,2),(1,2,3)])
            with self.assertRaisesRegex(ValueError,"closed mesh"):inspect_3mf(path)
