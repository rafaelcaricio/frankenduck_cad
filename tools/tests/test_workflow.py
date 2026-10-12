"""Regression coverage for stale/cached/failed receipts, with no CAD builds."""
from contextlib import redirect_stdout, redirect_stderr
from io import StringIO
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("cad_workflow", Path(__file__).resolve().parents[1] / "cad.py")
workflow = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workflow)


class WorkflowReceipts(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.patches = [patch.object(workflow, "ROOT", self.root),
                        patch.object(workflow, "LATEST", self.root / "reports/latest")]
        for p in self.patches:
            p.start()
        fixtures = {"src/model.py": "# model\n", "src/animation.js": "// animation\n",
                    "source/assembly.json": "{}", "STEP/imported/board.step": "input",
                    "requirements.txt": "cadgen", "build.sh": "# build\n",
                    "STEP/frankenduck.step": "saved geometry",
                    "STEP/frankenduck.step.json": "{}", "checks/common.py": "# helpers\n",
                    "checks/__init__.py": "", "tools/cad.py": "# runner\n",
                    "fixture.txt": "value A"}
        for name, content in fixtures.items():
            p = self.root / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(content)
        self.entry = dict(id="fixture", call="workflow_fixture:run", purpose="fixture check",
                          suites=["fast", "full"], inputs=["fixture.txt"],
                          report="reports/latest/fixture.json", coverage="temporary fixture")
        workflow.write_json(self.root / "checks/registry.json", dict(checks=[self.entry]))
        workflow.write_json(workflow.LATEST / "build.json", dict(status="PASS",
            input_key=workflow.signature(workflow.BUILD_INPUTS)["key"],
            output_key=workflow.signature(workflow.BUILD_OUTPUTS)["key"]))
        self.calls = 0
        module = types.ModuleType("workflow_fixture")
        module.run = self.success
        self.module = module
        self.module_patch = patch.dict(sys.modules, {"workflow_fixture": module})
        self.module_patch.start()

    def tearDown(self):
        self.module_patch.stop()
        for p in reversed(self.patches):
            p.stop()
        self.temp.cleanup()

    def success(self):
        self.calls += 1
        report = dict(passed=True)
        workflow.write_json(self.root / self.entry["report"], report)
        return report

    def run_suite(self):
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            return workflow.run_checks("fast")

    def test_unchanged_pass_is_reused(self):
        self.assertEqual(self.run_suite(), 0)
        self.assertEqual(self.run_suite(), 0)
        self.assertEqual(self.calls, 1)

    def test_content_change_with_same_mtime_invalidates_cache(self):
        self.run_suite()
        path = self.root / "fixture.txt"
        stamp = path.stat()
        path.write_text("value B")
        os.utime(path, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
        self.assertEqual(workflow.check_status(self.entry, "PASS")[0], "STALE")
        self.assertEqual(self.run_suite(), 0)
        self.assertEqual(self.calls, 2)

    def test_report_tampering_invalidates_cache(self):
        self.run_suite()
        (self.root / self.entry["report"]).write_text('{"passed": false}')
        self.assertEqual(workflow.check_status(self.entry, "PASS")[0], "STALE")

    def test_new_model_file_invalidates_build(self):
        (self.root / "src/new_part.py").write_text("# new model")
        self.assertEqual(workflow.build_status()[0], "STALE")
        self.assertEqual(self.run_suite(), 1)
        self.assertEqual(self.calls, 0)

    def test_missing_input_does_not_pass(self):
        self.run_suite()
        (self.root / "fixture.txt").unlink()
        self.assertEqual(self.run_suite(), 1)
        self.assertEqual(workflow.check_status(self.entry, "PASS")[0], "STALE")

    def test_exception_cannot_reuse_old_passing_report(self):
        self.run_suite()
        (self.root / "fixture.txt").write_text("new input")
        def failure():
            raise RuntimeError("geometry computation failed")
        self.module.run = failure
        self.assertEqual(self.run_suite(), 1)
        self.assertEqual(workflow.check_status(self.entry, "PASS")[0], "FAILED")

    def test_inputs_changing_during_check_invalidate_result(self):
        def changing():
            report = self.success()
            (self.root / "fixture.txt").write_text("changed while running")
            return report
        self.module.run = changing
        self.assertEqual(self.run_suite(), 1)
        self.assertNotEqual(workflow.check_status(self.entry, "PASS")[0], "PASS")

    def test_check_implementation_change_invalidates_cache(self):
        self.run_suite()
        (self.root / "checks/common.py").write_text("# changed intersection")
        self.assertEqual(workflow.check_status(self.entry, "PASS")[0], "STALE")

    def test_interrupted_run_is_visible(self):
        self.run_suite()
        path = workflow.LATEST / "fixture.receipt.json"
        receipt = workflow.read_json(path)
        receipt["status"] = "RUNNING"
        workflow.write_json(path, receipt)
        self.assertEqual(workflow.check_status(self.entry, "PASS")[0], "RUNNING")

    def test_status_refresh_does_not_break_provenance_file_hashes(self):
        self.run_suite()
        with redirect_stdout(StringIO()):
            workflow.manifest()
        receipt = workflow.read_json(workflow.LATEST / "provenance_manifest.json")
        self.assertNotIn("reports/latest/summary.json", receipt["files"])
        with redirect_stdout(StringIO()):
            workflow.status()
        for name, info in receipt["files"].items():
            self.assertEqual(workflow.digest(self.root / name), info["sha256"])


if __name__ == "__main__":
    unittest.main()
