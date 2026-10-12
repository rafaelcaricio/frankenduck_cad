"""Resolve the face's default-volume discrepancy with exact B-rep checks."""
from pathlib import Path
import json,sys
from cadgen import read_step
import cadquery as cq
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
ROOT=Path(__file__).resolve().parents[2]
local=cq.Shape.cast(read_step(ROOT/'STEP/parts/face_part_native.step').wrapped)
reference=cq.importers.importStep(str(ROOT/'STEP/imported/reference_face_part_native.step')).val()
def accurate(s):
 p=GProp_GProps();error=BRepGProp.VolumeProperties_s(s.wrapped,p,1e-9,True,False)
 return dict(volume_mm3=p.Mass(),reported_relative_error=error)
extra,missing=local.cut(reference),reference.cut(local)
report=dict(local=accurate(local),reference=accurate(reference),extra_solids=len(extra.Solids()),missing_solids=len(missing.Solids()),extra_volume_mm3=extra.Volume(),missing_volume_mm3=missing.Volume(),tolerance_mm3=1e-5)
report['passed']=report['extra_solids']==report['missing_solids']==0 and abs(report['local']['volume_mm3']-report['reference']['volume_mm3'])<report['tolerance_mm3']
(ROOT/'reports/archive/import/face_equivalence.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
assert report['passed']
