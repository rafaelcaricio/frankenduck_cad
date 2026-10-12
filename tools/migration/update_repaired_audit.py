from pathlib import Path
import json, sys, importlib, time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
p=ROOT/'reports/archive/import/geometry_audit.json';rows=json.loads(p.read_text());baselines=json.loads((ROOT/'source/port_audit/steve_baselines.json').read_text())
for row in rows:
 if row['status']=='failed':
  t=time.monotonic();s=importlib.import_module('parts.'+row['model']).make();b=s.bounding_box();v=sum(x.volume for x in s.solids());target=baselines[row['component']]['model']['metadata']['volume_mm3'];delta=abs(v-target)
  row.update(status='ok',seconds=time.monotonic()-t,metrics=dict(volume_mm3=v,extent_mm=[*b.min,*b.max],solids=len(s.solids()),faces=len(s.faces()),valid=s.is_valid),comparison=dict(expected_volume_mm3=target,delta_mm3=delta,relative_error=delta/target,pass_volume=delta<max(.02,target*1e-6)))
  row.pop('error',None);row.pop('traceback',None);print(row['model'],row['comparison'],flush=True)
p.write_text(json.dumps(rows,indent=2))
