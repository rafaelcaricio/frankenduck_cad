"""Independent local reconstruction and comparison with pinned STEVE measurements."""
from pathlib import Path
import importlib, json, time, traceback, sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
variants=json.loads((ROOT/'source/variants.json').read_text())
baselines=json.loads((ROOT/'source/port_audit/steve_baselines.json').read_text())
report=[]
for v in variants:
    name=v['model'];t=time.monotonic()
    try:
        obj=importlib.import_module('parts.'+name).make()
        box=obj.bounding_box();volume=sum(s.volume for s in obj.solids())
        metrics=dict(volume_mm3=volume,extent_mm=[*box.min,*box.max],solids=len(obj.solids()),faces=len(obj.faces()),valid=obj.is_valid)
        base=baselines.get(v['component'],{}).get('model',{})
        expected=base.get('metadata',{})
        comparison=None
        if expected and not base.get('stale') and not v['params'].get('seat_x'):
            target=expected['volume_mm3'];delta=abs(volume-target)
            comparison=dict(expected_volume_mm3=target,delta_mm3=delta,relative_error=delta/max(1,abs(target)),pass_volume=delta<max(0.02,abs(target)*1e-6))
        row=dict(**v,status='ok',seconds=time.monotonic()-t,metrics=metrics,comparison=comparison,baseline_stale=bool(base.get('stale')))
    except Exception as e:
        row=dict(**v,status='failed',seconds=time.monotonic()-t,error=str(e),traceback=traceback.format_exc())
    report.append(row);print(name,row['status'],round(row['seconds'],2),row.get('comparison'),row.get('error',''),flush=True)
    (ROOT/'reports/archive/import/geometry_audit.json').write_text(json.dumps(report,indent=2))
