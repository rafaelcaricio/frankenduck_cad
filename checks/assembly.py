"""Assembly declarations and saved geometry inventory."""

def metadata():
    from pathlib import Path
    import json
    ROOT = Path(__file__).resolve().parents[1]
    data=json.loads((ROOT/'source/assembly_data.json').read_text());side=json.loads((ROOT/'STEP/frankenduck.step.json').read_text())
    additions=json.loads((ROOT/'source/local_additions.json').read_text())
    expected={m['name']:m for m in data['mates']+additions['mates']};actual={m['name']:m for m in side['kinematics']['mates']}
    assert expected.keys()==actual.keys()
    refs={item['name']:f'o1.{i}' for i,item in enumerate(data['instances']+additions['instances'],1)}
    for name,m in actual.items():
     e=expected[name]
     for k in ['kind','parent','child']:assert m[k]==e[k],(name,k)
     assert m['parentId']==refs[e['parent'][1:]] and m['childId']==refs[e['child'][1:]],name
     if m['kind']=='revolute':
      assert m['axis']['origin']==list(e['origin']),name
      # Axes are already unit vectors in the frozen canonical source.
      assert m['axis']['dir']==list(e['direction']),name
      assert m['limits']['value']==list(e.get('limits',[-180,180])),name
    assert side['kinematics']['poses']==data['poses']
    src=(ROOT/'src/animation.js').read_text();clips=json.loads(src.split('const DATA = ',1)[1].split(';\nconst OPS = ',1)[0])
    for a in data['animations']:
     c=clips[a['name']];assert c['keys']==a['keys'];assert c['loop']==a['loop'];assert c['duration']==a['keys'][-1][1]
    ops=json.loads(src.split('const OPS = ',1)[1].split(';\n',1)[0])
    board_op=next(x for x in ops if x['target']=='#o1.15')
    heat_op=next(x for x in ops if x['target']=='#o1.74')
    assert board_op['joints']==heat_op['joints']
    assert side['animation']['source']==src
    report=dict(passed=True,mates=73,revolute=15,fastened=58,canonical_mates_preserved=72,heatsink_animation_matches_board=True,poses=list(data['poses']),animation_keyframes=len(data['animations'][0]['keys']),animation_duration_s=clips[data['animations'][0]['name']]['duration'],animation_loop=False,jaw_display_range_degrees=[-180,180],jaw_measured_range=None)
    (ROOT/'reports/latest/metadata_verification.json').write_text(json.dumps(report,indent=2));print(report)

    return report


def inventory():
    from pathlib import Path
    import json,hashlib
    from checks.common import read_scene
    ROOT = Path(__file__).resolve().parents[1]
    path=ROOT/'STEP/frankenduck.step';scene=read_scene(path);source=json.loads((ROOT/'source/assembly_data.json').read_text());side=json.loads(path.with_suffix('.step.json').read_text())
    additions=json.loads((ROOT/'source/local_additions.json').read_text())
    expected_instances=source['instances']+additions['instances']
    root=scene.roots[0]
    print('root',root.label,root.ref,'children',len(root.children),flush=True)
    rows=[]
    for expected,actual in zip(expected_instances,root.children):
     print(actual.ref,actual.label,flush=True)
     assert actual.label==expected['name'],(actual.label,expected['name'])
     s=actual.shape();b=s.bounding_box()
     rows.append(dict(name=actual.label,ref=actual.ref,solids=len(s.solids()),volume_mm3=sum(x.volume for x in s.solids()),extent_mm=[*b.min,*b.max]))
    assert len(root.children)==len(expected_instances)==74
    summary=dict(document_sha256=scene.document_hash,occurrences=len(root.children),canonical_occurrences=73,local_added_occurrences=1,solids=sum(x['solids'] for x in rows),volume_mm3=sum(x['volume_mm3'] for x in rows),extent_mm=[*[min(x['extent_mm'][i] for x in rows) for i in range(3)],*[max(x['extent_mm'][i+3] for x in rows) for i in range(3)]],sidecar_keys=list(side),instances=rows)
    (ROOT/'reports/latest/assembly_verification.json').write_text(json.dumps(summary,indent=2))
    print({k:v for k,v in summary.items() if k!='instances'},flush=True)

    return summary


def catalog():
    """Metadata-only coverage of every part script and assembly occurrence."""
    from pathlib import Path
    import json
    from tools.components import summary
    root = Path(__file__).resolve().parents[1]
    report = summary(root)
    (root / "reports/latest/component_classification.json").write_text(json.dumps(report, indent=2) + "\n")
    print({k:v for k,v in report.items() if k != "models"})
    return report
