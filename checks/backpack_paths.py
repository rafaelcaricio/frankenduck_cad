"""Saved STEP checks for the compact backpack operating and service paths.

They cover the named trajectory and neutral-pose removal; no hardware commands.
"""

def motion():
    from pathlib import Path
    import json,math,itertools
    import numpy as np
    import cadquery as cq
    from checks.common import read_scene,bounds,volume
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    ROOT=Path(__file__).resolve().parents[1]
    OUT=ROOT/'reports/latest'
    from lib import backpack as bp
    scene=read_scene(ROOT/'STEP/frankenduck.step')
    occ={o.ref:o for o in scene.roots[0].children}
    sh={o.label:cq.Shape.cast(o.shape().wrapped) for o in occ.values()}
    changed=('trunk_base','backpack_dock','left_shell_native','right_shell_native')
    bb={n:bounds(q) for n,q in sh.items()}
    report={'motion':{}}
    def boundgap(a,b):return math.sqrt(sum(max(a[i]-b[i+3],b[i]-a[i+3],0)**2 for i in range(3)))
    def dist(a,b):
     d=BRepExtrema_DistShapeShape()
     d.SetMultiThread(True)
     d.LoadS1(a.wrapped);d.LoadS2(b.wrapped);d.Perform()
     assert d.IsDone();return d.Value()
    def sat_gap(moving_box,m,other_box):
        # Exact boxes enclose the B-reps; projected separation is a conservative
        # lower bound on real part clearance, including after joint rotation.
        a=np.array(moving_box);b=np.array(other_box)
        ca=m[:3,:3]@((a[:3]+a[3:])/2)+m[:3,3];ea=(a[3:]-a[:3])/2
        cb=(b[:3]+b[3:])/2;eb=(b[3:]-b[:3])/2;R=m[:3,:3]
        axes=[*R.T,*np.eye(3),*[np.cross(x,y) for x in R.T for y in np.eye(3)]]
        best=0
        for ax in axes:
            norm=np.linalg.norm(ax)
            if norm<1e-9:continue
            ax=ax/norm
            g=abs((ca-cb)@ax)-np.abs(ax@R)@ea-np.abs(ax)@eb
            best=max(best,g)
        return best
    # All 1702 recorded walk/sit keyframes, with exact B-rep distance near bounds.
    source=(ROOT/'src/animation.js').read_text();clip=json.loads(source.split('const DATA = ',1)[1].split(';\nconst OPS = ',1)[0])['walk_sit_stand_walk_sit'];ops=json.loads(source.split('const OPS = ',1)[1].split(';\n',1)[0]);legs=[o for o in ops if any(j['name'].startswith(('left_','right_')) for j in o['joints'])]
    def matrix(joints,pose):
     result=np.eye(4)
     for j in joints:
      ax=np.array(j['axis'],float);x,y,z=ax;ang=math.radians(pose.get(j['name'],0));sk=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);r=np.eye(3)*math.cos(ang)+(1-math.cos(ang))*np.outer(ax,ax)+math.sin(ang)*sk;o=np.array(j['origin']);m=np.eye(4);m[:3,:3]=r;m[:3,3]=o-r@o;result=m@result
     return result
    corners={n:np.array([(*p,1) for p in itertools.product(*[(b[i],b[i+3]) for i in range(3)])]) for n,b in bb.items()}
    mins={n:{'gap_mm':1000} for n in changed+('parkside_battery',)};bad=[];count=0;exact=0
    for k,(pose,t) in enumerate(clip['keys']):
     for op in legs:
      n=occ[op['target']].label;m=matrix(op['joints'],pose);points=(m@corners[n].T).T[:,:3];b=[*points.min(0),*points.max(0)];candidate=None
      for target in mins:
       count+=1;g=boundgap(b,bb[target])
       if g>=2:continue
       if sat_gap(bb[n],m,bb[target])>=2:continue
       if candidate is None:
        candidate=sh[n]
        for j in op['joints']:candidate=candidate.rotate(j['origin'],[u+v for u,v in zip(j['origin'],j['axis'])],pose.get(j['name'],0))
       d=dist(candidate,sh[target]);exact+=1
       if d<mins[target]['gap_mm']:mins[target]={'part':n,'time_s':t,'gap_mm':d}
       if d<1-1e-5:
        v=volume(candidate.intersect(sh[target])) if d<1e-6 else 0;r={'part':n,'target':target,'time_s':t,'gap_mm':d,'overlap_mm3':v};bad.append(r)
        if len(bad)<12:print('MOTION_FAIL',r,flush=True)
     if k%100==0:print('motion',k,len(clip['keys']),count,exact,mins,flush=True)
    certified_min=min([2.0]+[row['gap_mm'] for row in mins.values()])
    for target,row in list(mins.items()):
        if row['gap_mm']==1000:
            mins[target]={'lower_bound_mm':2.0,'method':'all pairs separated by conservative bounds'}
        else:
            row['minimum_exact_tested_gap_mm']=row.pop('gap_mm')
    report['motion']={'keyframes':len(clip['keys']),'pairs':count,'exact_distances':exact,'near_minima':mins,'proven_sampled_gap_lower_bound_mm':certified_min,'below_1mm':bad,'passed':not bad,'scope':'30 leg occurrences against battery, carrier, trunk and complete shells. Keyframes only; linear interpolation not continuous certified. Distances >=2 mm may be rejected using conservative bounds.'}
    
    report['motion']['document_sha256']=scene.document_hash
    (OUT/'backpack_motion_verification.json').write_text(json.dumps(report['motion'],indent=2));print('MOTION_RESULT',report['motion'],flush=True)
    
    result=report['motion']
    assert result['passed'],result['below_1mm']
    return result

def removal():
    from pathlib import Path
    import json,math,itertools
    import numpy as np
    import cadquery as cq
    from checks.common import read_scene,bounds,volume
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    ROOT=Path(__file__).resolve().parents[1]
    OUT=ROOT/'reports/latest'
    from lib import backpack as bp
    scene=read_scene(ROOT/'STEP/frankenduck.step')
    occ={o.ref:o for o in scene.roots[0].children}
    sh={o.label:cq.Shape.cast(o.shape().wrapped) for o in occ.values()}
    changed=('trunk_base','backpack_dock','left_shell_native','right_shell_native')
    bb={n:bounds(q) for n,q in sh.items()}
    report={'motion':{}}
    def boundgap(a,b):return math.sqrt(sum(max(a[i]-b[i+3],b[i]-a[i+3],0)**2 for i in range(3)))
    def dist(a,b):
     d=BRepExtrema_DistShapeShape()
     d.SetMultiThread(True)
     d.LoadS1(a.wrapped);d.LoadS2(b.wrapped);d.Perform()
     assert d.IsDone();return d.Value()
    # Two-stage removal: 51 mm up the rail, then 80 mm normal away from the dock.
    # Rails terminate at local Z131; rail nose begins Z81.815: 49.185 disengagement.
    a=math.radians(bp.TILT_DEG);s,c=math.sin(a),math.cos(a);path=[('slide',float(d),(0,-d*s,d*c)) for d in np.r_[np.arange(0,51,2),51]]+[('depart',float(d),(0,-51*s+d*c,51*c+d*s)) for d in np.arange(2,81,2)]
    from functools import lru_cache
    pieces={n:[(q,bounds(q)) for q in shape.Solids()] for n,shape in sh.items()}
    rows=[];exact=0
    for stage,d,delta in path:
        contacts=[]
        pb=[v+delta[i%3] for i,v in enumerate(bb['parkside_battery'])]
        for n,q in sh.items():
            if n=='parkside_battery' or boundgap(pb,bb[n])>3:continue
            minimum=3.0;bad=0
            for battery_solid,b in pieces['parkside_battery']:
                moved_box=[v+delta[i%3] for i,v in enumerate(b)]
                for other_solid,other_box in pieces[n]:
                    if boundgap(moved_box,other_box)>minimum:continue
                    moved=battery_solid.translate(delta)
                    separation=dist(moved,other_solid);exact+=1;minimum=min(minimum,separation)
                    if separation<1e-6:bad+=volume(moved.intersect(other_solid))
            row=dict(stage=stage,travel_mm=d,part=n,gap_mm=minimum,overlap_mm3=bad,passed=bad<1e-5)
            rows.append(row)
            if minimum<3:contacts.append((n,minimum,bad))
        print(stage,d,'exact',exact,'near',contacts,flush=True)
    result=dict(passed=all(r['passed'] for r in rows),exact_solid_pairs=exact,rows=rows,scope='Neutral head and legs. 51 mm rail slide sampled at 2 mm plus endpoint, then 80 mm departure normal to backing sampled at 2 mm. Gaps at least 3 mm may be reported as 3 mm lower bounds.')
    result['document_sha256']=scene.document_hash
    (OUT/'backpack_removal_verification.json').write_text(json.dumps(result,indent=2));print('REMOVAL_RESULT',result['passed'],flush=True)
    
    assert result['passed'],[r for r in rows if not r['passed']]
    return result
