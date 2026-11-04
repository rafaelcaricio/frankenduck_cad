"""Installed knee bearing collision, complete and sectioned; review only."""
from pathlib import Path
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model, read_step
from .adapter_bridge_v2 import servo_adapter_review, bearing_envelope, world_parts, HORN_FRONT_Z

ROOT=Path(__file__).resolve().parents[3]

def flatten(shape):
    return bd.Compound(children=list(world_parts(shape)))

def place(s, at, inverse=False):
    translation,(axis,angle)=at
    axis={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}.get(axis,axis) if isinstance(axis,str) else axis
    axis=bd.Axis((0,0,0),axis)
    result = s.translate(tuple(-x for x in translation)).rotate(axis,-angle) if inverse else s.rotate(axis,angle).translate(translation)
    # build123d 0.11 compounds can intersect stale child placements after a move.
    # Extract the placed solids before Boolean operations; retain the exact BRep.
    solids = list(result.solids())
    baked = solids[0] if len(solids)==1 else bd.Compound(children=solids)
    baked.label=s.label; baked.color=s.color
    return baked

def volume(s):
    return sum(q.volume for q in s.solids()) if s else 0

def styled(shape,label,color):
    s=bd.Compound(children=list(shape.solids()),label=label)
    s.color=color
    for c in s.children:c.color=color
    return s

@model
def installed_collision():
    data=json.loads((ROOT/'source/bearing_revision_baseline.json').read_text())
    servo=next(i for i in data['instances'] if i['name']=='servo_leg')
    leg=next(i for i in data['instances'] if i['name']=='leg_left')
    source=read_step(str(ROOT/'STEP/parts'/(leg['component']+'.step')))
    existing=place(place(flatten(source),leg['at']),servo['at'],inverse=True)
    # Only this local mating link follows the new horn face. This is a seating
    # hypothesis, not a relocation of the complete robot's downstream chain.
    existing=existing.translate((0,0,HORN_FRONT_Z))
    bearing=bearing_envelope()
    parts=[p.rotate(bd.Axis.X,180) for p in world_parts(servo_adapter_review())]
    adapter=next(p for p in parts if p.label=='PRINTED front adapter')
    bearing_collision=existing.intersect(bearing)
    adapter_collision=existing.intersect(adapter)
    bearing_overlap=volume(bearing_collision)
    adapter_overlap=volume(adapter_collision)
    print('V2_CORRECTED_DATUM',json.dumps({
        'hub_seating':'vendor datum, provisional',
        'horn_front_z_mm':HORN_FRONT_Z,
        'bearing_ahead_of_horn_mm':HORN_FRONT_Z-bearing.bounding_box().min.Z,
        'bearing_leg_overlap_mm3':bearing_overlap,
        'adapter_leg_overlap_mm3':adapter_overlap}))
    parts=[p for p in parts if p is not adapter]
    parts += [
        styled(existing-bearing-adapter,'Existing left knee link','#3ca997'),
        styled(bearing-existing,'6704 bearing','#5299e3'),
        styled(adapter-existing,'V2 backing and collar','#f39632'),
    ]
    if bearing_overlap>1e-6:
        parts.append(styled(bearing_collision,'Red - bearing / link interference','#fa2238'))
    if adapter_overlap>1e-6:
        parts.append(styled(adapter_collision,'Magenta - collar / link interference','#d640ba'))
    complete=[];section=[]
    cutter=bd.Box(100,200,150,align=(bd.Align.MAX,bd.Align.CENTER,bd.Align.CENTER))
    for p in parts:
        full=p.rotate(bd.Axis.X,180)
        complete.append(full)
        cut=p.intersect(cutter)
        if cut and volume(cut)>1e-6:
            c=styled(cut,p.label,p.color).rotate(bd.Axis.X,180).translate((75,0,0))
            section.append(c)
    return bd.Compound(children=[
        bd.Compound(children=complete,label='LEFT - complete installed knee assembly'),
        bd.Compound(children=section,label='RIGHT - cutaway at shaft center; positions unchanged')],
        label='V2 with corrected 5.8 mm horn height - seating provisional')

