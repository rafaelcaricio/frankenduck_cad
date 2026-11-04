"""Candidate: close unused rear horn openings, preserving roll servo axes. mm."""
import sys
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model
from .shared_hip_yaw_adapter import yaw_bracket,shared_adapter,instances,hardware,place,volume,left_hip_dual,right_hip_dual,cyl

def closed_bracket(side):
 i=instances();n='servo_hip_l' if side=='left' else 'servo_hip_l_2'
 old=yaw_bracket(side);local=place(old,i[n]['at'],inverse=True)
 bores=[f for f in local.faces() if f.geom_type==bd.GeomType.CYLINDER and abs(f.radius-9.65)<1e-5 and abs(f.bounding_box().min.Z-35.2)<1e-5]
 assert len(bores)==1
 z0=bores[0].bounding_box().min.Z;z1=bores[0].bounding_box().max.Z
 fill=cyl(9.85,z0,z1)-cyl(3.3,z0-1,z1+1)
 s=local.fuse(fill,tol=1e-6).clean()
 assert s.is_valid and len(s.solids())==1
 assert volume(local-s)<1e-5
 assert volume(s.intersect(cyl(3.3,z0-1,z1+1)))<1e-5
 for x,y in ((5,0),(-5,0),(0,5),(0,-5)):
  gauge=cyl(.1,z0,z1).translate((x,y,0));assert volume(gauge-s)<1e-6
 s=place(s,i[n]['at']);added=s-old
 hits=[]
 obstacles=hardware(i)+[left_hip_dual(),right_hip_dual(),shared_adapter()]
 for q in obstacles:
  v=volume(added.intersect(q))
  if v>1e-5:hits.append((q.label,v))
 assert not hits,hits
 print('REAR_PIN_CLOSURE',json.dumps({'side':side,'old_opening_diameter':19.3,'new_opening_diameter':6.6,'pin_diameter':6,'radial_clearance':.3,'plate_thickness':z1-z0,'added_volume':volume(added),'added_static_hits':hits}),flush=True)
 s.label=side.title()+' yaw bracket - closed rear pin support';s.color='#62b6a7'
 return s
@model
def left():return closed_bracket('left')
@model
def right():return closed_bracket('right')
@model
def installed():
 i=instances();hw=[p for p in hardware(i) if p.label not in ('servo_yaw2roll / PRINTED front adapter','servo_bearing_roll / PRINTED front adapter')]
 return bd.Compound(children=[shared_adapter(),left_hip_dual(),right_hip_dual(),left(),right(),*hw],label='Hip assembly - rear pin closure candidate')
