"""Shared stationary hip-yaw bearing adapter. Candidate; mm. Servo axes unchanged."""
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model
from .hip_servos_bearing_revision import (instances,imported,right_import,hardware,left_hip_dual,right_hip_dual,yaw_adapter_shape,as_solid,seat_relief)
from .adapter_bridge_v3 import cyl,world_parts,bearing_envelope
from .collision_bridge_v2 import place,volume

def box(x0,x1,y0,y1,z0,z1):
 return bd.Box(x1-x0,y1-y0,z1-z0,align=(bd.Align.MIN,)*3).translate((x0,y0,z0))

@model
def blended_adapter():
 i=instances()
 seats=[place(yaw_adapter_shape(side),i[n]['at']) for side,n in [('left','servo_yaw2roll'),('right','servo_bearing_roll')]]
 # Rounded shoulders follow the annotated front and rear flares in plan.
 z=120.75
 front_l=bd.Bezier((-23.625,-23.75,z),(-15,-23.75,z),(-12,-20,z),(0,-20,z))
 front_r=bd.Bezier((0,-20,z),(12,-20,z),(15,-23.75,z),(23.625,-23.75,z))
 rear_r=bd.Bezier((23.625,24.9,z),(15,24.9,z),(12,20,z),(0,20,z))
 rear_l=bd.Bezier((0,20,z),(-12,20,z),(-15,24.9,z),(-23.625,24.9,z))
 outline=bd.Wire([front_l,front_r,bd.Edge.make_line((23.625,-23.75,z),(23.625,24.9,z)),rear_r,rear_l,bd.Edge.make_line((-23.625,24.9,z),(-23.625,-23.75,z))])
 bridge=bd.extrude(bd.Face(outline),amount=4)
 cases=[p for p in hardware(i) if 'PRINTED' not in p.label and ' / 6704 bearing' not in p.label and 'horn' not in p.label]
 # New bridge alone gets conservative 0.2 mm casing-envelope clearance.
 for p in cases:
  b=p.bounding_box()
  bridge=bridge-box(b.min.X-.2,b.max.X+.2,b.min.Y-.2,b.max.Y+.2,b.min.Z-.2,b.max.Z+.2)
 s=seats[0].fuse(seats[1],*bridge.solids(),tol=1e-6).clean()
 assert s.is_valid and len(s.solids())==1
 for seat in seats:assert volume(seat-s)<1e-4
 for n in ('servo_yaw2roll','servo_bearing_roll'):
  for x in (-10.25,10.25):
   access=place(cyl(2,-10,.5).translate((x,-29,0)),i[n]['at'])
   bore=place(cyl(1.1,-10,3.21).translate((x,-29,0)),i[n]['at'])
   s=s-access-bore
   assert volume(s.intersect(access))<1e-5
  assert volume(s.intersect(place(bearing_envelope(),i[n]['at'])))<1e-5
 obstacles=[*cases,*[imported(n,i,'#74988c') for n in ('trunk_base','left_shell_native','right_shell_native')]]
 hits=[]
 for p in obstacles:
  v=volume(s.intersect(p))
  if v>1e-4:hits.append([p.label,v])
 print('SHARED_ADAPTER',json.dumps({'solids':len(s.solids()),'bounds':list(s.bounding_box().size),'hits':hits,'bridge_volume':volume(bridge)}),flush=True)
 assert not hits,hits
 # A continuous 3 mm-thick bridge must cross the central span.
 for x in (-7,0,7):
  g=box(x-.01,x+.01,-14,-4,120.75,123.75)
  assert volume(g-s)<1e-5,(x,volume(g-s))
 s.label='Shared hip yaw adapter - flared connections V2';s.color='#f39632'
 return s


from .hip_rear_pin_closure_v1 import left,right
@model
def installed():
 i=instances();hw=[p for p in hardware(i) if p.label not in ('servo_yaw2roll / PRINTED front adapter','servo_bearing_roll / PRINTED front adapter')]
 return bd.Compound(children=[blended_adapter(),left_hip_dual(),right_hip_dual(),left(),right(),*hw],label='Hip assembly - flared bridge and closed rear pin supports')
