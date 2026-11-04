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
def shared_adapter():
 i=instances()
 seats=[place(yaw_adapter_shape(side),i[n]['at']) for side,n in [('left','servo_yaw2roll'),('right','servo_bearing_roll')]]
 bridge=box(-12,12,-20,20,120.75,124.75)
 bridge=bd.fillet(bridge.edges().filter_by(bd.Axis.Z),4)
 bottom=[e for e in bridge.edges() if abs(e.center().Z-120.75)<1e-5]
 bridge=bd.fillet(bottom,1.0)
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
 s.label='Shared hip yaw adapter - two stationary bearing seats';s.color='#f39632'
 return s

def yaw_bracket(side):
 i=instances();n='servo_yaw2roll' if side=='left' else 'servo_bearing_roll'
 src=imported('yaw2roll_left',i,'#62b6a7') if side=='left' else right_import('yaw2roll_right')
 src=as_solid(place(src,i[n]['at'],inverse=True))
 faces=[f for f in src.faces() if f.geom_type==bd.GeomType.PLANE and abs(f.center().Z+4)<1e-5 and f.normal_at().Z<-.99]
 assert len(faces)==1
 fill=bd.extrude(faces[0],amount=2.2,dir=(0,0,-1))
 s=seat_relief(src.fuse(fill,tol=1e-6)).clean()
 # Level the whole top around the horn land; do not leave tall corner fins
 # outside the circular relief on the original rectangular plate.
 s=(s-(box(-40,40,-40,40,-3.7,1)-cyl(9.7,-4,2))).clean()
 assert s.is_valid and len(s.solids())==1
 samples=[]
 for x,y in ((11,0),(-11,0),(0,11),(0,-11),(8,8),(-8,8)):
  samples.append(volume(s.intersect(cyl(.05,-7,-3.699).translate((x,y,0))))/(3.141592653589793*.05**2))
 assert min(samples)>2.49,samples
 s=place(s,i[n]['at']);s.label=side.title()+' yaw bracket - 2.5 mm bearing web';s.color='#62b6a7'
 return s

@model
def left_yaw():return yaw_bracket('left')
@model
def right_yaw():return yaw_bracket('right')

@model
def installed():
 i=instances();shared=shared_adapter()
 hips=[left_hip_dual(),right_hip_dual()];links=[left_yaw(),right_yaw()]
 hw=[p for p in hardware(i) if p.label not in ('servo_yaw2roll / PRINTED front adapter','servo_bearing_roll / PRINTED front adapter')]
 hits=[]
 for p in hips+links:
  for q in [shared,*hw]:
   if 'horn' in q.label:continue
   v=volume(p.intersect(q))
   if v>1e-4:
    h=p.intersect(q);bb=bd.Compound(children=list(h.solids())).bounding_box()
    side='left' if p.label.startswith('Left') else 'right'
    old=imported('yaw2roll_left',i,'#62b6a7') if side=='left' else right_import('yaw2roll_right')
    hits.append([p.label,q.label,v,{'bounds':[list(bb.min),list(bb.max)],'original_overlap':volume(old.intersect(q))}])
 existing=[];introduced=[]
 for h in hits:
  if 'yaw bracket' in h[0] and 'rear casing' in h[1] and abs(h[2]-h[3]['original_overlap'])<1e-5:
   existing.append(h)
  else:introduced.append(h)
 print('SHARED_INSTALLED',json.dumps({'introduced_hits':introduced,'existing_reference_overlaps':existing}),flush=True)
 assert not introduced,introduced
 return bd.Compound(children=[shared,*hips,*links,*hw],label='Shared yaw adapter installed - independent roll assemblies')

