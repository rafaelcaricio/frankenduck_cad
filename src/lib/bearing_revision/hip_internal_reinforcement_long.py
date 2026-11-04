"""Continuous internal hip-rib reinforcement. Original exterior and mounting datums.
Candidate; geometry and sampled collision checks are not strength validation.
"""
import json,math
from cadgen import build123d as bd
from .runtime import cached_shape as model
from .hip_bearing_revision import build_hip as relieved_hip
from .upper_leg_carrier_v10 import instances,imported,inner_carrier
from .right_leg_bearing_revision import right_import,right_carrier,both_legs
from .adapter_bridge_v3 import world_parts,servo_adapter_review,bearing_envelope,cyl
from .collision_bridge_v2 import place,volume

def interior_bulge():
 # Central 12 mm band, clear of all original screw counterbores.
 # Elliptical sections merge into the existing internal surface at both ends.
 profiles=[]
 for y,depth in ((-12,.05),(-11,1.2),(-9,2),(-5,2),(0,2),(7,2),(12,2),(17.5,2),(19.5,1.8),(21,1.4),(22.5,.5),(23.5,.05)):
  base=-6.075 if y<=17.55 else -14.175+math.sqrt(8.1**2-(y-17.55)**2)
  section=bd.Plane(origin=(0,y,base+.10),x_dir=(1,0,0),z_dir=(0,1,0))*bd.Ellipse(6,depth)
  profiles.append(section)
 rib=bd.loft(profiles)
 # Keep the round endpoint within the existing mounting plate outline.
 positive=bd.Box(40,80,23.975,align=(bd.Align.CENTER,bd.Align.MIN,bd.Align.MIN)).translate((0,0,-30))
 footprint=cyl(12.6,-30,-6.025).fuse(positive)
 rib=rib.intersect(footprint)
 for x,y in ((7,0),(-7,0),(0,7),(0,-7)):
  rib=bd.Compound(children=list(rib)) if isinstance(rib,bd.ShapeList) else rib
  rib=rib-cyl(3.45,-20,2).translate((x,y,0))
 return rib

@model
def left_hip():return build('left')
@model
def right_hip():return build('right')

def build(side):
 i=instances();name='servo_upper_leg_'+side
 baseline=relieved_hip(side)
 bulge=interior_bulge()
 s=baseline.fuse(*(list(bulge) if isinstance(bulge,bd.ShapeList) else [bulge]),tol=1e-6).clean()
 assert s.is_valid and len(s.solids())==1
 # Only the central interior band may change. The original exterior bounding
 # box, bearing-side face, both mounting plates and screw openings are protected.
 add=s-baseline
 assert 10<volume(add)<600,volume(add)
 bb=add.bounding_box()
 assert bb.min.X>=-6.01 and bb.max.X<=6.01
 assert bb.min.Y>=-12.01 and bb.max.Y<=23.51
 assert bb.max.Z<-5.9
 for x,y in ((7,0),(-7,0),(0,7),(0,-7)):
  assert volume(add.intersect(cyl(3.4,-20,2).translate((x,y,0))))<1e-5,(x,y,volume(add.intersect(cyl(3.4,-20,2).translate((x,y,0)))))
 for a,b in zip(list(s.bounding_box().min)+list(s.bounding_box().max),list(baseline.bounding_box().min)+list(baseline.bounding_box().max)):
  assert abs(a-b)<1e-4,(a,b)
 assert volume(s.intersect(bearing_envelope()))<1e-5
 # Measure thickness along the new load path, across the relieved neck.
 measurements=[]
 for x,y in ((0,-11),(0,0),(5,7),(-5,7),(0,12.5),(0,14),(0,17.5),(0,21)):
  g=cyl(.05,-12,-3.699).translate((x,y,0))
  measurements.append({'x':x,'y':y,'old_mm':volume(baseline.intersect(g))/(math.pi*.05**2),'new_mm':volume(s.intersect(g))/(math.pi*.05**2)})
 # Only added material is checked against roll geometry; existing baseline
 # interferences at the CAD end stops are not silently reclassified as new ones.
 bodyname='servo_hip_l' if side=='left' else 'servo_hip_l_2'
 obstacles=[]
 for p in world_parts(servo_adapter_review()):
  if 'PRINTED' in p.label or 'horn' in p.label or '从动' in p.label:continue
  obstacles.append(place(p.rotate(bd.Axis.X,180),i[bodyname]['at']))
 obstacles.append(imported('yaw2roll_left',i,'#888888') if side=='left' else right_import('yaw2roll_right'))
 roll_axis=bd.Axis(i[bodyname]['at'][0],(0,-1,0))
 world_add=place(add,i[name]['at'])
 hits=[]
 for angle in range(-31,32):
  posed=world_add.rotate(roll_axis,angle)
  for p in obstacles:
   v=volume(posed.intersect(p))
   if v>1e-4:hits.append([angle,p.label,v])
 assert not hits,hits
 carrier=inner_carrier() if side=='left' else right_carrier()
 pitch_axis=bd.Axis(i[name]['at'][0],(1,0,0))
 for angle in range(-90,91,5):
  assert volume(world_add.intersect(carrier.rotate(pitch_axis,angle)))<1e-4,(side,angle)
 print('INTERNAL_RIB',json.dumps({'side':side,'added_mm3':volume(add),'added_bounds':[list(bb.min),list(bb.max)],'sections':measurements,'roll_samples':63,'pitch_samples':37,'new_collisions':0,'outer_bounds_unchanged':True}))
 s.label=side.title()+' hip - continuous internal rib, original exterior';s.color='#72b6ad'
 return s

@model
def complete_hips_and_legs():
 i=instances();parts=[both_legs()]
 for side,fn in (('left',left_hip),('right',right_hip)):
  parts.append(place(fn(),i['servo_upper_leg_'+side]['at']))
 return bd.Compound(children=parts,label='Both legs - internal hip reinforcement, original exterior')

