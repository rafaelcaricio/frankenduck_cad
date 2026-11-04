"""Hip bearing relief preserving the body-side mating datum. mm, candidates."""
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model
from .upper_leg_carrier_v10 import instances,imported,inner_carrier
from .right_leg_bearing_revision import right_import,right_carrier,both_legs
from .collision_bridge_v2 import place,volume
from .adapter_bridge_v3 import cyl,HORN_FRONT_Z,bearing_envelope,adapter_print,COLLAR_FRONT

def build_hip(side):
 i=instances();name='servo_upper_leg_'+side
 original=imported('hip_left',i,'#72b6ad') if side=='left' else right_import('hip_right')
 source=place(original,i[name]['at'],inverse=True)
 # Machine only the carrier-side mounting face; keep the remote hip mate fixed.
 relief=cyl(16,-3.7,3)-cyl(9.7,-4,4)
 face_setback=cyl(9.7,HORN_FRONT_Z,3)
 s=(source-relief-face_setback).clean()
 assert s.is_valid and len(s.solids())==1
 bearing=bearing_envelope();adapter=adapter_print().translate((0,0,COLLAR_FRONT))
 assert volume(s.intersect(bearing))<1e-5 and volume(s.intersect(adapter))<1e-5
 assert bearing.distance_to(s)>=.1999
 # The distant body-side mounting plate and all its bores are unchanged.
 protected=bd.Box(80,80,60,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MAX)).translate((0,20,-10))
 assert abs(volume(source.intersect(protected))-volume(s.intersect(protected)))<1e-5
 for x,y in ((7,0),(-7,0),(0,7),(0,-7)):
  assert volume(s.intersect(cyl(1.69,-10,2).translate((x,y,0))))<1e-5
 samples=[]
 for x,y in ((11,0),(-11,0),(0,-11),(8,8),(-8,8),(0,11)):
  samples.append(volume(s.intersect(cyl(.05,-10,-3.699).translate((x,y,0))))/(3.141592653589793*.05**2))
 assert min(samples)>2.3,samples
 world=place(s,i[name]['at'])
 carrier=inner_carrier() if side=='left' else right_carrier()
 overlap=volume(world.intersect(carrier))
 assert overlap<1e-4,overlap
 print('HIP_FIT',json.dumps({'side':side,'bearing_gap_mm':bearing.distance_to(s),'adapter_gap_mm':adapter.distance_to(s),'carrier_overlap_mm3':overlap,'sample_web_mm':samples,'remote_mount_preserved':True,'one_solid':True}))
 s.label=side.title()+' hip link - bearing relief';s.color='#72b6ad'
 return s

@model
def left_hip():return build_hip('left')
@model
def right_hip():return build_hip('right')
@model
def complete_hips_and_legs():
 i=instances()
 parts=[both_legs()]
 for side,fn in (('left',left_hip),('right',right_hip)):
  parts.append(place(fn(),i['servo_upper_leg_'+side]['at']))
 return bd.Compound(children=parts,label='Both legs with revised hip links - fixed body mounting datums')
