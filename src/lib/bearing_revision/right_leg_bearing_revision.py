"""Right handed versions of approved left bearing revisions; actual servo placements."""
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model, read_step
from .upper_leg_carrier_v10 import instances,inner_carrier,ROOT
from .collision_bridge_v2 import place,flatten,volume
from .adapter_bridge_v3 import servo_adapter_review,world_parts,bearing_envelope,COLLAR_FRONT,adapter_print
from .leg_bearing_revision import right_leg
from .leg_foot_bearing_review_v2 import delta,complete_leg as left_complete

def right_import(name):
 i=instances()[name]
 s=place(flatten(read_step(str(ROOT/'STEP/parts'/(i['component']+'_right.step')))),i['at'])
 s.label=name;s.color='#72b6ad'
 for c in s.children:c.color='#72b6ad'
 return s

@model
def right_carrier():
 s=inner_carrier().mirror(bd.Plane.YZ)
 assert s.is_valid and len(s.solids())==1
 s.label='Right carrier - blended rear pin saddle';s.color='#f39432'
 return s

@model
def right_ankle():
 from .adapter_bridge_v3 import HORN_FRONT_Z,cyl
 i=instances()
 source=place(right_import('ankle_right'),i['servo_ankle_right']['at'],inverse=True).translate((0,0,HORN_FRONT_Z))
 relief=cyl(16,-3.7,5)-cyl(9.7,-4,6)
 s=(source-relief).clean()
 assert s.is_valid and len(s.solids())==1
 bearing=bearing_envelope();adapter=adapter_print().translate((0,0,COLLAR_FRONT))
 assert volume(s.intersect(bearing))<1e-5 and volume(s.intersect(adapter))<1e-5
 gap=bearing.distance_to(s)
 assert gap>=.1998,gap
 land=cyl(9.7,-10,1)
 assert abs(volume(source.intersect(land))-volume(s.intersect(land)))<1e-5
 for x,y in ((7,0),(-7,0),(0,7),(0,-7)):
  assert volume(s.intersect(cyl(1.68,-10,1).translate((x,y,0))))<1e-5
 samples=[]
 for x,y in ((11,0),(-11,0),(0,-10.8),(8,8),(-8,8),(0,11)):
  samples.append(volume(s.intersect(cyl(.05,-12,-3.699).translate((x,y,0))))/(3.141592653589793*.05**2))
 assert min(samples)>3.39,samples
 print('RIGHT_ANKLE',json.dumps({'bearing_gap_mm':gap,'adapter_gap_mm':adapter.distance_to(s),'sample_thickness_mm':samples,'one_solid':True}))
 s.label='Right ankle - horn bearing relief';s.color='#72b6ad'
 return s

def servo_at(name,shift=(0,0,0)):
 i=instances()
 for p in world_parts(servo_adapter_review()):
  if '从动' in p.label or (name!='servo_ankle_right' and p.label=='PRINTED front adapter'):continue
  q=place(p.rotate(bd.Axis.X,180),i[name]['at']).translate(shift)
  q.label=name+' / '+p.label
  yield q

@model
def right_complete():
 i=instances();shift=delta(i['servo_leg_2']['at']);footshift=shift+delta(i['servo_ankle_right']['at'])
 carrier=right_carrier()
 shell=right_import('upper_leg_right')
 lower=place(right_leg(),i['servo_leg_2']['at']);lower.label='Right lower leg - knee bearing relief'
 upper_servos=[p for n in ('servo_upper_leg_right','servo_leg_2') for p in servo_at(n)]
 foot_servo=list(servo_at('servo_ankle_right',shift))
 bearings=[]
 for n in ('servo_upper_leg_right','servo_leg_2','servo_ankle_right'):
  b=place(bearing_envelope(),i[n]['at'])
  if n=='servo_ankle_right':b=b.translate(shift)
  b.label=n+' / 6704 bearing';b.color='#468dd5';bearings.append(b)
 ankle=place(right_ankle(),i['servo_ankle_right']['at']).translate(shift)
 feet=[right_import(n).translate(footshift) for n in ('ankle_eye_right','ankle_bearing_outer_keeper_right','ankle_bearing_inner_cap_right','foot_a_right','foot_b_right')]
 report={p.label:volume(carrier.intersect(p)) for p in [shell,lower,*upper_servos,*bearings]}
 print('RIGHT_CARRIER_OVERLAPS',json.dumps(report))
 assert max(report.values())<1e-4,report
 adapter=next(p for p in foot_servo if 'PRINTED front adapter' in p.label)
 report2=[{'part':p.label,'bearing_overlap_mm3':volume(bearings[2].intersect(p)),'adapter_overlap_mm3':volume(adapter.intersect(p))} for p in [ankle,lower,*feet]]
 print('RIGHT_FOOT_OVERLAPS',json.dumps(report2))
 assert max(max(r['bearing_overlap_mm3'],r['adapter_overlap_mm3']) for r in report2)<1e-4
 assert volume(bearings[1].intersect(lower))<1e-5
 return bd.Compound(children=[carrier,shell,lower,*upper_servos,*foot_servo,*bearings,ankle,*feet],label='Right leg - bearing carrier and revised knee and ankle')

@model
def both_legs():
 return bd.Compound(children=[left_complete(),right_complete()],label='Both legs - bearing carriers and revised knee and ankle brackets')

