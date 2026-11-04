"""Thread-forming pilot preparation for the four new printed cradle bosses."""
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model
from .shared_hip_yaw_blended_v2 import blended_adapter
from .shared_hip_yaw_adapter import box
from .hip_rear_pin_closure_v1 import left as closed_left,right as closed_right
from .hip_servos_bearing_revision import instances,hardware,left_hip_dual,right_hip_dual
from .adapter_bridge_v3 import adapter_print,COLLAR_FRONT,cyl
from .collision_bridge_v2 import place,volume

CENTERS=((-11.5,13.8),(11.5,13.8))
# Provisional 2 mm plastic thread-forming screw interface. Pilot depends on
# actual screw family, material and printed hole accuracy; qualify on a coupon.
PILOT_D=1.6
ENTRY_D=2.2
ENTRY_DEPTH=.3
PILOT_END=11.8
FACE=2.3
INTERFACE=4.7

def common(a,b):
 from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
 op=BRepAlgoAPI_Common(a.wrapped,b.wrapped);op.SetFuzzyValue(1e-7);op.Build()
 assert op.IsDone()
 return bd.Compound(op.Shape())

@model
def fastened(side):
 i=instances();rn='servo_hip_l' if side=='left' else 'servo_hip_l_2'
 original=(closed_left if side=='left' else closed_right)()
 cradle=place(original,i[rn]['at'],inverse=True).solids()[0]
 own_cases=[place(q,i[rn]['at'],inverse=True) for q in hardware(i) if q.label.startswith(rn+' / ') and 'PRINTED' not in q.label and 'bearing' not in q.label and 'horn' not in q.label]
 max_case_x=max(max(abs(q.bounding_box().min.X),abs(q.bounding_box().max.X)) for q in own_cases)
 inner_x=max_case_x+.2
 case_top=max(q.bounding_box().max.Y for q in own_cases)
 roof_floor=case_top+.2
 # Repeat each existing inner-wall profile inward, retaining the external
 # silhouette and existing rounded boundary. 0.05 mm union overlap.
 walls=[f for f in cradle.faces() if f.geom_type==bd.GeomType.PLANE and abs(f.normal_at().X)>.99 and abs(abs(f.center().X)-13.705)<1e-5 and f.area>500]
 assert len(walls)==2
 for f in walls:
  sign=1 if f.center().X>0 else -1
  stock=bd.extrude(f,amount=13.705-inner_x+.05,dir=(-sign,0,0)).translate((sign*.05,0,0))
  # Rounded internal recess where the existing moving hip enters the gap
  # at full roll. Remove added stock only, preserving the original wall.
  relief=box(-16,16,-23.1,-19.9,27.0,31.9)
  relief=bd.fillet(relief.edges().filter_by(bd.Axis.X),.5)
  stock=(stock-relief).clean()
  cradle=cradle.fuse(stock,tol=1e-6).clean()
 inner=[f for f in cradle.faces() if f.geom_type==bd.GeomType.PLANE and abs(f.normal_at().X)>.99 and abs(abs(f.center().X)-inner_x)<1e-5 and f.area>500]
 assert len(inner)==2
 assert all(abs(abs(f.center().X)-inner_x)<1e-5 for f in inner)
 # Verify width against measured casing bounds rather than a visual estimate.
 assert abs((inner_x-max_case_x)-.2)<1e-5,(max_case_x,inner_x-max_case_x)
 print('CLOSE_FIT_WALLS',json.dumps({'side':side,'original_inner_width':27.41,'new_inner_width':2*inner_x,'casing_width':2*max_case_x,'minimum_side_allowance':inner_x-max_case_x,'sidewall_thickness':15.525-inner_x}),flush=True)
 # Continuous underside extension, overlapping the existing roof and walls.
 roof_stock=box(-13.755,13.755,roof_floor,13.725,10.1,35.25)
 # Keep the yaw horn's central bore and four screw access holes open through
 # the thicker roof. Their axes and sizes are measured from the native plate.
 for x,z,r in ((0,22.275,3.2),(7,22.275,1.7),(-7,22.275,1.7),(0,15.275,1.7),(0,29.275,1.7)):
  tool=cyl(r,0,10).rotate(bd.Axis.X,-90).translate((x,roof_floor-.1,z))
  roof_stock=(roof_stock-tool).clean()
 cradle=cradle.fuse(roof_stock,tol=1e-6).clean()
 assert abs((roof_floor-case_top)-.2)<1e-6
 for x,z in ((-9,12),(9,12),(-9,32),(9,32),(4,18),(4,27)):
  gauge=box(x-.1,x+.1,roof_floor,13.7,z-.1,z+.1)
  assert volume(gauge-cradle)<1e-6
 print('ROOF_EXTENSION',json.dumps({'side':side,'servo_top_local_y':case_top,'new_roof_underside_local_y':roof_floor,'nominal_top_clearance':roof_floor-case_top,'extension_below_old_plane':13.675-roof_floor,'servo_placement_unchanged':True}),flush=True)
 # Restore the underside screw-head seats from before the roof extension.
 before_recess=cradle
 for x,z in ((7,22.275),(-7,22.275),(0,15.275),(0,29.275)):
  pocket=cyl(3.1,0,13.675-roof_floor+.1).rotate(bd.Axis.X,-90).translate((x,roof_floor-.1,z))
  cradle=(cradle-pocket).clean()
  assert volume(common(cradle,pocket))<1e-5
  seat=(cyl(2.8,0,.1)-cyl(1.8,-.1,.2)).rotate(bd.Axis.X,-90).translate((x,13.675,z))
  assert volume(seat-cradle)<1e-5
 assert volume(cradle-before_recess)<1e-5
 print('ROOF_COUNTERBORES',json.dumps({'side':side,'count':4,'diameter_mm':6.2,'depth_mm':13.675-roof_floor,'seat_local_y_mm':13.675,'through_hole_diameter_mm':3.4,'material_removed_only':True}),flush=True)
 base=adapter_print().translate((0,0,COLLAR_FRONT)).solids()[0]
 # Continuous full-width top shoulder, flush with the screw ears.
 header=box(-11.5,11.5,10.3,17,FACE,INTERFACE)
 header=header-cyl(13.65,FACE-.1,INTERFACE+.1)
 adapter=base.fuse(header,tol=1e-6).clean()
 # Full-width support behind the orange shoulder. Upper plane retains the
 # existing 0.20 mm yaw-bearing clearance; all screw bores are recut below.
 bridge=box(-11.5,11.5,roof_floor,16.175,INTERFACE,12.5)
 cradle=cradle.fuse(bridge,tol=1e-6).clean()
 screws=[]
 for x,y in CENTERS:
  ear=cyl(3.2,FACE,INTERFACE).translate((x,y,0))
  root=box(x-3.2,x+3.2,0,y,FACE,INTERFACE)
  root=root-cyl(13.65,FACE-.1,INTERFACE+.1)
  ear=ear.fuse(root).clean()
  adapter=adapter.fuse(ear).clean()
  boss=cyl(3.2,INTERFACE,12.5).translate((x,y,0))
  cradle=cradle.fuse(boss).clean()
  through=cyl(1.1,FACE-.1,INTERFACE+.1).translate((x,y,0))
  sink=bd.Cone(2.1,1.1,1,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((x,y,FACE))
  adapter=(adapter-through-sink).clean()
  # 1.6 mm printed pilot: test-print fit required for M2 thread-forming.
  pilot=cyl(PILOT_D/2,INTERFACE-.1,PILOT_END).translate((x,y,0))
  before_entry=cradle
  lead=bd.Cone(ENTRY_D/2,PILOT_D/2,ENTRY_DEPTH,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((x,y,INTERFACE))
  cradle=(cradle-pilot-lead).clean()
  assert volume(cradle-before_entry)<1e-5
  assert volume(common(cradle,lead))<1e-5
  screw_tip=FACE+8
  assert PILOT_END-screw_tip>1.4
  # Material remains around the thread-forming length of the pilot.
  wall=(cyl(1.5,INTERFACE+ENTRY_DEPTH,PILOT_END)-cyl(PILOT_D/2,INTERFACE-.1,PILOT_END+.1)).translate((x,y,0))
  assert volume(wall-cradle)<1e-5
  print('THREAD_FORMING_PILOT',json.dumps({'side':side,'center':[x,y],'pilot_diameter':PILOT_D,'entry_diameter':ENTRY_D,'entry_depth':ENTRY_DEPTH,'hole_depth':PILOT_END-INTERFACE,'nominal_engagement_beyond_entry':screw_tip-INTERFACE-ENTRY_DEPTH,'tip_clearance':PILOT_END-screw_tip,'screw_major_diameter':2,'thread_forming_screw_required':True}),flush=True)
  assert volume(common(adapter,through))<1e-5
  assert volume(common(cradle,pilot))<1e-5
  screw=(cyl(1,FACE+1,FACE+8)+bd.Cone(2,1,1,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((0,0,FACE))).translate((x,y,0))
  screw.label='2 x 8 plastic thread-forming screw - nominal envelope';screw.color='#b5bbc2';screws.append(screw)
 # Relieve the inner toes of the bosses around the existing bearing web.
 cradle=(cradle-cyl(15.85,INTERFACE-.01,4.9)).clean()
 for x in (-14.5,14.5):
  side_gauge=box(x-.1,x+.1,5.8,8.1,FACE,INTERFACE)
  assert volume(side_gauge-adapter)<1e-6
 for x in (-7,0,7):
  support_gauge=box(x-.1,x+.1,12,15.8,4.91,12.4)
  assert volume(support_gauge-cradle)<1e-6
 for x in (-7,0,7):
  gauge=box(x-.1,x+.1,15.8,16.8,FACE,INTERFACE)
  assert volume(gauge-adapter)<1e-6
 assert len(adapter.solids())==len(cradle.solids())==1 and adapter.is_valid and cradle.is_valid
 assert volume(common(adapter,cradle))<1e-5
 additions=bd.Compound(children=[* (adapter-base).solids(),*(cradle-place(original,i[rn]['at'],inverse=True)).solids()])
 # New printed material must clear all purchased hardware; screw engagement
 # in the pilot is intentional and excluded from this material check.
 hits=[]
 for q in hardware(i):
  if 'PRINTED' in q.label:continue
  v=volume(common(place(additions,i[rn]['at']),q))
  if v>1e-4:hits.append([q.label,v])
 assert not hits,hits
 print('FASTENER_STATIC',json.dumps({'side':side,'centers_local_mm':CENTERS,'adapter_thickness':INTERFACE-FACE,'clearance_hole_diameter':2.2,'pilot_diameter':1.6,'screw_nominal':'2 x 8 countersunk plastic thread-forming, unselected','hardware_hits':hits}),flush=True)
 adapter.label=side.title()+' roll adapter - continuous top and side shoulders';adapter.color='#f39632'
 cradle.label=side.title()+' yaw cradle - thread-forming pilots and recessed horn screws';cradle.color='#62b6a7'
 return place(adapter,i[rn]['at']),place(cradle,i[rn]['at']),[place(s,i[rn]['at']) for s in screws],place(additions,i[rn]['at'])

@model
def left_adapter():return fastened('left')[0]
@model
def right_adapter():return fastened('right')[0]
@model
def left_cradle():return fastened('left')[1]
@model
def right_cradle():return fastened('right')[1]
@model
def installed():
 i=instances();left=fastened('left');right=fastened('right')
 hw=[p for p in hardware(i) if 'PRINTED front adapter' not in p.label]
 return bd.Compound(children=[blended_adapter(),left_hip_dual(),right_hip_dual(),left[0],left[1],*left[2],right[0],right[1],*right[2],*hw],label='Hip assembly - plastic thread-forming screw pilots')

@model
def movement_check():
 i=instances();shared=blended_adapter();hits=[]
 for side,rn,yn,hfn in [('left','servo_hip_l','servo_yaw2roll',left_hip_dual),('right','servo_hip_l_2','servo_bearing_roll',right_hip_dual)]:
  a,c,screws,added=fastened(side)
  new=bd.Compound(children=[*added.solids(),*screws]);hip=hfn()
  roll=bd.Axis(i[rn]['at'][0],(0,-1,0));yaw=bd.Axis(i[yn]['at'][0],(0,0,-1))
  for angle in range(-31,32):
   v=volume(common(new,hip.rotate(roll,angle)))
   if v>1e-4:hits.append([side,'roll',angle,v])
  for angle in range(-100,101,5):
   v=volume(common(new.rotate(yaw,angle),shared))
   if v>1e-4:hits.append([side,'yaw vs shared adapter',angle,v])
 print('FASTENER_MOTION',json.dumps({'roll_samples_per_side':63,'yaw_samples_per_side':41,'hits':hits}),flush=True)
 return bd.Compound(children=[left_adapter(),right_adapter()])
