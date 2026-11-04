"""Canonical V9 support, mounting, assembly access and local sampled motion."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[1]

def fit():
 from lib.passive_leg_support import make_left,native_part,hardware_native,instances,box,xcyl,KNEE,Y,Z,THICK,SHELL_MOUNTS,RESTORED_MOUNTS
 from lib.bearing_revision.collision_bridge_v2 import place,volume
 from lib.bearing_revision.factories import revised,servo
 from cadgen import build123d as b
 from OCP.BRepAdaptor import BRepAdaptor_Surface
 from OCP.GeomAbs import GeomAbs_Cylinder
 i=instances();report=[]
 for side in ('left','right'):
  link,carrier,shell,horn=[native_part(k,side).rotate(b.Axis.Z,-90) for k in ('support','carrier','shell','horn')]
  assert all(p.is_valid and len(p.solids())==1 for p in (link,carrier,shell,horn))
  knee='servo_leg' if side=='left' else 'servo_leg_2'
  axis=b.Axis(i[knee]['at'][0],(-1,0,0) if side=='left' else (1,0,0))
  limits=(-134,83) if side=='left' else (-83,135)
  fixed_servo=place(servo(),i[knee]['at']);upper=place(servo(),i['servo_upper_leg_'+side]['at'])
  screws=hardware_native(side,'shell').rotate(b.Axis.Z,-90)
  heads=hardware_native(side,'horn').rotate(b.Axis.Z,-90)
  hits=[]
  for angle in sorted(set([*range(limits[0],limits[1]+1,5),limits[1],0])):
   moving=[p.rotate(axis,angle) for p in (link,horn,heads)]
   for obstacle,label in ((shell,'shell'),(carrier,'carrier'),(fixed_servo,'knee servo'),(upper,'upper servo'),(screws,'shell screws')):
    v=sum(volume(p.intersect(obstacle)) for p in moving)
    if v>.01:hits.append([angle,label,v])
  ankle=revised('ankle_left_native'+('_right' if side=='right' else '')).rotate(b.Axis.Z,-90)
  ankle_axis=b.Axis(i['servo_ankle_'+side]['at'][0],(1,0,0) if side=='left' else (-1,0,0))
  ankle_limits=(-77,77.5) if side=='left' else (-77.5,77)
  ankle_hits=[]
  for angle in sorted(set([*range(-77,78,5),*ankle_limits,0])):
   v=volume(link.intersect(ankle.rotate(ankle_axis,angle)))
   if v>.01:ankle_hits.append([angle,v])
  insertion=[]
  for shift in [0,1,3,5,10,20]:
   shifted=shell.translate((shift if side=='left' else -shift,0,0))
   overlap=volume(shifted.intersect(link))+volume(shifted.intersect(horn))
   assert overlap<.01,(side,shift,overlap)
   insertion.append([shift,overlap])
  assert not hits and not ankle_hits,(side,hits,ankle_hits)
  # The S-bend's two concentric cylindrical faces differ by 2.6 mm.
  radii=[]
  for face in link.faces():
   surf=BRepAdaptor_Surface(face.wrapped)
   if surf.GetType()==GeomAbs_Cylinder:
    c=surf.Cylinder();d=c.Axis().Direction()
    if abs(d.Y())>.999 and 2.5<c.Radius()<5.5:radii.append(c.Radius())
  assert len(radii)==4 and abs(max(radii)-min(radii)-2.6)<1e-5,radii
  # Four clear horn bores and centre access; original shell screw locations.
  for dy,dz in ((7,0),(-7,0),(0,7),(0,-7),(0,0)):
   tool=xcyl(3.19 if (dy,dz)==(0,0) else 1.69,88,100,Y+dy,Z+dz)
   if side=='right':tool=tool.mirror(b.Plane.YZ)
   assert volume(link.intersect(tool))<1e-5
  report.append(dict(side=side,valid_single_solids=True,knee_limits_deg=limits,ankle_limits_deg=ankle_limits,
   sample_step_deg=5,knee_hits=hits,ankle_hits=ankle_hits,measured_bend_normal_thickness_mm=max(radii)-min(radii),
   shell_mounts=8,shell_insertion_samples_mm3=insertion))
 result=dict(passed=True,units='mm',legs=report,coverage='Local sampled joint movement and arm/horn shell insertion; not continuous whole-robot clearance or physical strength validation.')
 (ROOT/'reports/latest/passive_leg_fit.json').write_text(json.dumps(result,indent=2)+'\n')
 return result

def bearing_contacts():
 from lib.bearing_race_clearance import TARGETS
 from lib.passive_leg_support import instances
 from lib.bearing_revision.collision_bridge_v2 import place,volume
 from lib.bearing_revision.adapter_bridge_v3 import cyl
 from importlib import import_module
 from cadgen import build123d as b
 i=instances();rows=[]
 for model,servos in TARGETS.items():
  shape=import_module('parts.'+model).make().rotate(b.Axis.Z,-90)
  assert shape.is_valid and len(shape.solids())==1
  for servo in servos:
   local=place(shape,i[servo]['at'],inverse=True)
   inner=cyl(11,-3.5,.5)-cyl(10,-3.6,.6)
   shield=cyl(13.5,-3.5,.5)-cyl(11,-3.6,.6)
   assert volume(local.intersect(inner))<1e-5
   assert volume(local.intersect(shield))<1e-5
   contact=local.distance_to(inner);gap=local.distance_to(shield)
   assert contact<1e-5 and gap>=.1999,(model,servo,contact,gap)
   rows.append(dict(model=model,servo=servo,inner_ring_contact_mm=contact,shield_gap_mm=gap))
 assert len(rows)==12
 result=dict(passed=True,interfaces=rows,measured_inner_ring_od_mm=22,lip_od_mm=21.6)
 (ROOT/'reports/latest/bearing_inner_ring.json').write_text(json.dumps(result,indent=2)+'\n')
 return result
