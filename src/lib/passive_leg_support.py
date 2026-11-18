"""Approved V9 passive leg supports and integral shell bridges, millimetres.
All geometry is rebuilt from maintained factories and vendor inputs; no scratch dependencies.
The original knee axes, joint spacing and measured travel remain unchanged.
"""
from pathlib import Path
import math,json
from cadgen import build123d as b,declare_input
from .bearing_revision.runtime import cached_shape
from .bearing_revision.collision_bridge_v2 import place,volume
from .bearing_revision.adapter_bridge_v3 import vendor_parts,adapter_print,COLLAR_FRONT
from .viewer_palette import paint_original
ROOT=Path(__file__).resolve().parents[2]
KNEE=(51.975,42.8991,70.675)
Y,Z=KNEE[1:]
FRONT=89.225
THICK=2.6
RESTORED_MOUNTS=[(Y-10.25,Z+8.3),(Y+10.25,Z+8.3)]
SHELL_MOUNTS=[(32.649,103.425),(53.149,103.425),(2.9,110.625),(2.9,90.125),(27.35,110.625),(27.35,90.125)]

def instances():
 path=ROOT/'source/assembly_data.json';declare_input(path)
 return {i['name']:i for i in json.loads(path.read_text())['instances']}

def bake(s):
 solids=list(s.solids());return solids[0] if len(solids)==1 else b.Compound(children=solids)

def box(x0,x1,y0,y1,z0,z1):
 return b.Box(x1-x0,y1-y0,z1-z0,align=(b.Align.MIN,)*3).translate((x0,y0,z0))

def xcyl(r,x0,x1,y=Y,z=Z):
 return b.Cylinder(r,x1-x0,align=(b.Align.CENTER,b.Align.CENTER,b.Align.MIN)).rotate(b.Axis.Y,90).translate((x0,y,z))

@cached_shape
def horn_left():
 horn=next(p for p in vendor_parts() if '从动' in p.label)
 return place(horn,instances()['servo_leg']['at'])

@cached_shape
def carrier_left():
 stock_path=ROOT/'STEP/imported/passive_carrier_stock.step';declare_input(stock_path)
 stock=bake(b.import_step(stock_path)).rotate(b.Axis.Z,-90)
 carrier=bake(stock.intersect(box(40,54.975,-40,80,40,140)))
 adapter_path=ROOT/'STEP/imported/passive_servo_adapter_stock.step';declare_input(adapter_path)
 base=bake(b.import_step(adapter_path));i=instances()
 carrier=bake(carrier.fuse(place(base,i['servo_leg']['at']),place(base,i['servo_upper_leg_left']['at']),tol=1e-6).clean())
 assert carrier.is_valid and len(carrier.solids())==1
 return carrier

def ankle_adapter_base():
 # Approved lower ankle interface, including its checked end-stop relief.
 path=ROOT/'STEP/imported/passive_ankle_mount_stock.step';declare_input(path)
 return bake(b.import_step(path))

def radial_running_envelope(shape):
 # Project tessellated triangles onto axial-distance/radius space. Include
 # edge midpoints so convex radius interpolation stays within the allowance.
 from shapely.geometry import MultiPoint
 from shapely.ops import unary_union
 vertices,triangles=shape.tessellate(.025,.08)
 cells=[]
 for tri in triangles:
  p=[vertices[i] for i in tri]
  samples=p+[(p[i]+p[(i+1)%3])*.5 for i in range(3)]+[(p[0]+p[1]+p[2])/3]
  poly=MultiPoint([(v.X,math.hypot(v.Y-Y,v.Z-Z)) for v in samples]).convex_hull
  if poly.area>1e-8:cells.append(poly)
 profile=unary_union(cells).buffer(.35).simplify(.025,preserve_topology=True)
 # The axis itself is not a shell contact surface; avoid negative radii.
 from shapely.geometry import box as rect
 profile=profile.intersection(rect(70,.001,105,120))
 solids=[]
 for poly in ([profile] if profile.geom_type=='Polygon' else profile.geoms):
  outer=b.Wire.make_polygon([(x,Y+rr,Z) for x,rr in list(poly.exterior.coords)[:-1]],close=True)
  holes=[b.Wire.make_polygon([(x,Y+rr,Z) for x,rr in list(r.coords)[:-1]],close=True) for r in poly.interiors]
  face=b.Face(outer,holes)
  solids.append(b.revolve(face,axis=b.Axis(KNEE,(1,0,0)),revolution_arc=360))
 return b.Compound(children=[solid for p in solids for solid in p.solids()])

def make_link():
 # Circular S-bend with 2.6 mm true normal wall thickness.
 start=35.475;end=43.475;d=7.55;h=end-start;R=(d*d+h*h)/(4*d)
 theta=math.acos(1-d/(2*R));c1=83.975+R;c2=91.525-R
 def p1(t,off):return (c1-(R-off)*math.cos(t),Y-13.25,start+(R-off)*math.sin(t))
 def p2(t,off):return (c2+(R+off)*math.cos(t),Y-13.25,end-(R+off)*math.sin(t))
 e=[b.Edge.make_line(p1(0,-1.3),p1(0,1.3)),
 b.Edge.make_three_point_arc(p1(0,1.3),p1(theta/2,1.3),p1(theta,1.3)),
 b.Edge.make_three_point_arc(p2(theta,1.3),p2(theta/2,1.3),p2(0,1.3)),
 b.Edge.make_line(p2(0,1.3),(92.825,Y-13.25,Z)),
 b.Edge.make_line((92.825,Y-13.25,Z),(90.225,Y-13.25,Z)),
 b.Edge.make_line((90.225,Y-13.25,Z),p2(0,-1.3)),
 b.Edge.make_three_point_arc(p2(0,-1.3),p2(theta/2,-1.3),p2(theta,-1.3)),
 b.Edge.make_three_point_arc(p1(theta,-1.3),p1(theta/2,-1.3),p1(0,-1.3))]
 foot=b.extrude(b.Face(b.Wire(e)),amount=26.5,dir=(0,1,0))
 # Tangent side taper from original ankle plate to a 19.2 mm wide arm.
 def q(y,z):return (80,Y+y,z)
 edges=[b.Edge.make_line(q(-13.25,35.475),q(13.25,35.475)),
 b.Edge.make_bezier(q(13.25,35.475),q(13.25,39),q(9.6,42),q(9.6,46)),
 b.Edge.make_line(q(9.6,46),q(9.6,Z)),b.Edge.make_line(q(9.6,Z),q(-9.6,Z)),
 b.Edge.make_line(q(-9.6,Z),q(-9.6,46)),
 b.Edge.make_bezier(q(-9.6,46),q(-9.6,42),q(-13.25,39),q(-13.25,35.475))]
 mask=b.extrude(b.Face(b.Wire(edges)),amount=20,dir=(1,0,0))
 foot=bake(foot.intersect(mask))
 adapter=ankle_adapter_base()
 link=foot.fuse(adapter,xcyl(9.6,89.225,92.825),tol=1e-6).clean()
 for dy,dz in ((7,0),(-7,0),(0,7),(0,-7)):
  link=link-xcyl(1.7,88,100,Y+dy,Z+dz)
 return (link-xcyl(3.2,88,100)).clean()

def make_frame():
 radius=math.hypot(10.25,8.3);angle=math.atan2(8.3,10.25);r=3.2
 def q(rad,t):return (87.175,Y+rad*math.cos(t),Z+rad*math.sin(t))
 def cap(sign): return (87.175,Y+sign*(10.25+r*math.sin(angle)),Z+8.3-r*math.cos(angle))
 outer=radius+r;inner=radius-r
 e=[b.Edge.make_three_point_arc(q(outer,angle),q(outer,math.pi/2),q(outer,math.pi-angle)),
 b.Edge.make_three_point_arc(q(outer,math.pi-angle),cap(-1),q(inner,math.pi-angle)),
 b.Edge.make_three_point_arc(q(inner,math.pi-angle),q(inner,math.pi/2),q(inner,angle)),
 b.Edge.make_three_point_arc(q(inner,angle),cap(1),q(outer,angle))]
 frame=b.extrude(b.Face(b.Wire(e)),amount=2.6,dir=(1,0,0))
 # Offset the fixed spine into the open upper quadrant, away from the
 # moving arm's extreme angle; blend depth around the raised case.
 def p(x,z): return (x,Y-8,z)
 e=[b.Edge.make_line(p(87.175,Z+10),p(89.775,Z+10)),
 b.Edge.make_line(p(89.775,Z+10),p(89.775,Z+12)),
 b.Edge.make_bezier(p(89.775,Z+12),p(89.775,Z+14),p(92.075,Z+15),p(92.075,Z+17)),
 b.Edge.make_line(p(92.075,Z+17),p(92.075,Z+33)),
 b.Edge.make_line(p(92.075,Z+33),p(89.475,Z+33)),
 b.Edge.make_line(p(89.475,Z+33),p(89.475,Z+17)),
 b.Edge.make_bezier(p(89.475,Z+17),p(89.475,Z+15),p(87.175,Z+14),p(87.175,Z+12)),
 b.Edge.make_line(p(87.175,Z+12),p(87.175,Z+10))]
 spine=b.extrude(b.Face(b.Wire(e)),amount=6,dir=(0,1,0))
 frame=frame.fuse(spine).clean()
 return frame

def xcone(r0,r1,x0,x1,y,z):
 return b.Cone(r0,r1,x1-x0,align=(b.Align.CENTER,b.Align.CENTER,b.Align.MIN)).rotate(b.Axis.Y,90).translate((x0,y,z))

def original_shell_boss(y,z):
 # Restore the actual boss only, not nearby wall fragments inside a bounding tube.
 r=3.243;f=1.25
 def p(x,rad):return (x,y+rad,z)
 edges=[b.Edge.make_line(p(87.175,0),p(87.175,r)),
 b.Edge.make_line(p(87.175,r),p(94.925,r)),
 b.Edge.make_three_point_arc(p(94.925,r),p(94.925+f/math.sqrt(2),r+f-f/math.sqrt(2)),p(96.175,r+f)),
 b.Edge.make_line(p(96.175,r+f),p(96.375,r+f)),
 b.Edge.make_line(p(96.375,r+f),p(96.375,0)),
 b.Edge.make_line(p(96.375,0),p(87.175,0))]
 return b.revolve(b.Face(b.Wire(edges)),axis=b.Axis((0,y,z),(1,0,0)),revolution_arc=360)

@cached_shape
def make_left():
 i=instances()
 link=make_link();carrier=carrier_left();horn=horn_left()
 from .passive_leg_shell import make as make_shell
 shell=place(make_shell(),i['upper_leg_left']['at'])
 uncut=shell
 # Keep the existing outside, cutting only the running space from its inside.
 from shapely.geometry import Polygon,Point
 from shapely.affinity import rotate
 from shapely.ops import unary_union
 profile=Polygon([(-14,25-Z),(14,25-Z),(14,0),(-14,0)]).union(Point(0,0).buffer(14,resolution=64))
 sweep=unary_union([rotate(profile,a,origin=(0,0)) for a in range(-83,136)]).buffer(.8).simplify(.03)
 pts=[(81,Y+y,Z+z) for y,z in list(sweep.exterior.coords)[:-1]]
 broad=b.extrude(b.Face(b.Wire.make_polygon(pts,close=True)),amount=14.525,dir=(1,0,0))
 running=radial_running_envelope(b.Compound(children=[link,horn]))
 link.parent=None;horn.parent=None
 shell=shell-bake(broad.intersect(running))
 for y,z in SHELL_MOUNTS:
  shell=shell.fuse(bake(original_shell_boss(y,z).intersect(uncut)),tol=1e-6).clean()
 # The user marked this inner skirt for removal. Exterior rounded skin stays.
 shell=shell-box(82,92.2,-30,80,30,72.5)
 frame=make_frame()
 # A single curved root joins the fixed spine to the shell's inner surface.
 def p(x,z):return (x,Y-8,z)
 edges=[b.Edge.make_line(p(89.475,95),p(92.075,95)),
 b.Edge.make_three_point_arc(p(92.075,95),p(93.28348,97.91652),p(96.2,99.125)),
 b.Edge.make_line(p(96.2,99.125),p(96.4,99.125)),
 b.Edge.make_line(p(96.4,99.125),p(96.4,104)),
 b.Edge.make_line(p(96.4,104),p(89.475,104)),
 b.Edge.make_line(p(89.475,104),p(89.475,95))]
 root=b.extrude(b.Face(b.Wire(edges)),amount=6,dir=(0,1,0))
 shell=shell.fuse(frame,root,tol=1e-6).clean()
 for y,z in RESTORED_MOUNTS:
  shell=shell-xcyl(1.1,84,88.675,y,z)-xcone(1.1,2.2,88.675,89.775,y,z)-xcyl(2.2,89.775,100,y,z)
 for y,z in SHELL_MOUNTS:
  shell=shell-xcyl(1.1,86,90.175,y,z)-xcyl(2.2,90.175,100,y,z)
 shell=shell.clean()
 assert shell.is_valid and len(shell.solids())==1,('shell',len(shell.solids()))
 assert link.is_valid and len(link.solids())==1
 # All lower pads stay 2.6 mm deep with 1.5 mm material below the countersink.
 for y,z in RESTORED_MOUNTS:
  assert volume(shell.intersect(xcyl(1.09,84,100,y,z)))<1e-6
  assert volume(link.intersect(xcyl(2.2,89.775,100,y,z)))<1e-6
 from .bearing_revision.factories import servo as servo_model
 servo=place(servo_model(),i['servo_leg']['at'])
 assert volume(shell.intersect(servo))<.01,volume(shell.intersect(servo))
 assert volume(link.intersect(horn))<1e-5
 assert link.distance_to(horn)<1e-5
 return link,carrier,shell,horn


@cached_shape
def native_part(kind,side):
 index={'support':0,'carrier':1,'shell':2,'horn':3}[kind]
 shape=make_left()[index]
 if side=='right':shape=shape.mirror(b.Plane.YZ)
 shape=shape.rotate(b.Axis.Z,90)
 shape.label={'support':'passive_leg_support','carrier':'bearing_carrier','shell':'upper_leg','horn':'knee_passive_horn'}[kind]+'_'+side
 shape.color='#bfc3c7' if kind=='horn' else '#1d1d1f'
 if kind=='shell':shape=paint_original(shape,'upper_leg_'+side)
 return shape

@cached_shape
def hardware_native(side,kind):
 if kind=='shell':
  shapes=[]
  for number,(y,z) in enumerate(SHELL_MOUNTS+RESTORED_MOUNTS,1):
   if number<=6:
    screw=xcyl(1,84.175,90.175,y,z).fuse(xcyl(2,90.175,91.775,y,z)).clean()
   else:
    screw=xcyl(1,84.775,88.775,y,z).fuse(xcone(1,2,88.775,89.775,y,z)).clean()
   shapes.append(screw)
 else:
  shapes=[xcyl(3,92.825,95.825,Y+dy,Z+dz) for dy,dz in ((7,0),(-7,0),(0,7),(0,-7))]
 # Transform each leaf before assembling. Mirroring the compound's BRep alone
 # leaves its child tree unmirrored; the color-preserving STEP exporter uses
 # that tree and would put the right-side fasteners on the left leg.
 if side=='right':shapes=[leaf.mirror(b.Plane.YZ) for leaf in shapes]
 shapes=[leaf.rotate(b.Axis.Z,90) for leaf in shapes]
 for leaf in shapes:leaf.color='#bfc3c7'
 shape=b.Compound(children=shapes);shape.color='#bfc3c7'
 return shape
