"""V8 LiPo: two integral arms at the original battery carrier hole positions.

World millimetres. Four M3 x 10 screws clamp the recessed trunk plate onto the
8 mm arms, with M3 x 5.7 heat-set inserts. The central tongue is removed.
"""
from functools import lru_cache
import cadquery as cq
from cadgen import build123d as b
from lib import lipo_profile as profile, lipo_compact as compact, stick_pod as sp
from lib.lipo_profile import (PACK_X,PACK_Y,PACK_Z,PACK_CENTER_Y,PACK_BOTTOM,
 MOUTH_Z,FLOOR_TOP,BASE_BOTTOM,CAVITY_X,CAVITY_Y,COWL_REAR_Y,COWL_WALL,
 REMOVAL_TRAVEL,REAR_EXIT_TRAVEL,battery,reference_hardware,relieve_shell)
ARM_X0,ARM_X1=12.0,29.0
ARM_FRONT_Y,ARM_ROOT_Y=27.7,37.8
ARM_BOTTOM,ARM_TOP=150.45,158.45
ARM_CORNER=2.0
ROOT_RADIUS=3.0
SCREWS=tuple((x,31.7) for x in (-25.,-18.,18.,25.))
PLATE_HOLE_D=3.4
INSERT_PILOT_D=4.0
INSERT_DEPTH=6.0
SCREW_SEAT_Z=162.45
SCREW=dict(size="M3-0.5",length=10.0,fastener_type="iso4762")


def box(x0,x1,y0,y1,z0,z1):
 return cq.Solid.makeBox(x1-x0,y1-y0,z1-z0,cq.Vector(x0,y0,z0))


def insert_pilots():
 cuts=[]
 for x,y in SCREWS:
  cuts.append(cq.Solid.makeCylinder(INSERT_PILOT_D/2,INSERT_DEPTH+.1,cq.Vector(x,y,ARM_TOP-INSERT_DEPTH)))
  cuts.append(cq.Solid.makeCylinder(1.5,ARM_TOP-ARM_BOTTOM+1,cq.Vector(x,y,ARM_BOTTOM-.1)))
 return cuts[0].fuse(*cuts[1:]).clean()


def arm_blank(sign):
 x0,x1=sorted((sign*ARM_X0,sign*ARM_X1))
 r=ROOT_RADIUS;y=compact.FRONT_Y;z=ARM_BOTTOM
 plane=cq.Plane(origin=(x0,0,0),xDir=(0,1,0),normal=(1,0,0))
 # A tangent quarter-circle joins the underside directly into the front
 # wall. Its full-width root extends into the existing wall for the union.
 q=(cq.Workplane(plane).moveTo(ARM_FRONT_Y,z).lineTo(y-r,z)
    .threePointArc((y-r+r/2**.5,z-r+r/2**.5),(y,z-r))
    .lineTo(ARM_ROOT_Y,z-r).lineTo(ARM_ROOT_Y,ARM_TOP)
    .lineTo(ARM_FRONT_Y,ARM_TOP).close().extrude(x1-x0).val())
 edges=[e for e in q.Edges() if abs(e.Center().y-ARM_FRONT_Y)<1e-6 and e.BoundingBox().zlen>7.9]
 q=q.fillet(ARM_CORNER,edges)
 bottom=[e for e in q.Edges() if e.BoundingBox().zmax<ARM_TOP-.1 and e.BoundingBox().ymax<ARM_ROOT_Y-.01]
 return q.fillet(.5,bottom)


def convert_holder(source):
 body=cq.Shape.cast(source.wrapped)
 # Remove the entire former central arm, including the insert pilots and
 # blended shoulders. The retained front wall is still 1.8 mm thick.
 body=body.intersect(box(-100,100,compact.FRONT_Y,150,0,220)).clean()
 arms=[arm_blank(s) for s in (-1,1)]
 body=body.fuse(*arms).clean()
 body=body.cut(insert_pilots()).cut(compact.cavity()).clean()
 assert body.isValid() and len(body.Solids())==1,'two-arm holder'
 out=b.Solid(body.Solids()[0].wrapped);out.label='lipo_pod';out.color=b.Color(sp.COLOR)
 return out


@lru_cache(maxsize=1)
def housing():return convert_holder(profile.housing())


def trunk_frame(source):
 body=cq.Shape.cast(source.wrapped)
 # Remove the obsolete pads below the plate, including their 0.7 mm rear
 # overhang. The 6 mm parent plate and all servo/shell mounts remain intact.
 for s in (-1,1):
  x0,x1=sorted((s*ARM_X0,s*ARM_X1))
  body=body.cut(box(x0-.001,x1+.001,ARM_FRONT_Y-.001,36,150,sp.PLATE_BOTTOM_Z))
  body=body.cut(box(x0-.001,x1+.001,35,36,sp.PLATE_BOTTOM_Z,159.001))
 for x,y in sp.SCREWS:
  body=body.fuse(cq.Solid.makeCylinder(sp.SCREW_CLEARANCE_D/2+.01,sp.PLATE_TOP_Z-sp.PLATE_BOTTOM_Z,cq.Vector(x,y,sp.PLATE_BOTTOM_Z)))
 for x,y in SCREWS:
  body=body.cut(cq.Solid.makeCylinder(PLATE_HOLE_D/2,20,cq.Vector(x,y,150)))
 # One open recess for each screw pair: remove the thin rear strip and
 # middle web that separate counterbores would leave. Retain 4 mm of plate.
 for sign in (-1,1):
  x0,x1=sorted((sign*15.,sign*28.))
  pocket=box(x0,x1,28.7,36,SCREW_SEAT_Z,170)
  front=[e for e in pocket.Edges() if abs(e.Center().y-28.7)<1e-6 and e.BoundingBox().zlen>7]
  body=body.cut(pocket.fillet(3,front))
 body=body.clean();assert body.isValid() and len(body.Solids())==1,'trunk for two-arm holder'
 out=b.Solid(body.Solids()[0].wrapped);out.label='trunk_base';out.color=b.Color(sp.COLOR)
 return out


def fasteners():
 import bd_warehouse.fastener as bw
 screw=bw.SocketHeadCapScrew(SCREW['size'],SCREW['length'],SCREW['fastener_type'],simple=True)
 nut=bw.HeatSetNut('M3-0.5-5.7','CNCKitchen')
 screws=[];inserts=[]
 for x,y in SCREWS:
  q=b.Solid(screw.wrapped).translate((x,y,SCREW_SEAT_Z));q.label=f'M3 x 10 screw at {x:g}';q.color=b.Color('#8c8e91');screws.append(q)
  q=b.Solid(nut.wrapped).translate((x,y,ARM_TOP-nut.nut_data['m']));q.label=f'M3 x 5.7 insert at {x:g}';q.color=b.Color('#b88b46');inserts.append(q)
 return [b.Compound(children=screws,label='lipo_mount_screws'),b.Compound(children=inserts,label='lipo_mount_inserts')]
