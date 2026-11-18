"""V9: enclosed LiPo side shoulders following the existing body silhouette.

Only the holder-side openings are closed. The neck opening, pack column,
paired mounting arms and removable open top retain their accepted datums.
"""
from functools import lru_cache
import cadquery as cq
from cadgen import build123d as b
from lib import lipo_mount as mount,lipo_profile as profile,lipo_compact as compact,lipo_pod as base,stick_pod as sp
from lib.body_profile import world_profile_prism
from lib.lipo_mount import (PACK_X,PACK_Y,PACK_Z,PACK_CENTER_Y,PACK_BOTTOM,
 MOUTH_Z,FLOOR_TOP,BASE_BOTTOM,CAVITY_X,CAVITY_Y,COWL_REAR_Y,COWL_WALL,
 REMOVAL_TRAVEL,REAR_EXIT_TRAVEL,battery,reference_hardware,trunk_frame,fasteners)
box=mount.box
SHOULDER_INNER_X=24.5
SHELL_GAP=.4

@lru_cache(maxsize=2)
def upper_column(grow=0.):
 # Extrude the straight section immediately below the former dome. Using
 # its exact wire preserves the lower seam while removing the inward curl.
 outer=compact.moved(sp.local_to_world(sp.fairing(-grow)))
 clipped=outer.intersect(box(-100,100,-100,150,0,154.6))
 tops=[f for f in clipped.Faces() if abs(f.BoundingBox().zmin-154.6)<1e-5 and abs(f.BoundingBox().zmax-154.6)<1e-5]
 assert len(tops)==1,'straight upper profile'
 return cq.Solid.extrudeLinear(tops[0],cq.Vector(0,0,80))

@lru_cache(maxsize=1)
def shoulder_outer():
 return upper_column().intersect(world_profile_prism()).intersect(box(-60,60,compact.FRONT_Y,100,154.6,190)).clean()

def rear_access():
 # Four-millimetre tangent runouts rise from the existing low rear rim.
 tool=box(-SHOULDER_INNER_X,SHOULDER_INNER_X,COWL_REAR_Y,100,MOUTH_Z,220)
 edges=[e for e in tool.Edges() if abs(e.Center().z-MOUTH_Z)<1e-5 and e.BoundingBox().ylen>10]
 return tool.fillet(4,edges)


def finish_opening(body):
 # Round only the exposed perimeter; the front concave corners are already
 # the 4 mm runouts. Including those corners would consume the entry bevel.
 edges=[]
 for e in body.Edges():
  bb=e.BoundingBox()
  crest=abs(bb.ymin-COWL_REAR_Y)<1e-5 and abs(bb.ymax-COWL_REAR_Y)<1e-5 and bb.zmin>174
  side=abs(abs(bb.xmin)-SHOULDER_INNER_X)<1e-5 and bb.xlen<1e-5 and bb.ylen>1 and bb.zmin>160.9 and bb.zmax>174
  rear=bb.ymin>60 and bb.zmin>156.99 and bb.zmax<161.01
  if crest or side or rear:edges.append(e)
 assert len(edges)==6,'opening perimeter'
 body=body.fillet(.5,edges).fix()
 # A ruled lead-in avoids the chamfer builder propagating a microscopic
 # remnant down the full-height rounded cavity corner.
 wires=[]
 for z,grow in [(MOUTH_Z-.5,0.),(MOUTH_Z+.5,1.)]:
  prism=(cq.Workplane('XY',origin=(0,PACK_CENTER_Y,z-1)).rect(CAVITY_X+2*grow,CAVITY_Y+2*grow).extrude(1).edges('|Z').fillet(.4+grow).val())
  face=next(f for f in prism.Faces() if abs(f.BoundingBox().zmin-z)<1e-5 and abs(f.BoundingBox().zmax-z)<1e-5)
  wires.append(face.outerWire())
 bevel=cq.Solid.makeLoft(wires,ruled=True).intersect(box(-50,50,COWL_REAR_Y,100,150,160))
 return body.cut(bevel).clean()

def convert_holder(source):
 old=cq.Shape.cast(source.wrapped)
 # Replace the entire upper rim and shoulder region; do not fuse thin
 # crescents of the former dome onto a finished edge.
 old=old.cut(box(-60,60,compact.FRONT_Y,100,154.6,220)).clean()
 extension=shoulder_outer()
 extension=extension.cut(rear_access()).cut(compact.cavity()).clean()
 body=old.fuse(extension).clean()
 body=finish_opening(body)
 assert body.isValid() and len(body.Solids())==1,'closed shoulder holder'
 result=b.Solid(body.Solids()[0].wrapped);result.label='lipo_pod';result.color=b.Color(sp.COLOR)
 return result

@lru_cache(maxsize=1)
def housing():return convert_holder(mount.housing())

@lru_cache(maxsize=1)
def shell_envelope():
 outside=compact.moved(sp.local_to_world(sp.fairing(-SHELL_GAP)))
 lower=compact.moved(base.lower_loft(-SHELL_GAP*base.LOWER_WALL/base.COVER_WALL))
 lower=lower.intersect(box(-100,100,-100,150,0,133.1))
 outside=outside.fuse(lower,upper_column(SHELL_GAP)).intersect(box(-100,100,compact.FRONT_Y-SHELL_GAP,150,0,250))
 # The V8 full-width entry box removed the body beside the holder. Reserve
 # only the actual access channel here; the side skin can return around it.
 entry=box(-SHOULDER_INNER_X-SHELL_GAP,SHOULDER_INNER_X+SHELL_GAP,compact.FRONT_Y-SHELL_GAP,100,154,250)
 front=box(-30.9,30.9,compact.FRONT_Y-SHELL_GAP,COWL_REAR_Y+SHELL_GAP,154,250)
 return outside.fuse(entry,front).clean()

def relieve_shell(shell_native,label):
 return compact.relieve_shell(shell_native,label,envelope=shell_envelope())


def finish_shell(source,accepted_lower):
 # Keep the already-finished lower seam. Re-solving its tangent round after
 # changing the upper edge's endpoint changes that otherwise unrelated trim.
 region=box(-100,100,compact.FRONT_Y-SHELL_GAP,150,154,250)
 retained=cq.Shape.cast(accepted_lower.wrapped).cut(region)
 upper=cq.Shape.cast(source.wrapped).intersect(region)
 shape=retained.fuse(upper).clean()
 assert shape.isValid() and len(shape.Solids())==1
 result=b.Solid(shape.Solids()[0].wrapped);result.label=source.label;result.color=b.Color(sp.COLOR)
 return result


def body_shell(module,side):
 from lib.body_shell_finish import round_rear_corner
 baseline=module.build(compact.relieve_shell).rotate(b.Axis.Z,-90)
 baseline.label=side+'_shell_native'
 baseline=round_rear_corner(baseline,side)
 source=module.build(relieve_shell).rotate(b.Axis.Z,-90)
 source.label=baseline.label
 return finish_shell(source,baseline)
