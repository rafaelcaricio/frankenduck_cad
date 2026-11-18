"""V7 rear cover follows the body silhouette above a close-fitting lower cup.

Only the rear cover changes. Functional datums, the inward pack placement,
mounts, shells and the open-top service path are inherited from V6.
"""
from functools import lru_cache
import cadquery as cq
from cadgen import build123d as b
from lib import lipo_compact as compact, lipo_pod as base, stick_pod as sp
from lib.body_profile import world_profile_prism
from lib.lipo_compact import (PACK_X,PACK_Y,PACK_Z,PACK_CENTER_Y,PACK_BOTTOM,
 MOUTH_Z,FLOOR_TOP,BASE_BOTTOM,CAVITY_X,CAVITY_Y,COWL_REAR_Y,COWL_WALL,
 REMOVAL_TRAVEL,REAR_EXIT_TRAVEL,battery,reference_hardware,relieve_shell)
REAR_CAVITY_Y=PACK_CENTER_Y+CAVITY_Y/2
REAR_WALL=1.8
LOWER_REAR_Y=REAR_CAVITY_Y+REAR_WALL
TRANSITION_START_Z,TRANSITION_END_Z=128.0,140.0


def body_rear_y(z):
    ray=cq.Edge.makeLine(cq.Vector(0,0,z),cq.Vector(0,90,z))
    edges=ray.intersect(world_profile_prism()).Edges()
    assert edges
    return max(max(e.startPoint().y,e.endPoint().y) for e in edges)


@lru_cache(maxsize=1)
def rear_envelope():
    # Interpolate the actual body spline at 0.5 mm stations, rather than the
    # drawing. The lower continuation preserves the socket's 1.8 mm rear wall.
    x=-50.0
    points=[cq.Vector(x,body_rear_y(140+i*.5),140+i*.5) for i in range(37)]
    upper=cq.Edge.makeSpline(points)
    t=upper.tangentAt(0);slope=t.y/t.z
    start=cq.Vector(x,LOWER_REAR_Y,TRANSITION_START_Z)
    end=points[0]
    join=cq.Edge.makeBezier([start,cq.Vector(x,LOWER_REAR_Y,132),
                            cq.Vector(x,end.y-4*slope,136),end])
    a=cq.Vector(x,LOWER_REAR_Y,0);top=cq.Vector(x,points[-1].y,210)
    p=cq.Vector(x,-100,210);q=cq.Vector(x,-100,0)
    edges=[cq.Edge.makeLine(a,start),join,upper,cq.Edge.makeLine(points[-1],top),
           cq.Edge.makeLine(top,p),cq.Edge.makeLine(p,q),cq.Edge.makeLine(q,a)]
    face=cq.Face.makeFromWires(cq.Wire.assembleEdges(edges))
    return cq.Solid.extrudeLinear(face,cq.Vector(100,0,0))


def finish_rear(body):
    """Round the new rear lip and sew micron-sized Boolean transition seams."""
    import math
    from scipy.optimize import brentq
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing,BRepBuilderAPI_MakeSolid
    from OCP.ShapeFix import ShapeFix_Solid
    from OCP.TopoDS import TopoDS
    r=.5;zc=MOUTH_Z-r
    def slope(z):return (body_rear_y(z+.001)-body_rear_y(z-.001))/.002
    zt=brentq(lambda z:z+r*slope(z)/math.sqrt(1+slope(z)**2)-zc,156,157)
    yt=body_rear_y(zt);yc=yt-r/math.sqrt(1+slope(zt)**2)
    angle=math.atan2(zt-zc,yt-yc);mid=(angle+math.pi/2)/2
    plane=cq.Plane(origin=(-50,0,0),xDir=(0,1,0),normal=(1,0,0))
    tool=(cq.Workplane(plane).moveTo(yc,MOUTH_Z)
          .threePointArc((yc+r*math.cos(mid),zc+r*math.sin(mid)),(yt,zt))
          .lineTo(yt+3,zt).lineTo(yt+3,MOUTH_Z+2)
          .lineTo(yc,MOUTH_Z+2).close().extrude(100).val())
    body=body.cut(tool).clean()
    maker=BRepBuilderAPI_MakeSolid()
    for shell in body.Shells():
        tiny=[f for f in shell.Faces() if abs(f.Area())<.001]
        if not tiny:maker.Add(TopoDS.Shell_s(shell.wrapped));continue
        for f in tiny:
            bb=f.BoundingBox()
            assert (156.49<bb.zmin<=bb.zmax<157.01) or (88.89<bb.zmin<=bb.zmax<88.91),(f.Area(),bb.zmin,bb.zmax)
        sew=BRepBuilderAPI_Sewing(.003)
        for f in shell.Faces():
            if not any(f.isSame(t) for t in tiny):sew.Add(f.wrapped)
        sew.Perform();fixed=cq.Shape.cast(sew.SewedShape()).Shells();assert len(fixed)==1
        maker.Add(TopoDS.Shell_s(fixed[0].wrapped))
    fix=ShapeFix_Solid(maker.Solid());fix.Perform();result=cq.Shape.cast(fix.Solid())
    assert result.isValid() and len(result.Solids())==1
    return result


@lru_cache(maxsize=1)
def housing():
    old=cq.Shape.cast(compact.housing().wrapped)
    envelope=rear_envelope()
    lower=compact.moved(base.lower_loft()).intersect(base.box(200,200,133-BASE_BOTTOM,(0,50,BASE_BOTTOM)))
    upper=compact.moved(sp.local_to_world(sp.fairing())).intersect(base.box(200,200,MOUTH_Z-133,(0,50,133)))
    outside=lower.fuse(upper,tol=.003)
    # Replace the rear fairing wall in one continuous band. Fill up to the
    # cavity plane, so a hairline gap cannot remain behind the old socket wall.
    band=outside.intersect(envelope).intersect(base.box(200,100,100,(0,REAR_CAVITY_Y+49.95,BASE_BOTTOM)))
    body=old.intersect(envelope).clean().fuse(band).clean()
    body=body.cut(compact.cavity()).cut(sp.insert_pilots()).clean()
    # A quarter-cylinder cut rounds the new straight rear/floor junction.
    # The adjoining side edges already have their accepted 0.5 mm rounds.
    y,z=LOWER_REAR_Y,BASE_BOTTOM
    plane=cq.Plane(origin=(-50,0,0),xDir=(0,1,0),normal=(1,0,0))
    tool=(cq.Workplane(plane).moveTo(y-.5,z-1).lineTo(y-.5,z)
          .threePointArc((y-.5+.5/2**.5,z+.5-.5/2**.5),(y,z+.5))
          .lineTo(y+1,z+.5).lineTo(y+1,z-1).close().extrude(100).val())
    body=body.cut(tool).clean()
    # Restore the rear entrance bevel after forming the new rear wall.
    mouth=[e for e in body.Edges() if abs(e.BoundingBox().zmin-MOUTH_Z)<1e-5 and abs(e.BoundingBox().zmax-MOUTH_Z)<1e-5 and abs(e.Center().y-REAR_CAVITY_Y)<.01 and e.BoundingBox().xmin>-18 and e.BoundingBox().xmax<18]
    assert mouth,'rear entrance edge'
    body=body.chamfer(.5,None,mouth).fix()
    body=finish_rear(body)
    assert body.isValid() and len(body.Solids())==1,'profiled holder'
    result=b.Solid(body.Solids()[0].wrapped);result.label='lipo_pod';result.color=b.Color(sp.COLOR)
    return result
