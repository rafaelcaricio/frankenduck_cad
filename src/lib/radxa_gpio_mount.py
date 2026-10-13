"""One flat open carrier supports the four Cubie mounting screws only.

The button area above the lower rail has no carrier behind it. Native bracket
coordinates: X is depth; placement in frankenduck.py maps Y to world -X.
"""
import math
import cadquery as cq
from cadgen import build123d as bd

BACK_X=76.0
DEPTH=4.0
GPIO_SCREWS=((-41.13,291.0),(16.87,291.0))
FACE_SCREWS=((-39.17,286.07),(38.92,286.07),(38.92,316.94))
SEAT_R=3.2

def native(x,z):return (-x-.25,z)
def circle(x,z,r,depth=DEPTH,start=BACK_X):
    u,v=native(x,z)
    return cq.Workplane('YZ',origin=(start,0,0)).center(u,v).circle(r).extrude(depth).val()
def rectangle(x0,x1,z0,z1):
    u,v=native((x0+x1)/2,(z0+z1)/2)
    return cq.Workplane('YZ',origin=(BACK_X,0,0)).center(u,v).rect(x1-x0,z1-z0).extrude(DEPTH).val()
def capsule(p,q,r):
    dx,dz=q[0]-p[0],q[1]-p[1];d=math.hypot(dx,dz)
    nx,nz=-dz*r/d,dx*r/d
    pts=[(p[0]+nx,p[1]+nz),(q[0]+nx,q[1]+nz),(q[0]-nx,q[1]-nz),(p[0]-nx,p[1]-nz)]
    web=cq.Workplane('YZ',origin=(BACK_X,0,0)).polyline([native(*v) for v in pts]).close().extrude(DEPTH).val()
    return web.fuse(circle(*p,r),circle(*q,r)).clean()

def make():
    pcbs=[(-41.13,291.0),(-41.13,314.0),(16.87,291.0),(16.87,314.0)]
    face=FACE_SCREWS
    # One broad, straight lower rail joins both sides. Its underside clears the
    # UBEC clamp by 0.8 mm; the PCB-facing surface is recessed below.
    rail=rectangle(-45.63,43.12,288.3,295.3)
    depth_edges=[e for e in rail.Edges() if e.geomType()=='LINE'
                 and abs(e.Length()-DEPTH)<1e-6
                 and abs(e.startPoint().x-e.endPoint().x)>DEPTH-.01]
    rail=rail.fillet(1.5,depth_edges)
    left=capsule(pcbs[0],pcbs[1],4.5).fuse(capsule(face[0],pcbs[0],4.5))
    left=left.cut(rectangle(-48,-30,275,283.4))
    right=capsule(face[1],face[2],4.2).fuse(
        capsule(pcbs[2],face[1],4.2),capsule(pcbs[3],face[2],4.2))
    right=right.cut(rectangle(10,48,275,282.5))
    shape=left.fuse(right,rail).clean()
    # Only the annular screw seat touches the PCB. Recess the remaining tab
    # 2.0 mm beneath the board underside, including material near GPIO solder.
    pcb_relief=cq.Solid.makeBox(2.01,74.0,35.5,cq.Vector(BACK_X+DEPTH-2.0,-27.62,285.25))
    seat=cq.Compound.makeCompound([circle(*p,SEAT_R,depth=2.2,start=BACK_X+DEPTH-2.1) for p in pcbs])
    shape=shape.cut(pcb_relief.cut(seat))
    # Imported underside microSD housing: retain 2 mm around it outside the
    # screw island. The island itself stays R3.2 and clears the housing by 1.3 mm.
    sd_clearance=cq.Solid.makeBox(6.0,21.75,17.75,cq.Vector(76.0,-27.62,293.5))
    screw_island=cq.Compound.makeCompound([circle(*p,SEAT_R,depth=7.0,start=75.0) for p in pcbs])
    shape=shape.cut(sd_clearance.cut(screw_island))
    # M2 pan-head screw from the face side; 2.4 mm grip (M2 screw length must include the added 0.8 mm standoff).
    for p in pcbs:
        u,z=native(*p)
        shape=shape.cut(cq.Solid.makeCylinder(1.2,DEPTH+.2,cq.Vector(BACK_X-.1,u,z),cq.Vector(1,0,0)),
                        cq.Solid.makeCylinder(2.2,1.6,cq.Vector(BACK_X,u,z),cq.Vector(1,0,0)))
    for x,z in face:
        u,z=native(x,z)
        shape=shape.cut(cq.Solid.makeCylinder(1.7,DEPTH+.2,cq.Vector(BACK_X-.1,u,z),cq.Vector(1,0,0)),
                        cq.Solid.makeCone(1.7,3.0,1.3,cq.Vector(BACK_X+DEPTH-1.3,u,z),cq.Vector(1,0,0)))
    shape=shape.clean()
    assert shape.isValid() and len(shape.Solids())==1,'single Radxa carrier'
    result=bd.Compound(shape.wrapped);result.label='radxa_single_carrier';result.color=bd.Color('#a4c9d4')
    return result
