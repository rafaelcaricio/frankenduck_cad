"""Zeee 2S 2200 mAh holder, base construction; world millimetres, rear +Y.

Published body: 73 x 34 x 18.5 mm, XT60, 98 g (Zeee product page).
No vendor CAD: battery, leads and connector are dimensioned/visual references.
The default holder is built in stages: lipo_pod, lipo_compact, lipo_profile,
lipo_mount and lipo_shoulders, which owns the final part.
"""
from functools import lru_cache
import math
import numpy as np
import cadquery as cq
from cadgen import build123d as b
from lib import stick_pod as sp

PACK_X, PACK_Y, PACK_Z = 34.0, 18.5, 73.0
PACK_CENTER_Y = 68.0
PACK_BOTTOM = 92.0
CLEAR_X, CLEAR_Y = .6, .75
CAVITY_X, CAVITY_Y = PACK_X+2*CLEAR_X, PACK_Y+2*CLEAR_Y
COVER_WALL = 1.2
SOCKET_WALL = 1.8
LOWER_WALL = 2.0
FLOOR_TOP = 90.8
BASE_BOTTOM = 88.4
MOUTH_Z = 157.0
RIM_THICKNESS = 2.4
REMOVAL_TRAVEL = 80.0
COWL_REAR_Y = PACK_CENTER_Y-CAVITY_Y/2
COWL_WALL = 1.8
COWL_TRIM_START_Z = 158.0
COWL_EDGE_RADIUS = .5
SOURCE_URL = 'https://zeeebattery.com/products/zeee-2s-lipo-battery-2200mah-7-4v-50c-xt60'


def box(x,y,z,at):
    return cq.Solid.makeBox(x,y,z,cq.Vector(at[0]-x/2,at[1]-y/2,at[2]))


def rect_prism(x,y,z0,z1,r=.8):
    return (cq.Workplane('XY',origin=(0,PACK_CENTER_Y,z0)).rect(x,y).extrude(z1-z0)
            .edges('|Z').fillet(r).val())


def row(a,bb):
    pts=[]
    for t in sp._angles():
        radius=(abs(math.sin(t)/a)**4+abs(math.cos(t)/bb)**4)**(-.25)
        pts.append((radius*math.sin(t),PACK_CENTER_Y+radius*math.cos(t)*-1))
    return np.array(pts)


def lower_loft(inset=0):
    # One periodic surface with fixed point correspondence avoids loft seam
    # twists; its upper row matches the original fairing above its battery-specific waist.
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace,BRepBuilderAPI_Sewing,BRepBuilderAPI_MakeSolid
    from OCP.ShapeFix import ShapeFix_Solid
    from OCP.TopoDS import TopoDS
    from OCP.BRepLib import BRepLib
    bottom=row(23.7-inset,15.5-inset)
    cap=np.array(sp._box_row())+np.array(sp.STICK_ORIGIN[:2])
    if inset:
        tangent=np.roll(cap,-1,axis=0)-np.roll(cap,1,axis=0)
        normal=np.column_stack([tangent[:,1],-tangent[:,0]])
        normal/=np.linalg.norm(normal,axis=1)[:,None]
        cap-=inset*(COVER_WALL/LOWER_WALL)*normal
    rows=[]
    for zz in np.linspace(BASE_BOTTOM-1,135.6,65):
        blend=sp._smootherstep((zz-122)/(133.0-122))
        pts=(1-blend)*bottom+blend*cap
        rows.append(np.column_stack([pts,np.full(len(pts),zz)]))
    surface=sp._surface(np.array(rows));face=cq.Face(BRepBuilderAPI_MakeFace(surface,1e-6).Face())
    sew=BRepBuilderAPI_Sewing(1e-5);sew.Add(face.wrapped)
    for zz in (BASE_BOTTOM-1,135.6):
        edges=[e for e in face.Edges() if abs(e.Center().z-zz)<1e-3 and e.Length()>1]
        assert len(edges)==1
        sew.Add(cq.Face.makeFromWires(cq.Wire.assembleEdges(edges)).wrapped)
    sew.Perform();fix=ShapeFix_Solid(BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(sew.SewedShape())).Solid());fix.Perform()
    result=fix.Solid();BRepLib.SameParameter_s(result,1e-5,True)
    from OCP.ShapeUpgrade import ShapeUpgrade_ShapeDivideClosed
    divide=ShapeUpgrade_ShapeDivideClosed(result);divide.SetNbSplitPoints(3);divide.Perform()
    result=divide.Result();BRepLib.SameParameter_s(result,1e-5,True)
    solid=cq.Shape.cast(result);assert solid.isValid()
    return solid


def round_opening(shell):
    from OCP.BRep import BRep_Tool
    from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
    from OCP.gp import gp_Pnt
    band,outer=BRep_Tool.Surface_s(sp._front_band()),sp._outer_surface()
    def on(surface,edge):
        return all(GeomAPI_ProjectPointOnSurf(gp_Pnt(*edge.positionAt(t).toTuple()),surface).LowerDistance()<1e-5 for t in (0,.5,1))
    opening=[e for e in shell.Edges() if on(band,e) and on(outer,e)]
    return shell.fillet(.6,opening)


def cavity():
    # Full-width open mouth: the pack is inserted and removed vertically.
    return rect_prism(CAVITY_X,CAVITY_Y,FLOOR_TOP,PACK_BOTTOM+PACK_Z+150,.4)


def close_fairing_seam(body):
    """Sew the nominally coincident skins at Z133 without micron-wide faces.

    The two independent spline surfaces differ by less than 0.003 mm. Only
    their transition strips and the two submicron rim/window join strips
    are removed; functional surfaces stay.
    """
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing,BRepBuilderAPI_MakeSolid
    from OCP.ShapeFix import ShapeFix_Solid
    from OCP.TopoDS import TopoDS
    seam=[];keep=[]
    for face in body.Faces():
        bb=face.BoundingBox()
        at_waist=abs(bb.zmin-133)<.01 and abs(bb.zmax-133)<.01 and abs(face.Area())<.3
        c=face.Center()
        at_rim=abs(abs(c.x)-28.24)<.02 and abs(c.y-52.533)<.02 and 155.5<c.z<156.1 and abs(face.Area())<.001
        if at_waist or at_rim:
            seam.append(face)
        else:keep.append(face)
    if not seam:return body
    assert len(seam) in (2,4),'Unexpected fairing seam topology'
    sew=BRepBuilderAPI_Sewing(.003)
    for face in keep:sew.Add(face.wrapped)
    sew.Perform();shells=cq.Shape.cast(sew.SewedShape()).Shells()
    assert len(shells)==1
    fix=ShapeFix_Solid(BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(shells[0].wrapped)).Solid())
    fix.Perform();result=cq.Shape.cast(fix.Solid())
    assert result.isValid() and len(result.Solids())==1
    return result


def follow_body_roof(body):
    """Remove the front cowl's raised tip and follow the real body roof.

    Only stock above Z158 is trimmed. Keep the tongue, lower housing, battery
    cavity and screw paths. Round the exposed rear crest after clipping.
    """
    from lib.body_profile import world_profile_prism
    lower = cq.Solid.makeBox(200, 200, COWL_TRIM_START_Z,
                             cq.Vector(-100, -50, 0))
    keep = world_profile_prism().fuse(lower)
    body = body.intersect(keep).clean()
    crest = [e for e in body.Edges()
             if abs(e.BoundingBox().ymin-COWL_REAR_Y) < 1e-5
             and abs(e.BoundingBox().ymax-COWL_REAR_Y) < 1e-5
             and e.BoundingBox().zmin > 163]
    assert crest, 'Body-matched cowl needs an exposed rear crest'
    # OCCT's fillet needs its surface curves repaired at the spline seam.
    # This is geometric/topological repair, not STEP re-import healing.
    body = body.fillet(COWL_EDGE_RADIUS, crest).fix()
    assert body.isValid() and len(body.Solids()) == 1
    return body


@lru_cache(maxsize=1)
def housing():
    z_seat=sp._stick().rim_height+sp.SEAT_GAP
    inner=sp.fairing(COVER_WALL).intersect(box(200,200,200,(0,0,z_seat+2.4)))
    shell=round_opening(sp.open_front(sp.fairing().cut(inner).Solids()[0]))
    upper_outer=sp.local_to_world(sp.fairing())
    # Continue the original curved front skin up to the body arch. Keep the
    # battery's rear and side grip rim low; nothing spans its vertical opening.
    low_band=box(200,200,MOUTH_Z-133,(0,50,133))
    high_front=box(200,200,100,(0,COWL_REAR_Y-100,133))
    shell=sp.local_to_world(shell)
    raised_skin=shell.intersect(high_front).intersect(box(200,200,100,(0,50,MOUTH_Z-RIM_THICKNESS)))
    shell=shell.intersect(low_band)
    cowl=upper_outer.intersect(box(200,COWL_WALL,80,(0,COWL_REAR_Y-COWL_WALL/2,MOUTH_Z-2)))
    crest=[e for e in cowl.Edges() if abs(e.BoundingBox().ymin-COWL_REAR_Y)<1e-5 and abs(e.BoundingBox().ymax-COWL_REAR_Y)<1e-5 and e.BoundingBox().zmax>MOUTH_Z+.1]
    assert crest,'Cowl needs an exposed rear edge to round'
    cowl=cowl.fillet(.5,crest)
    lower_outer=lower_loft().intersect(box(200,200,133-BASE_BOTTOM,(0,50,BASE_BOTTOM)))
    lower_inner=lower_loft(LOWER_WALL).intersect(box(200,200,200,(0,50,FLOOR_TOP)))
    lower=lower_outer.cut(lower_inner)
    socket=rect_prism(CAVITY_X+2*SOCKET_WALL,CAVITY_Y+2*SOCKET_WALL,FLOOR_TOP,MOUTH_Z,2)
    # The floor and rim belong to the housing. No cap, latch or retainer screws.
    rim=upper_outer.intersect(box(200,200,RIM_THICKNESS,(0,50,MOUTH_Z-RIM_THICKNESS)))
    bridge=box(19,20,8,(0,48,sp.TONGUE['z0']))
    bridge=cq.Workplane(obj=bridge).edges('|Z').fillet(2).val()
    upper_socket=socket.intersect(upper_outer)
    lower_socket=socket.intersect(lower_outer)
    body=shell.fuse(lower,tol=.001).clean().fuse(upper_socket).fuse(lower_socket).fuse(rim).fuse(bridge).fuse(sp.tongue())
    body=body.cut(cavity()).cut(sp.insert_pilots()).clean()
    mouth=[];outer=[]
    for edge in body.Edges():
        bb=edge.BoundingBox()
        if abs(bb.zmin-MOUTH_Z)<1e-5 and abs(bb.zmax-MOUTH_Z)<1e-5:
            if bb.xmin>-18 and bb.xmax<18 and bb.ymin>57.9 and bb.ymax<78.1:
                mouth.append(edge)
            else:outer.append(edge)
    assert mouth and outer,'Open top must have both an inner and outer rim'
    body=body.chamfer(.5,None,mouth)
    # Soften the exposed outer rim; reselect after the entrance chamfer.
    outer=[e for e in body.Edges() if abs(e.BoundingBox().zmin-MOUTH_Z)<1e-5 and abs(e.BoundingBox().zmax-MOUTH_Z)<1e-5 and (e.BoundingBox().xmin < -25 or e.BoundingBox().xmax > 25 or e.BoundingBox().ymin < 50 or e.BoundingBox().ymax > 84)]
    body=body.fillet(.5,outer)
    bottom=[e for e in body.Edges() if abs(e.BoundingBox().zmin-BASE_BOTTOM)<1e-5 and abs(e.BoundingBox().zmax-BASE_BOTTOM)<1e-5]
    body=body.fillet(.5,bottom)
    body=close_fairing_seam(body.clean())
    # Unify the shared spline/rim junction within 3 microns before recutting
    # protected openings; an exact union creates a submicron sliver here.
    body=body.fuse(raised_skin,cowl,tol=.003).clean().cut(cavity()).cut(sp.insert_pilots()).clean()
    body=follow_body_roof(body)
    assert body.isValid() and len(body.Solids())==1,'LiPo housing must be one valid solid'
    result=b.Solid(body.Solids()[0].wrapped);result.label='lipo_pod';result.color=b.Color(sp.COLOR)
    return result


def battery():
    base=rect_prism(PACK_X,PACK_Y,PACK_BOTTOM,PACK_BOTTOM+PACK_Z,.8)
    main=b.Solid(base.wrapped);main.label='Zeee 2S 2200 mAh nominal body';main.color=b.Color('#1d1f23')
    # Separate shallow colour panels on the nominal body's rear face.
    panel=b.Box(29,.10,59).translate((0,PACK_CENTER_Y+PACK_Y/2+.05,PACK_BOTTOM+PACK_Z/2))
    panel.label='blue printed label - appearance only';panel.color=b.Color('#1242ad')
    ends=[]
    for z in (PACK_BOTTOM+.5,PACK_BOTTOM+PACK_Z-.5):
        p=b.Box(33,.10,1).translate((0,PACK_CENTER_Y+PACK_Y/2+.05,z));p.label='silver end wrap';p.color=b.Color('#c2c6cc');ends.append(p)
    return b.Compound(children=[main,panel,*ends],label='zeee_2s_battery')


def reference_hardware():
    parts=[]
    pad=b.Solid(box(33,17,1.2,(0,PACK_CENTER_Y,FLOOR_TOP)).wrapped)
    pad.label='1.2 mm soft floor pad';pad.color=b.Color('#45484c');parts.append(pad)
    # The pack's wire end faces up. Both plugs remain above the open mouth.
    for x,color in [(-5,'#b51219'),(0,'#141518')]:
        points=[(x,68,165),(x,69,170),(x,83,172),(x,98,174.5)]
        path=cq.Workplane('XY').spline(points).wire().val()
        profile=cq.Workplane(cq.Plane(origin=cq.Vector(*points[0]),normal=cq.Vector(0,1,5))).circle(1.3)
        q=b.Solid(profile.sweep(path).val().wrapped);q.label='top power lead - approximate';q.color=b.Color(color);parts.append(q)
    plug=b.Box(15.5,8.2,15).translate((-2.5,98,182));plug.label='XT60 connector envelope - approximate';plug.color=b.Color('#edbd27');parts.append(plug)
    for start,end,color in [(8,8.5,'#111111'),(10,10,'#eeeeee'),(12,11.5,'#ba181c')]:
        points=[(start,68,165),(start,70,168),(end,82,170),(end,92,171)]
        path=cq.Workplane('XY').spline(points).wire().val()
        profile=cq.Workplane(cq.Plane(origin=cq.Vector(*points[0]),normal=cq.Vector(0,2,3))).circle(.55)
        q=b.Solid(profile.sweep(path).val().wrapped);q.label='top balance lead - approximate';q.color=b.Color(color);parts.append(q)
    balance=b.Box(9,5,6).translate((10,92,174));balance.label='balance plug envelope - approximate';balance.color=b.Color('#e3e3dd');parts.append(balance)
    return b.Compound(children=parts,label='lipo_pad_and_leads')
