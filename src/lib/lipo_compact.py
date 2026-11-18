"""Inward LiPo placement: keep the trunk mount fixed and shorten the holder front.

World millimetres; +Y is rearward. Pack dimensions/clearances remain those of
lipo_pod. A new shell relief also reserves the full uncovered top entry.
"""
from functools import lru_cache
import cadquery as cq
from cadgen import build123d as b
from lib import lipo_pod as base, stick_pod as sp
from lib.body_profile import world_profile_prism

PACK_X, PACK_Y, PACK_Z = base.PACK_X, base.PACK_Y, base.PACK_Z
PACK_CENTER_Y = 48.5
SHIFT_Y = PACK_CENTER_Y-base.PACK_CENTER_Y
PACK_BOTTOM = base.PACK_BOTTOM
MOUTH_Z, FLOOR_TOP, BASE_BOTTOM = base.MOUTH_Z, base.FLOOR_TOP, base.BASE_BOTTOM
CAVITY_X, CAVITY_Y = base.CAVITY_X, base.CAVITY_Y
COWL_WALL, REMOVAL_TRAVEL = base.COWL_WALL, 68.0
REAR_EXIT_TRAVEL = 35.0
COWL_REAR_Y = PACK_CENTER_Y-CAVITY_Y/2
FRONT_Y = COWL_REAR_Y-COWL_WALL
SHELL_GAP = .4


def moved(shape):
    return shape.translate(cq.Vector(0, SHIFT_Y, 0))


def cavity():
    return moved(base.cavity())


def sew_floor_seams(body):
    """Join two submicron numerical faces at the clipped socket/floor edge.

    The front clip coincides with the rounded socket's outer tangent plane.
    Its two 0.0000024 mm2 seam faces carry no designed feature. Sew their
    neighbouring boundaries within 0.001 mm; retain every functional face.
    """
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing,BRepBuilderAPI_MakeSolid
    from OCP.ShapeFix import ShapeFix_Solid
    from OCP.TopoDS import TopoDS
    keep=[];removed=[]
    for face in body.Faces():
        bb=face.BoundingBox()
        if abs(face.Area())<.00001 and abs(bb.zmin-FLOOR_TOP)<.001 and abs(bb.zmax-FLOOR_TOP)<.001:
            removed.append(face)
        else:keep.append(face)
    if not removed:return body
    assert len(removed)==2
    sew=BRepBuilderAPI_Sewing(.001)
    for face in keep:sew.Add(face.wrapped)
    sew.Perform();shells=cq.Shape.cast(sew.SewedShape()).Shells();assert len(shells)==1
    fix=ShapeFix_Solid(BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(shells[0].wrapped)).Solid())
    fix.Perform();result=cq.Shape.cast(fix.Solid())
    assert result.isValid() and len(result.Solids())==1
    return result


@lru_cache(maxsize=1)
def housing():
    # The previous holder's accepted socket, floor, rear and sides stay intact.
    old=cq.Shape.cast(base.housing().wrapped)
    keep=base.box(200,200,220,(0,FRONT_Y+100,0))
    body=moved(old).intersect(keep).clean()
    # Close the shortened front through both loft regions; the entire former
    # forward skirt/tongue is replaced, not left hidden inside the new wall.
    lower=base.lower_loft().intersect(base.box(200,200,133-BASE_BOTTOM,(0,50,BASE_BOTTOM)))
    upper=sp.local_to_world(sp.fairing()).intersect(base.box(200,200,MOUTH_Z-133,(0,50,133)))
    slab=base.box(200,COWL_WALL,100,(0,FRONT_Y+COWL_WALL/2,BASE_BOTTOM))
    front=moved(lower.fuse(upper,tol=.003)).intersect(slab).clean()
    # Follow the original body roof at the new Y datum, closing the higher
    # forward roof without a lid across the battery opening.
    cowl=world_profile_prism().intersect(base.box(61,COWL_WALL,30,(0,FRONT_Y+COWL_WALL/2,MOUTH_Z-2.4)))
    # Keep the front union exact: fuzzy fusion drops its side panels.
    # Round after the union: pre-rounding this narrow spline slab makes the
    # pinned kernel incorrectly discard the cup during the fuzzy union.
    body=body.fuse(front).clean().fuse(cowl,tol=.003).clean()
    body=body.fuse(sp.tongue(),tol=.003).clean()
    body=body.cut(cavity()).cut(sp.insert_pilots()).clean()
    crest=[e for e in body.Edges() if e.BoundingBox().zmin>170]
    assert crest
    body=sew_floor_seams(body.fillet(.5,crest).fix())
    assert body.isValid() and len(body.Solids())==1,'compact holder'
    bb=body.BoundingBox()
    assert abs(bb.zmin-BASE_BOTTOM)<.005 and bb.ymax>PACK_CENTER_Y+18
    assert bb.zmax>173, 'The full cup, floor and body-height cowl must remain'
    result=b.Solid(body.Solids()[0].wrapped);result.label='lipo_pod';result.color=b.Color(sp.COLOR)
    return result


def battery():
    return base.battery().translate((0,SHIFT_Y,0))


def reference_hardware():
    return base.reference_hardware().translate((0,SHIFT_Y,0))


@lru_cache(maxsize=1)
def shell_envelope():
    # Offset the fairing skin, then stop its obsolete forward extension at
    # the new front plane. The original body outline is rebuilt around it.
    outside=moved(sp.local_to_world(sp.fairing(-SHELL_GAP)))
    keep=base.box(200,200,220,(0,FRONT_Y-SHELL_GAP+100,0))
    # The LiPo waist differs from the original stick fairing. Reserve its
    # own expanded loft too, with 0.4 mm nominal expansion at its upper row.
    lower=moved(base.lower_loft(-SHELL_GAP*base.LOWER_WALL/base.COVER_WALL))
    lower=lower.intersect(base.box(200,200,133.1-BASE_BOTTOM,(0,50,BASE_BOTTOM)))
    outside=outside.fuse(lower).intersect(keep)
    # Full top entry; rounded plan corners match the cowl edge plus its gap.
    rear=base.PACK_CENTER_Y+19+SHIFT_Y+SHELL_GAP
    entry=(cq.Workplane('XY',origin=(0,(FRONT_Y-SHELL_GAP+rear)/2,MOUTH_Z-3))
           .rect(61+2*SHELL_GAP,rear-(FRONT_Y-SHELL_GAP)).extrude(150)
           .edges('|Z').fillet(SHELL_GAP).val())
    return outside.fuse(entry).clean()


def relieve_shell(shell_native,label,envelope=None):
    shell=cq.Shape.cast(sp.world(shell_native).wrapped)
    split=cq.Solid.makeBox(2*sp.SHELL_SPLIT_HALF_GAP,200,200,cq.Vector(-sp.SHELL_SPLIT_HALF_GAP,-50,50))
    shape=shell.cut(shell_envelope() if envelope is None else envelope).cut(split).clean()
    # Sew the submicron plane where the two clearance surfaces meet. The
    # fairing and waist have independently parameterized spline boundaries.
    tiny=[f for f in shape.Faces() if abs(f.Area())<.001 and abs(f.BoundingBox().zmin-133.1)<.001 and abs(f.BoundingBox().zmax-133.1)<.001]
    if tiny:
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing,BRepBuilderAPI_MakeSolid
        from OCP.ShapeFix import ShapeFix_Solid
        from OCP.TopoDS import TopoDS
        sew=BRepBuilderAPI_Sewing(.003)
        for f in shape.Faces():
            if not any(f.isSame(t) for t in tiny):sew.Add(f.wrapped)
        sew.Perform();shells=cq.Shape.cast(sew.SewedShape()).Shells();assert len(shells)==1
        fix=ShapeFix_Solid(BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(shells[0].wrapped)).Solid())
        fix.Perform();shape=cq.Shape.cast(fix.Solid())
    assert shape.isValid() and len(shape.Solids())==1,label
    return sp.native(b.Solid(shape.Solids()[0].wrapped),label)
