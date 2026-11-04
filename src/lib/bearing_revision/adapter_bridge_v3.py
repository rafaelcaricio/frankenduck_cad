"""Standalone fit prototype. mm; original vendor horn face z=0, case lies toward +Z.
Vendor case is authoritative to that STEP, not measured hardware. The C001 horn
and 6704 bearing are explicit provisional envelopes; see README.md.
"""
from pathlib import Path
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model, read_step

HERE = Path(__file__).resolve().parent
FRONT = 0.50
BACK = 3.00
SEAT = 0.50  # V2 prototype geometry, audited with corrected horn datum
POCKET_D = 27.30
RELIEF_D = 25.40
RIM_D = 31.30
CASE_RELIEF = 0.15
SIDE_RELIEF = 0.20  # lateral X allowance at each open-channel sidewall
HORN_D = 19.85  # user caliper measurement, 2026-11-03
HORN_TOTAL_HEIGHT = 5.80  # user measured, includes central hub
HORN_RIM_THICKNESS = 2.10  # user approximate measurement at mounting screws
HORN_BACK_Z = 4.50  # provisional vendor hub seating datum
HORN_FRONT_Z = HORN_BACK_Z-HORN_TOTAL_HEIGHT
HORN_RIM_BACK_Z = HORN_FRONT_Z+HORN_RIM_THICKNESS
HORN_HUB_RADIUS = 5.25  # vendor envelope; actual hub diameter unmeasured
# The leg's closest flat face in the collar band is 0.005 mm behind
# the provisional horn mounting plane, measured in the knee review.
LEG_FACE_Z = HORN_FRONT_Z + .005
LEG_ALLOWANCE = .20
COLLAR_FRONT = LEG_FACE_Z + LEG_ALLOWANCE
RIM_ROUND = .35
BRIDGE_BACK = 4.70
BRIDGE_INNER_D = HORN_D+2*0.20
RACE_BACK_RELIEF = 0.30


def cyl(radius, z0, z1):
    return bd.Cylinder(radius, z1-z0, align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((0,0,z0))


def world_parts(node, parent=None):
    parent = bd.Location() if parent is None else parent
    loc = parent * node.location
    if node.children:
        for c in node.children:
            yield from world_parts(c, loc)
    else:
        yield node.located(loc)


def vendor_parts():
    source = read_step(str(HERE.parents[2]/'STEP/imported/sts3215_vendor_bearing_reference.step'))
    return [p.rotate(bd.Axis.Z,-90).rotate(bd.Axis.Y,90).translate((0,-25.5,9.6)) for p in world_parts(source)]


def front_case():
    parts = [p for p in vendor_parts() if p.label.startswith('SG-ZIJI_15')]
    assert len(parts)==1 and parts[0].is_valid
    return parts[0]


def build_adapter():
    from .adapter import adapter_print as original_adapter
    case=front_case()
    # Reuse the validated 0.20 mm side-clearance plate and flush screw pads.
    body=original_adapter().translate((0,0,FRONT))
    collar=cyl(RIM_D/2,COLLAR_FRONT,SEAT+RACE_BACK_RELIEF)
    collar=collar-cyl(POCKET_D/2,COLLAR_FRONT-.1,SEAT)
    collar=collar-cyl(RELIEF_D/2,COLLAR_FRONT-.1,SEAT+RACE_BACK_RELIEF+.1)
    mouth=bd.Cone(POCKET_D/2+.30,POCKET_D/2,.30,
                 align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((0,0,COLLAR_FRONT))
    collar=collar-mouth
    front_outer=[edge for edge in collar.edges()
                 if edge.geom_type==bd.GeomType.CIRCLE
                 and abs(edge.radius-RIM_D/2)<1e-5
                 and abs(edge.center().Z-COLLAR_FRONT)<1e-5]
    assert len(front_outer)==1
    collar=bd.fillet(front_outer,RIM_ROUND)
    # Broad annular web: exact flat contact on the servo's z=4.70 front plane.
    # Conservative simple keepouts prevent it entering the lower raised cover.
    # The original detailed contour remains in the original plate beneath it.
    web=cyl(RIM_D/2,SEAT+RACE_BACK_RELIEF-.10,BRIDGE_BACK)
    web=web-cyl(BRIDGE_INNER_D/2,-8,10)
    lower=bd.Box(24.9,50,10,align=(bd.Align.CENTER,bd.Align.MAX,bd.Align.MIN)).translate((0,-5.4,3.05))
    boss=bd.Box(14.4,50,10,align=(bd.Align.CENTER,bd.Align.MAX,bd.Align.MIN)).translate((0,-9.0,1.95))
    boss_arc=cyl(11.60,1.95,10).intersect(bd.Box(30,40,15,align=(bd.Align.CENTER,bd.Align.MAX,bd.Align.MIN)).translate((0,-5.4,0)))
    web=web-lower-boss-bd.Compound(children=list(boss_arc.solids()))
    web=web-cyl(RELIEF_D/2,SEAT,SEAT+RACE_BACK_RELIEF)
    body=body.fuse(collar,web,tol=1e-5).clean()
    assert body.is_valid and len(body.solids())==1
    assert body.volume>0
    bounds=body.bounding_box()
    assert abs(bounds.size.X-RIM_D)<1e-5
    assert abs(bounds.size.Y-(33+RIM_D/2))<1e-5
    assert abs(bounds.size.Z-(BRIDGE_BACK-COLLAR_FRONT))<1e-5
    bores=[f for f in body.faces() if f.geom_type==bd.GeomType.CYLINDER
           and f.radius is not None and abs(f.radius-POCKET_D/2)<1e-5]
    assert bores
    assert abs(min(f.bounding_box().min.Z for f in bores)-(COLLAR_FRONT+.3))<1e-5
    assert abs(max(f.bounding_box().max.Z for f in bores)-SEAT)<1e-5
    assert body.distance_to(case)<1e-6
    intersection=body.intersect(case)
    overlap=sum(q.volume for q in intersection.solids()) if intersection else 0
    assert overlap<1e-5,overlap
    # The marked front annular bridge stays solid at every depth station.
    for z in (.6,1.5,3.5,4.5):
        gauge=cyl(14.5,z,z+.02)-cyl(13.8,z-.01,z+.03)
        half=gauge.intersect(bd.Box(40,30,10,align=(bd.Align.CENTER,bd.Align.MIN,bd.Align.MIN)))
        missing=bd.Compound(children=list(half.solids()))-body
        assert sum(q.volume for q in missing.solids())<1e-5
    pad_faces=[f for f in body.faces() if f.geom_type==bd.GeomType.PLANE
               and abs(f.center().Z-3.2)<1e-6 and abs(f.center().Y+29)<1e-5
               and abs(abs(f.center().X)-10.25)<1e-5]
    assert len(pad_faces)==2
    # Actual straight channel wall locations, compared to the relieved vendor.
    def walls(shape):
        return sorted([f for f in shape.faces() if f.geom_type==bd.GeomType.PLANE
                       and abs(f.normal_at().X)>.99 and 6<abs(f.center().X)<8.5
                       and f.bounding_box().min.Y < -25 and f.bounding_box().max.Y > -18
                       and f.bounding_box().max.Z<3.4 and f.area>10],key=lambda f:f.center().X)
    def x_at(f,z):
        n=f.normal_at();return (n.dot(f.center())-n.Y*(-20)-n.Z*z)/n.X
    old,new=walls(case),walls(body)
    assert len(old)==len(new)==2,(len(old),len(new))
    gaps=[abs(x_at(b,2.5))-abs(x_at(a,2.5+CASE_RELIEF)) for a,b in zip(old,new)]
    assert all(abs(g-SIDE_RELIEF)<1e-5 for g in gaps),gaps
    print('BACKING_CHECK',json.dumps({'one_solid':True,'case_overlap_mm3':overlap,
          'side_allowance_mm':gaps,'pad_z_mm':[f.center().Z for f in pad_faces],
          'front_bridge_stations_z_mm':[.6,1.5,3.5,4.5],'bounds_mm':list(bounds.size)}))
    return body


@model
def adapter_print():
    # Collar front on Z=0; slicer must assess the raised plate overhang.
    s=build_adapter().translate((0,0,-COLLAR_FRONT))
    s.label='STS3215 full bearing backing - CANDIDATE V3';s.color='#e68b2c'
    return s


def bearing_envelope():
    return cyl(13.5,SEAT-4,SEAT)-cyl(10,SEAT-4.1,SEAT+0.1)


def horn_envelope():
    # User dimensions: OD 19.85, complete height 5.8, mounting plate ~2.1.
    # Seating depth, hub OD and bore details remain vendor-based assumptions.
    h=cyl(HORN_D/2,HORN_FRONT_Z,HORN_RIM_BACK_Z)
    h=h+cyl(HORN_HUB_RADIUS,HORN_RIM_BACK_Z,HORN_BACK_Z)
    h=h-cyl(1.6,HORN_FRONT_Z-.1,HORN_BACK_Z+.1)
    for x,y in ((7,0),(-7,0),(0,7),(0,-7)):
        h=h-cyl(1.6,HORN_FRONT_Z-.1,HORN_RIM_BACK_Z+.1).translate((x,y,0))
    assert h.is_valid and len(h.solids())==1
    assert abs(h.bounding_box().size.Z-HORN_TOTAL_HEIGHT)<1e-6
    assert abs(h.bounding_box().size.X-HORN_D)<1e-6
    outer=[f for f in h.faces() if f.geom_type==bd.GeomType.CYLINDER
           and abs(f.radius-HORN_D/2)<1e-6]
    assert len(outer)==1 and abs(outer[0].bounding_box().size.Z-HORN_RIM_THICKNESS)<1e-6
    return h


@model
def fit_review():
    adapter=adapter_print().translate((0,0,COLLAR_FRONT));adapter.label='printed adapter'
    case=front_case();case.label='vendor front casing';case.color='#3f4650'
    horn=horn_envelope();horn.label='horn OD19.85 H5.8 plate2.1 - seating provisional';horn.color='#c5cbd2'
    bearing=bearing_envelope();bearing.label='6704 bearing envelope - NOT printable';bearing.color='#8ca6bf'
    for a,b,name in [(adapter,case,'case'),(adapter,horn,'horn'),(adapter,bearing,'bearing'),(bearing,case,'bearing vs case')]:
        v=sum(s.volume for s in a.intersect(b).solids()) if a.intersect(b) else 0
        assert v<1e-5,(name,v)
    assert abs(bearing.bounding_box().size.Z-4)<1e-6
    # The rotating center/shields keepout is relieved all the way through.
    keepout=cyl(12.69,COLLAR_FRONT-.1,SEAT+RACE_BACK_RELIEF-.01)-cyl(HORN_D/2,COLLAR_FRONT-.2,SEAT+RACE_BACK_RELIEF)
    overlap=adapter.intersect(keepout)
    assert not overlap or sum(s.volume for s in overlap.solids())<1e-5
    print('FIT_CHECKS',json.dumps({'adapter_valid':True,'adapter_solids':1,'case_horn_bearing_interference_mm3':0,'bearing_id_od_width_mm':[20,27,4],'horn_diameter_measured_mm':HORN_D,'horn_bore_diametral_clearance_mm':20-HORN_D,'pocket_mm':POCKET_D,'pocket_diametral_clearance_mm':POCKET_D-27,'inner_relief_mm':RELIEF_D,'outer_ring_support_radial_mm':(27-RELIEF_D)/2,'seat_z_mm':SEAT,'collar_depth_mm':SEAT-COLLAR_FRONT,'backing_inner_diameter_mm':BRIDGE_INNER_D,'inner_ring_back_relief_mm':RACE_BACK_RELIEF,'horn_total_height_mm':HORN_TOTAL_HEIGHT,'horn_plate_thickness_mm':HORN_RIM_THICKNESS,'horn_front_z_mm':HORN_FRONT_Z,'horn_engagement_mm':min(SEAT,HORN_RIM_BACK_Z)-max(SEAT-4,HORN_FRONT_Z),'bearing_ahead_of_horn_mm':HORN_FRONT_Z-(SEAT-4),'mount_hole_spacing_mm':20.5,'adapter_bounds_mm':list(adapter.bounding_box().size),'print_volume_mm3':adapter.volume}))
    return bd.Compound(children=[case,adapter,horn,bearing],label='STS3215 adapter fit study')


@model
def section_review():
    assembly=fit_review()
    cutter=bd.Box(50,100,50,align=(bd.Align.MIN,bd.Align.CENTER,bd.Align.CENTER))
    parts=[]
    for p in world_parts(assembly):
        s=p.intersect(cutter)
        if s:
            s=bd.Compound(children=list(s.solids()),label=p.label)
            s.color=p.color;parts.append(s)
    return bd.Compound(children=parts,label='center section - fit study')


@model
def servo_adapter_review():
    # Complete exterior context. The rear vendor casing has an invalid inner
    # shell; use its valid outer shell for display only, never adapter cutting.
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
    parts=[]
    for p in vendor_parts():
        if not (p.label.startswith(('SG-','ZK_','XG-')) or '从动' in p.label):
            continue
        if not p.is_valid:
            shell=max(p.shells(),key=lambda q:q.area)
            p=bd.Solid(BRepBuilderAPI_MakeSolid(shell.wrapped).Solid())
            assert p.is_valid
            p.label='rear casing - repaired outer envelope'
        p.color='#687078' if '从动' not in p.label else '#c5cbd2'
        parts.append(p)
    a=adapter_print().translate((0,0,COLLAR_FRONT));a.label='PRINTED front adapter';a.color='#f39632';parts.append(a)
    h=horn_envelope();h.label='horn OD19.85 H5.8 plate2.1 - seating provisional';h.color='#d7dbe0';parts.append(h)
    # Flip review only so the horn side faces upward in the default camera.
    return bd.Compound(children=[p.rotate(bd.Axis.X,180) for p in parts],label='STS3215 servo + front adapter')

