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
SEAT = 1.80
POCKET_D = 27.30
RELIEF_D = 25.40
RIM_D = 31.30
CASE_RELIEF = 0.15
SIDE_RELIEF = 0.20  # lateral X allowance at each open-channel sidewall
HORN_D = 19.85  # user caliper measurement, 2026-11-03
HORN_TOTAL_HEIGHT = 5.80  # user measured, includes central hub
HORN_RIM_THICKNESS = 2.10  # user approximate measurement at mounting screws
HORN_BACK_Z = 4.50  # PROVISIONAL: preserve the vendor horn rear hub datum
HORN_FRONT_Z = HORN_BACK_Z-HORN_TOTAL_HEIGHT
HORN_RIM_BACK_Z = HORN_FRONT_Z+HORN_RIM_THICKNESS
HORN_HUB_RADIUS = 5.25  # vendor envelope; actual custom hub diameter unmeasured


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
    case=front_case()
    # A modest full-face plate plus a circular bearing register. Negative-Z face
    # is planar. Upper screw sites are deliberately NOT used: bearing overlaps.
    profile=bd.RectangleRounded(26.5,33,1.0).translate((0,-16.5,0)) + bd.Circle(RIM_D/2)
    body=bd.extrude(profile,amount=BACK-FRONT).translate((0,0,FRONT))
    # Actual vendor B-rep boolean. Only the shallow front 3.0 mm is used, before
    # the case's internal cavity begins, so this cannot create internal fingers.
    # Preserve the 0.15 mm axial relief, then expand the two drafted channel
    # walls by exactly 0.20 mm along X each. The channel is open at both ends
    # (adapter end and central through relief); no enclosing Y end walls exist.
    # Translate/cut the real vendor geometry instead of scaling the servo.
    body=body-case.translate((0,0,-CASE_RELIEF))
    for dx in (-SIDE_RELIEF,SIDE_RELIEF):
        body=body-case.translate((dx,0,-CASE_RELIEF))
    # Narrow annular shoulder contacts only the outermost 0.8 mm of a 27 mm ring.
    body=body-cyl(RELIEF_D/2,-5,8)-cyl(POCKET_D/2,-5,SEAT)
    # 0.3 mm entry bevel makes insertion possible without sharp printed edges.
    mouth=bd.Cone(POCKET_D/2+0.30,POCKET_D/2,0.30,
                  align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((0,0,FRONT))
    body=body-mouth
    for x in (-10.25,10.25):
        # Small flat contact pads restore the actual mounting plane, leaving
        # the rest of the boolean recess relieved.
        pad=cyl(1.65,2.95,3.20).translate((x,-29,0))
        body=body+pad
        hole=cyl(1.1,-5,8).translate((x,-29,0))
        countersink=bd.Cone(2.0,1.1,0.9,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((x,-29,FRONT))
        body=body-hole-countersink
    body=body.clean()
    # The vendor has a sealed internal blind cavity at this location; subtracting
    # its solid leaves a small detached plug. It is not adapter material.
    solids=sorted(body.solids(),key=lambda s:s.volume,reverse=True)
    assert sum(s.volume for s in solids[1:])<0.51
    body=solids[0]
    assert body.is_valid and len(body.solids())==1
    zs=[v.center().Z for v in body.vertices()]
    assert abs(max(zs)-min(zs)-2.70)<1e-5
    # Independently inspect the generated bore surfaces and shoulder plane.
    bores=[f for f in body.faces() if f.geom_type==bd.GeomType.CYLINDER and abs(f.radius-POCKET_D/2)<1e-5]
    assert bores and max(f.bounding_box().max.Z for f in bores)>SEAT-1e-5
    seats=[f for f in body.faces() if f.geom_type==bd.GeomType.PLANE and abs(f.center().Z-SEAT)<1e-5 and abs(f.normal_at().Z)>0.99]
    assert seats
    # Check actual opposing wall planes at a shared section, compensating only
    # for the independently specified axial relief. Draft is preserved.
    def channel_planes(shape):
        return [f for f in shape.faces()
                if f.geom_type==bd.GeomType.PLANE
                and abs(f.normal_at().X)>0.99
                and 6 < abs(f.center().X) < 8.5
                and f.bounding_box().min.Y < -25
                and f.bounding_box().max.Y > -14
                and f.bounding_box().max.Z < 3.4
                and f.area>10]
    def plane_x(face,z):
        n=face.normal_at();c=face.center()
        return (n.dot(c)-n.Y*(-20)-n.Z*z)/n.X
    original=sorted(channel_planes(case),key=lambda f:f.center().X)
    finished=sorted(channel_planes(body),key=lambda f:f.center().X)
    assert len(original)==len(finished)==2,(len(original),len(finished))
    measured=[]
    for original_wall,finished_wall in zip(original,finished):
        clearance=abs(plane_x(finished_wall,2.5))-abs(plane_x(original_wall,2.5+CASE_RELIEF))
        assert abs(clearance-SIDE_RELIEF)<1e-5,clearance
        measured.append(clearance)
    # Both annular screw pads remain on the vendor mounting plane z=3.2.
    pad_faces=[f for f in body.faces() if f.geom_type==bd.GeomType.PLANE
               and abs(f.center().Z-3.2)<1e-6 and abs(f.center().Y+29)<1e-5
               and abs(abs(f.center().X)-10.25)<1e-5]
    assert len(pad_faces)==2
    assert all(abs(f.area-3.141592653589793*(1.65**2-1.1**2))<1e-4 for f in pad_faces)
    print('RECESS_CLEARANCE_CHECK',json.dumps({'side_allowance_mm':measured,
          'total_channel_width_increase_mm':sum(measured),
          'mounting_pad_z_mm':[f.center().Z for f in pad_faces]}))
    return body


@model
def adapter_print():
    # Broad planar front on bed; cavity faces upward. Printable part ONLY.
    s=build_adapter().translate((0,0,-FRONT))
    s.label='STS3215 front adapter - FIT PROTOTYPE';s.color='#e68b2c'
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
    adapter=adapter_print().translate((0,0,FRONT));adapter.label='printed adapter'
    case=front_case();case.label='vendor front casing';case.color='#3f4650'
    horn=horn_envelope();horn.label='horn OD19.85 H5.8 plate2.1 - seating provisional';horn.color='#c5cbd2'
    bearing=bearing_envelope();bearing.label='6704 bearing envelope - NOT printable';bearing.color='#8ca6bf'
    for a,b,name in [(adapter,case,'case'),(adapter,horn,'horn'),(adapter,bearing,'bearing'),(bearing,case,'bearing vs case')]:
        v=sum(s.volume for s in a.intersect(b).solids()) if a.intersect(b) else 0
        assert v<1e-5,(name,v)
    assert abs(bearing.bounding_box().size.Z-4)<1e-6
    # The rotating center/shields keepout is relieved all the way through.
    keepout=cyl(12.69,-3,3.2)-cyl(HORN_D/2,-3.1,3.3)
    overlap=adapter.intersect(keepout)
    assert not overlap or sum(s.volume for s in overlap.solids())<1e-5
    print('FIT_CHECKS',json.dumps({'adapter_valid':True,'adapter_solids':1,'case_horn_bearing_interference_mm3':0,'bearing_id_od_width_mm':[20,27,4],'horn_diameter_measured_mm':HORN_D,'horn_bore_diametral_clearance_mm':20-HORN_D,'pocket_mm':POCKET_D,'pocket_diametral_clearance_mm':POCKET_D-27,'inner_relief_mm':RELIEF_D,'outer_ring_support_radial_mm':(27-RELIEF_D)/2,'seat_z_mm':SEAT,'horn_total_height_mm':HORN_TOTAL_HEIGHT,'horn_plate_thickness_mm':HORN_RIM_THICKNESS,'horn_front_z_mm_provisional':HORN_FRONT_Z,'horn_engagement_mm':min(SEAT,HORN_RIM_BACK_Z)-max(SEAT-4,HORN_FRONT_Z),'bearing_ahead_of_horn_mm_provisional':HORN_FRONT_Z-(SEAT-4),'mount_hole_spacing_mm':20.5,'adapter_bounds_mm':list(adapter.bounding_box().size),'print_volume_mm3':adapter.volume}))
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
    a=adapter_print().translate((0,0,FRONT));a.label='PRINTED front adapter';a.color='#f39632';parts.append(a)
    h=horn_envelope();h.label='horn OD19.85 H5.8 plate2.1 - seating provisional';h.color='#d7dbe0';parts.append(h)
    # Flip review only so the horn side faces upward in the default camera.
    return bd.Compound(children=[p.rotate(bd.Axis.X,180) for p in parts],label='STS3215 servo + front adapter')

