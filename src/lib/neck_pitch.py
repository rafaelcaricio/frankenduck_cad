"""Fresh reference-shaped neck pitch bracket, canonical; native frame mm.

DuckBuddy neck_pitch: 18 mm wide, 3 mm arms, 29 mm inside spacing,
semicircular ends and simple inner-corner braces. STS3215 adaptation retains
Frankenduck's existing axes and 14 mm horn screw pitch; no legacy solid is used.
"""
from cadgen import build123d as bd
from .bearing_revision.runtime import cached_shape
from .bearing_revision.collision_bridge_v2 import place, volume
from .head_bearings import YAW, base_adapter
from .bearing_revision.adapter_bridge_v3 import bearing_envelope, HORN_FRONT_Z
from .viewer_palette import paint_original

PITCH_X, PITCH_Z = 35.1, 235.769
ARM_RADIUS = 12.2
ARM_THICKNESS = 4.0
HORN_RADIAL_CLEARANCE = 0.20
DRIVEN_BRACE_DEPTH = 3.0
PASSIVE_BRACE_DEPTH = 1.8  # .25 mm axial gap to the raised vendor rear casing
# Original upper-pitch servo origin stays at native Y=17.6.
# The measured active horn face is -1.3 mm in its local frame; the vendor
# passive horn's outer mounting plane is local Z=37.25 (not yet measured).
DRIVEN_INNER_Y = 17.6 - HORN_FRONT_Z
PASSIVE_INNER_Y = 17.6 - 37.25
YAW_Z = 258.0048
BRIDGE_TOP = YAW_Z - 3.7
BRIDGE_BOTTOM = BRIDGE_TOP - 4.0
LAND_TOP = YAW_Z + HORN_FRONT_Z

@cached_shape
def native():
    import cadquery as cq
    from .cadquery_adapter import Model, to_build123d
    def arm(y0, y1):
        # Monotone tangent-continuous sides: no bulging shoulder or scallop.
        # R12.2 bottom contains the R12.2 internal horn-support rim.
        return (cq.Workplane('XZ', origin=(0,y1,0))
            .moveTo(PITCH_X-14,BRIDGE_TOP)
            .bezier([(PITCH_X-14,BRIDGE_TOP-6),(PITCH_X-ARM_RADIUS,PITCH_Z+6),
                     (PITCH_X-ARM_RADIUS,PITCH_Z)],includeCurrent=True)
            .threePointArc((PITCH_X,PITCH_Z-ARM_RADIUS),(PITCH_X+ARM_RADIUS,PITCH_Z))
            .bezier([(PITCH_X+ARM_RADIUS,PITCH_Z+6),(PITCH_X+14,BRIDGE_TOP-6),
                     (PITCH_X+14,BRIDGE_TOP)],includeCurrent=True)
            .close().extrude(y1-y0))
    y0 = PASSIVE_INNER_Y-ARM_THICKNESS
    y1 = DRIVEN_INNER_Y+ARM_THICKNESS
    # One shared exterior for arms, bridge, and cover. There are no wider
    # boxes left over outside the tapered arm silhouette.
    outer=arm(y0,y1)
    # Continuous cover with a shallow curved underside in the axial direction.
    # The full-width central roof remains above the rotating case envelope.
    lower=[(PASSIVE_INNER_Y,246.2),(-17.85,248.6),(-16.8,248.8),
           (0,248.97),(15.3,248.8),(16.2,248.6),(DRIVEN_INNER_Y,246.2)]
    canopy=(cq.Workplane('YZ',origin=(PITCH_X-14,0,0))
        .moveTo(PASSIVE_INNER_Y,BRIDGE_TOP).lineTo(*lower[0])
        .lineTo(-17.85,248.6).threePointArc((0,248.97),(16.2,248.6))
        .lineTo(DRIVEN_INNER_Y,246.2).lineTo(DRIVEN_INNER_Y,BRIDGE_TOP)
        .close().extrude(28))
    interior=cq.Workplane('XY').box(40,DRIVEN_INNER_Y-PASSIVE_INNER_Y,80).translate(
        (PITCH_X,(DRIVEN_INNER_Y+PASSIVE_INNER_Y)/2,PITCH_Z))
    body=outer.cut(interior.cut(canopy))
    # Relieve only the interior cover against the sampled servo motion envelope.
    # Horn mounting arms and their seating planes are outside this mask.
    from .bearing_revision.factories import servo
    pose=((17.6,-35.1,235.769),((.5773502691896257,-.5773502691896257,-.5773502691896258),119.99999999999999))
    hardware=servo()
    from shapely.geometry import MultiPoint, box as rectangle
    from shapely.affinity import rotate
    from shapely.ops import unary_union
    hw=place(place(hardware,pose),((0,0,0),('Z',-90)),inverse=True)
    vertices,_=hw.tessellate(.08,.1)
    profile=MultiPoint([(v.X-PITCH_X,v.Z-PITCH_Z) for v in vertices]).convex_hull
    # Conservative projection across the case width, buffered beyond tessellation
    # error, with a smooth print-scale approximation of the sampled sweep.
    sweep=unary_union([rotate(profile,a,origin=(0,0)) for a in range(-90,91,2)])
    sweep=sweep.buffer(.30).simplify(.025,preserve_topology=True).intersection(rectangle(-16,0,16,30))
    points=[(x+PITCH_X,z+PITCH_Z) for x,z in list(sweep.exterior.coords)[:-1]]
    cutter=cq.Workplane('XZ',origin=(0,16.2,0)).polyline(points).close().extrude(34.05)
    body=body.cut(cutter)
    for sign,edge in ((-1,y0),(1,y1)):
        triangle=[(edge,BRIDGE_TOP+1),(edge,BRIDGE_TOP-7),
                  (edge-sign*8,BRIDGE_TOP+1)]
        cutter=cq.Workplane('YZ',origin=(PITCH_X-20,0,0)).polyline(triangle).close().extrude(40)
        body=body.cut(cutter)
    # Compact round yaw horn land; the annular bearing clears the bridge by .2 mm.
    land = cq.Workplane('XY', origin=(PITCH_X,0,BRIDGE_TOP-.2)).circle(9.7).extrude(LAND_TOP-BRIDGE_TOP+.2)
    land = land.faces('>Z').edges().chamfer(.25)
    body = body.union(land)
    # Annotation 12: upper semicircular clearance seats around both horn rims.
    # Active rim is the measured 19.85 mm OD. Passive vendor rim is 19.2 mm,
    # with a 0.2 mm front chamfer, follow that contour with 0.2 mm radial assembly clearance.
    for face,direction,radius in ((DRIVEN_INNER_Y,-1,9.925),(PASSIVE_INNER_Y,1,9.6)):
        radius += HORN_RADIAL_CLEARANCE
        start=face-direction*.2
        axis=cq.Vector(0,direction,0)
        stock=cq.Solid.makeCylinder(12.2,2.0,cq.Vector(PITCH_X,start,PITCH_Z),axis)
        if direction==-1:
            void=cq.Solid.makeCylinder(radius,2.2,cq.Vector(PITCH_X,start-direction*.1,PITCH_Z),axis)
        else:
            front=cq.Solid.makeCylinder(9.4+HORN_RADIAL_CLEARANCE,.3,cq.Vector(PITCH_X,start-.1,PITCH_Z),axis)
            bevel=cq.Solid.makeCone(9.4+HORN_RADIAL_CLEARANCE,9.6+HORN_RADIAL_CLEARANCE,.2,cq.Vector(PITCH_X,face,PITCH_Z),axis)
            rim=cq.Solid.makeCylinder(9.6+HORN_RADIAL_CLEARANCE,1.7,cq.Vector(PITCH_X,face+.2,PITCH_Z),axis)
            void=front.fuse(bevel).fuse(rim)
        upper=cq.Workplane('XY').box(30,50,20).translate((PITCH_X,0,PITCH_Z+10)).val()
        # Feather the rim support into the arm over the first 3 mm of rise,
        # instead of ending it with a projecting rectangular tab.
        start=face-direction*.2
        end=face+direction*1.8
        transition=(cq.Workplane('YZ',origin=(PITCH_X-15,0,0))
            .polyline([(start,PITCH_Z),(face,PITCH_Z),(end,PITCH_Z+3),
                       (end,PITCH_Z+20),(start,PITCH_Z+20)]).close().extrude(30))
        # Keep both arches inset from the arm sides; remove the lateral tails.
        side_limit=cq.Workplane('XY').box(19.0,60,30).translate((PITCH_X,0,PITCH_Z+19.5)).val()
        body=body.union(stock.cut(void).intersect(upper).intersect(transition.val()).intersect(side_limit))
    # Bound every support by the same exterior. Keep the raised yaw land.
    land_guard=cq.Workplane('XY').box(40,60,10).translate((PITCH_X,0,BRIDGE_TOP+5))
    body=body.intersect(outer.union(land_guard))
    outer_rims=[e for e in body.val().Edges() if e.BoundingBox().ylen<1e-6
        and (abs(e.Center().y-y0)<1e-6 or abs(e.Center().y-y1)<1e-6)]
    body=body.newObject(outer_rims).fillet(.5)
    def bore_y(x,z,r,a,b):
        return cq.Workplane('XZ', origin=(0,b,0)).center(x,z).circle(r).extrude(b-a)
    # Matching clean through holes on both arms. No diagonal escape slots.
    for x,z in [(PITCH_X-7,PITCH_Z),(PITCH_X+7,PITCH_Z),(PITCH_X,PITCH_Z-7),(PITCH_X,PITCH_Z+7)]:
        body = body.cut(bore_y(x,z,1.7,y0-1,y1+1))
    body = body.cut(bore_y(PITCH_X,PITCH_Z,3.2,y0-1,y1+1))
    # Recess only the passive hub; round and contained within the arm.
    body = body.cut(bore_y(PITCH_X,PITCH_Z,4.6,PASSIVE_INNER_Y-1.15,PASSIVE_INNER_Y+.1))
    for x,y in [(PITCH_X-7,0),(PITCH_X+7,0),(PITCH_X,-7),(PITCH_X,7)]:
        body = body.cut(cq.Workplane('XY',origin=(x,y,BRIDGE_BOTTOM-1)).circle(1.7).extrude(12))
    body = body.cut(cq.Workplane('XY',origin=(PITCH_X,0,BRIDGE_BOTTOM-1)).circle(3.2).extrude(12))
    # Annotation 14: reuse the existing horn screw-well specification
    # (yaw_roll_link_native): diameter 6.8 mm with a 4 mm clamped land.
    for x,y in ((PITCH_X-7,0),(PITCH_X+7,0),(PITCH_X,-7),(PITCH_X,7)):
        well=cq.Workplane('XY',origin=(x,y,PITCH_Z)).circle(3.4).extrude(LAND_TOP-4-PITCH_Z)
        body=body.cut(well)
    model=Model();model.add('printed_body',body);model.material('pla',color='#8a8a90')
    s=to_build123d(model,'neck_pitch')
    assert s.is_valid and len(s.solids())==1
    return s

@cached_shape
def neck_pitch():
    s=place(native(),((0,0,0),('Z',-90)))
    s.label='neck_pitch_001_native'
    bearing=place(bearing_envelope(),YAW)
    adapter=place(base_adapter(),YAW)
    assert volume(s.intersect(bearing))<1e-5
    assert volume(s.intersect(adapter))<1e-5
    assert s.distance_to(bearing)>.1999
    return paint_original(s,'neck_pitch_001_native')
