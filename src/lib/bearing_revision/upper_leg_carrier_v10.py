"""V10 continuous carrier. Original assembly placements; shell translation only in exploded view."""
from pathlib import Path
import json
from cadgen import build123d as bd
from .runtime import cached_shape as model, read_step
from .collision_bridge_v2 import place, flatten, volume, styled
from .adapter_bridge_v3 import (
    adapter_print, bearing_envelope, servo_adapter_review, world_parts,
    COLLAR_FRONT, cyl,
)
from .leg_bearing_revision import left_leg

ROOT = Path(__file__).resolve().parents[3]
SERVOS = ('servo_upper_leg_left', 'servo_leg')

def instances():
    data = json.loads((ROOT/'source/bearing_revision_baseline.json').read_text())
    return {p['name']:p for p in data['instances']}

def imported(name, items, color):
    i=items[name]
    s=place(flatten(read_step(str(ROOT/'STEP/parts'/(i['component']+'.step')))),i['at'])
    s.label=name
    s.color=color
    for child in s.children:
        child.color=color
    return s

def servo_parts(items):
    vendor=[p.rotate(bd.Axis.X,180) for p in world_parts(servo_adapter_review())
            if p.label!='PRINTED front adapter' and '从动' not in p.label]
    for name in SERVOS:
        for p in vendor:
            q=place(p,items[name]['at'])
            q.label=f'{name} / {p.label}'
            yield q

def hardware(items):
    for name in SERVOS:
        bearing=place(bearing_envelope(),items[name]['at'])
        bearing.label=f'{name} / 6704 bearing'
        bearing.color='#468dd5'
        yield bearing

@model
def interior_reference():
    i=instances()
    shell=imported('upper_leg_left',i,'#268f82')
    servos=list(servo_parts(i))
    shell=shell.translate((46,0,0))
    shell.label='Upper-leg shell - moved aside to expose the inside'
    return bd.Compound(children=[shell,*servos,*hardware(i)],label='Left upper leg with shell opened')


def stock(x0,x1,y0,y1,z0,z1):
    return bd.Box(x1-x0,y1-y0,z1-z0,align=(bd.Align.MIN,)*3).translate((x0,y0,z0))

def shell_contact_area(carrier,shell):
    result=0.0
    # Rear wall nominal inner plane: X=95.175-1.35.
    for a in carrier.faces():
        if a.geom_type!=bd.GeomType.PLANE or abs(a.center().X-93.825)>1e-5 or a.normal_at().X<.99:
            continue
        for b in shell.faces():
            if b.geom_type!=bd.GeomType.PLANE or abs(b.center().X-93.825)>1e-5 or b.normal_at().X>-.99:
                continue
            common=a.intersect(b)
            if common:
                result+=sum(f.area for f in common.faces())
    return result

@model
def inner_carrier():
    i=instances()
    shell=imported('upper_leg_left',i,'#268f82')
    cases=list(servo_parts(i))
    adapter=adapter_print().translate((0,0,COLLAR_FRONT))
    seats=[place(adapter,i[name]['at']) for name in SERVOS]
    # A broad front web joins both adapter bodies. A deep diagonal gusset
    # joins the underside rails. Only the added structure is trimmed to the
    # conservative servo envelopes, retaining the original detailed seats.
    def prism(points,x0,x1):
        wire=bd.Wire.make_polygon([(x0,y,z) for y,z in points],close=True)
        return bd.extrude(bd.Face(wire),amount=x1-x0,dir=(1,0,0))
    # Common external tangent of the two existing R1 screw-ear corners.
    # The tangent removes both steps without moving holes or front datum.
    import math
    corner_a=(26.6,112.625)
    corner_b=(55.1491,102.675)
    dy=corner_b[0]-corner_a[0]; dz=corner_b[1]-corner_a[1]
    length=math.hypot(dy,dz)
    ny,nz=-dz/length,dy/length
    tangent_a=(corner_a[0]+ny,corner_a[1]+nz)
    tangent_b=(corner_b[0]+ny,corner_b[1]+nz)
    front_web=prism([(18,83),(18,104),tangent_a,tangent_b,
                     (54,90),(44,80),(32,83)],52.475,54.975)
    # Keep tangent ends exact; round the other connecting-web corners.
    round_edges=[e for e in front_web.edges().filter_by(bd.Axis.X)
                 if all(math.hypot(e.center().Y-y,e.center().Z-z)>.01
                        for y,z in (tangent_a,tangent_b))]
    front_web=bd.fillet(round_edges,3.0)
    deep_brace=prism([(22,86.5),(28,86.5),(47.5,59.9),
                      (47.5,56.8),(37.5,56.8),(22,78)],52.475,93.825)
    upper_rail=stock(53.0,93.825,22.0,28.0,83.1,86.5)
    knee_rail=stock(53.0,93.825,37.5,47.5,56.8,59.9)
    # Reinforce beside the shell rib, away from the narrow servo-side gap.
    rib_bypass=stock(80.0,93.825,25.287,28.287,66.0,78.0)
    rib_bypass=bd.fillet(rib_bypass.edges().filter_by(bd.Axis.X),1.0)
    additions=front_web.fuse(deep_brace,upper_rail,knee_rail,rib_bypass,tol=1e-5)
    # A 0.20 mm box allowance is conservative around each vendor casing.
    for part in cases:
        if 'horn' in part.label or '从动' in part.label:
            continue
        bb=part.bounding_box()
        additions=additions-stock(bb.min.X-.2,bb.max.X+.2,
            bb.min.Y-.2,bb.max.Y+.2,bb.min.Z-.2,bb.max.Z+.2)
    # Use the detailed casing for the upper edge, where its bounding box
    # would leave artificial rear notches. Keep a 0.20 mm axial allowance.
    top_band=prism([tangent_a,tangent_b,
        (tangent_b[0],tangent_b[1]-2),(tangent_a[0],tangent_a[1]-2)],52.475,54.975)
    for part in cases:
        top_band=top_band-part.translate((-.2,0,0))
    additions=additions.fuse(top_band,tol=1e-5)
    # Smooth lower connection, from the knee collar to the hip screw plate.
    curve_points=[(52.475,27.249768696609,70.819671202057),
                  (52.475,20,79),(52.475,12,87.125),
                  (52.475,2.928265125463,87.125)]
    outline=bd.Edge.make_bezier(*curve_points)
    upper_points=[curve_points[-1],(52.475,3,90),(52.475,29,90),
                  (52.475,29,70.819671202057),curve_points[0]]
    edges=[outline]+[bd.Edge.make_line(a,b) for a,b in zip(upper_points,upper_points[1:])]
    lower_web=bd.extrude(bd.Face(bd.Wire(edges)),amount=2.5,dir=(1,0,0))
    for part in cases:
        lower_web=lower_web-part.translate((-.2,0,0))
    additions=additions.fuse(lower_web,tol=1e-5)
    lower=place(left_leg(),i['servo_leg']['at'])
    additions=additions-shell-lower
    # Open the 0.0521 mm shell/servo sliver and its taper into a relief slot.
    # This deliberately leaves 0.2 mm clearance beside the shell rib.
    additions=additions-stock(84.275,93.826,28.487,32.3391,62.0,71.675)
    for name in SERVOS:
        additions=additions-place(cyl(12.7,-6,5),i[name]["at"])
    for bearing in hardware(i):
        additions=additions-bearing
    # Rear knee pin cradle: 6 mm pin, 0.30 mm radial running allowance.
    # Open above the pin for the user-specified top-down insertion.
    py,pz=42.8991,70.675
    def axial_cylinder(radius,x0,x1):
        return bd.Cylinder(radius,x1-x0,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).rotate(bd.Axis.Y,90).translate((x0,py,pz))
    # Curved saddle flares into the bottom rail and shares the rear wall
    # plane, replacing the former pad rather than covering it with stock.
    x0=87.475
    a=(x0,35.3991,pz); b=(x0,46.5,pz)
    c=(x0,47.5,59.9); d=(x0,43.0,56.8); e=(x0,37.5,56.8)
    outline=bd.Wire([
        bd.Edge.make_line(a,b),
        bd.Edge.make_bezier(b,(x0,46.5,66.0),(x0,48.1,62.0),c),
        bd.Edge.make_bezier(c,(x0,47.2,57.8),(x0,46.0,56.8),d),
        bd.Edge.make_line(d,e),bd.Edge.make_line(e,a)])
    cradle=bd.extrude(bd.Face(outline),amount=93.825-x0,dir=(1,0,0))
    channel=axial_cylinder(3.3,85,95)+stock(85,95,py,52,pz-3.3,pz+3.3)
    cradle=cradle-channel-stock(85,95,34,52,pz,79)
    for part in cases:
        cradle=cradle-part.translate((.3,0,0))
    cradle=cradle-shell
    assert volume(cradle)>10
    for shift in (0,.5,1,2,4,8,12,18):
        for part in cases:
            if part.label.startswith('servo_leg /'):
                assert volume(cradle.intersect(part.translate((0,0,shift))))<1e-5,(shift,part.label)
    print('PIN_CRADLE',{'pin_diameter_mm':6,'slot_diameter_mm':6.6,
          'radial_clearance_mm':.3,'top_insertion_samples':8,'volume_mm3':volume(cradle)})
    additions=additions.fuse(cradle,tol=1e-5)
    carrier=seats[0].fuse(seats[1],*additions.solids(),tol=1e-5).clean()
    pieces=sorted(carrier.solids(),key=lambda part:part.volume,reverse=True)
    print("PIECES",[(p.volume,list(p.bounding_box().min),list(p.bounding_box().max)) for p in pieces])
    # Trimming against the shell isolates a rear wedge (39.46 mm3).
    # Remove that unsupported offcut, never a seat or the continuous body.
    assert sum(p.volume for p in pieces[1:])<40
    assert all(p.bounding_box().min.X>89.9 for p in pieces[1:])
    carrier=pieces[0]
    for seat in seats:
        assert volume(seat-carrier)<1e-4
    # The tie crosses one original knee screw position. Recut all four
    # through holes and head seats after the structural union.
    for name in SERVOS:
        for x in (-10.25,10.25):
            bore=cyl(1.1,-6,3.21).translate((x,-29,0))
            head=bd.Cone(2,1.1,.9,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).translate((x,-29,.5))
            access=cyl(2,-6,.5).translate((x,-29,0))
            carrier=carrier-place(bore+head+access,i[name]['at'])
            assert volume(carrier.intersect(place(bore,i[name]['at'])))<1e-5
    carrier=carrier.clean()
    assert carrier.is_valid and len(carrier.solids())==1
    front_faces=[f for f in carrier.faces() if f.geom_type==bd.GeomType.PLANE
        and f.normal_at().X<-.99 and abs(f.center().X-52.475)<1e-5]
    # The clearance slot now opens through the diagonal lower edge, with
    # no thin shelf remaining below the former Z=64.5 termination.
    assert volume(carrier.intersect(stock(84.275,93.826,28.487,32.3391,62.0,64.5)))<1e-5
    # Verify the removed wafer and the 3 mm bypass at multiple stations.
    assert volume(carrier.intersect(stock(84.475,89.975,30.287,30.3391,66.67,69.675)))<1e-5
    for station_x in (85.0,89.0,92.0):
        for station_z in (67.0,68.0,69.0):
            gauge=stock(station_x,station_x+.01,25.287,28.287,station_z,station_z+.01)
            assert volume(gauge-carrier)<1e-7
    # Sample continuity through the existing case-relieved front thickness.
    for t in (.03,.25,.5,.75,.97):
        y=tangent_a[0]+t*dy-.08*ny
        z=tangent_a[1]+t*dz-.08*nz
        gauge=stock(52.475,54.975,y-.01,y+.01,z-.01,z+.01)
        assert volume(gauge-carrier)<1e-7
    # Confirm solid material immediately inside the curved boundary.
    for t in (.05,.25,.5,.75,.95):
        weights=((1-t)**3,3*(1-t)**2*t,3*(1-t)*t*t,t**3)
        y=sum(w*p[1] for w,p in zip(weights,curve_points))+.15
        z=sum(w*p[2] for w,p in zip(weights,curve_points))+.15
        gauge=stock(52.475,54.975,y-.01,y+.01,z-.01,z+.01)
        assert volume(gauge-carrier)<1e-7
    print('LOWER_CURVE_CHECK',{'depth_mm':2.5,'stations':5})
    print('TOP_CONNECTION',{'verified_continuous_depth_mm':2.5,'tangent_start':tangent_a,'tangent_end':tangent_b,'stations':5})
    print('RIB_BYPASS_CHECK' ,{'original_wafer_mm':.0520999,'bypass_section_mm':3.0,'sample_count':9})
    print('FLUSH_FRONT' ,[(f.area,tuple(f.center())) for f in front_faces])
    assert max(f.area for f in front_faces)>1100
    assert volume(carrier.intersect(lower))<1e-5
    overlaps={p.label:volume(carrier.intersect(p)) for p in cases}
    assert max(overlaps.values())<1e-4,overlaps
    assert volume(carrier.intersect(shell))<1e-4
    for bearing in hardware(i):
        assert volume(carrier.intersect(bearing))<1e-5
    area=shell_contact_area(carrier,shell)
    print('CARRIER_CHECKS',json.dumps({'one_solid':True,'rear_case_contact_mm2':area,
          'shell_gap_mm':carrier.distance_to(shell),'servo_overlap_mm3':max(overlaps.values()),
          'bounds_mm':list(carrier.bounding_box().size)}))
    carrier.label='V10 carrier - blended rear pin saddle'
    carrier.color='#f39432'
    return carrier

@model
def carrier_inside_review():
    i=instances()
    carrier=inner_carrier()
    shell=imported('upper_leg_left',i,'#268f82')
    # Explode by translation only. Keep the installed orientation explicit.
    shell=shell.translate((50,0,0))
    shell.label='Shell - translated 50 mm toward servo backs, no rotation'
    return bd.Compound(children=[carrier,*servo_parts(i),*hardware(i),shell],
                       label='V10 upper leg - shell translated only, horns face away from back wall')

@model
def carrier_installed_cutaway():
    i=instances()
    carrier=inner_carrier()
    shell=imported('upper_leg_left',i,'#268f82')
    # Remove part of the shell only. The carrier and two servos stay installed.
    window=stock(56,105,-35,70,83,130)
    shell_cut=styled(shell-window,'Shell, cut to expose carrier','#268f82')
    knee=i['servo_leg']
    lower=place(left_leg(),knee['at'])
    lower.label='Revised lower leg'
    hip=imported('hip_left',i,'#85aaab')
    hip_bearing=next(iter(hardware(i)))
    print('ADJACENT_LINKS',json.dumps({
        'hip_bearing_vs_existing_link_mm3':volume(hip_bearing.intersect(hip)),
        'carrier_vs_existing_hip_link_mm3':volume(carrier.intersect(hip)),
        'carrier_vs_revised_lower_leg_mm3':volume(carrier.intersect(lower)),
    }))
    return bd.Compound(children=[carrier,*servo_parts(i),*hardware(i),shell_cut,lower],
                       label='Upper leg installed cutaway - carrier fit study')

@model
def carrier_installed_complete():
    i=instances()
    carrier=inner_carrier()
    shell=imported('upper_leg_left',i,'#268f82')
    shell.label='Full upper-leg shell - installed position'
    return bd.Compound(children=[carrier,*servo_parts(i),*hardware(i),shell],
                       label='V10 upper leg - complete installed assembly')

