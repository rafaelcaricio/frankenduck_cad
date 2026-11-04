"""Canonical head/neck fit and sampled motion checks."""
from pathlib import Path
R=Path(__file__).resolve().parents[1]

def fit():
    import sys,json
    from pathlib import Path
    from cadgen import build123d as b
    from lib.neck_pitch import neck_pitch,PITCH_X,PITCH_Z,ARM_RADIUS,BRIDGE_BOTTOM,BRIDGE_TOP,DRIVEN_INNER_Y,PASSIVE_INNER_Y,DRIVEN_BRACE_DEPTH,PASSIVE_BRACE_DEPTH
    from lib.head_bearings import YAW,base_adapter
    from lib.bearing_revision.collision_bridge_v2 import place,volume
    from lib.bearing_revision.adapter_bridge_v3 import bearing_envelope,cyl
    from lib.bearing_revision.factories import servo
    s=neck_pitch();local=place(s,((0,0,0),('Z',-90)),inverse=True)
    report={'valid':s.is_valid,'solids':len(s.solids()),'volume_mm3':volume(s),'sampled_arm_thicknesses_mm':[]}
    for y0,y1 in [(PASSIVE_INNER_Y-4,PASSIVE_INNER_Y),(DRIVEN_INNER_Y,DRIVEN_INNER_Y+4)]:
     for x,z in [(35.1+9.7,232.769),(35.1+9.7,231.769),(35.1+9,230.769)]:
      box=b.Box(.1,10,.1).translate((x,(y0+y1)/2,z))
      t=volume(local.intersect(box))/.01;report['sampled_arm_thicknesses_mm'].append(t);assert abs(t-4)<1e-4
    # Cover spans continuously across the gap; sample three interior axial stations.
    report['cover_sections_mm'] = []
    for y in (-15,0,14):
        t=volume(local.intersect(b.Box(.1,.1,15).translate((PITCH_X+5,y,250))))/.01
        assert t>3,(y,t)
        report['cover_sections_mm'].append([y,t])
    roof_sections=[]
    for x in range(-13,14):
        for y in (-14,14):
            t=volume(local.intersect(b.Box(.1,.1,25).translate((PITCH_X+x,y,250))))/.01
            roof_sections.append(t)
    assert min(roof_sections)>2, min(roof_sections)
    report['minimum_sampled_cover_thickness_mm']=min(roof_sections)
    # Matching recessed seats on all four yaw-horn holes.
    from lib.neck_pitch import LAND_TOP
    report['yaw_screw_wells'] = []
    for x,y in ((PITCH_X-7,0),(PITCH_X+7,0),(PITCH_X,-7),(PITCH_X,7)):
        gauge=cyl(3.399,PITCH_Z,LAND_TOP-4-.001).translate((x,y,0))
        assert volume(local.intersect(gauge))<1e-5
        # Probe beside the through bore: measure head-seat-to-horn grip.
        probe=b.Box(.05,.05,15).translate((x+2.4 if y else x,y if y else 2.4,251))
        grip=volume(local.intersect(probe))/.0025
        assert abs(grip-4)<1e-4,grip
        report['yaw_screw_wells'].append({'native_x_mm':x,'native_y_mm':y,'diameter_mm':6.8,'measured_grip_mm':grip})
    # Source-specified through holes stay fully open on both arms and the bridge.
    for x,z in [(PITCH_X-7,PITCH_Z),(PITCH_X+7,PITCH_Z),(PITCH_X,PITCH_Z-7),(PITCH_X,PITCH_Z+7),(PITCH_X,PITCH_Z)]:
     r=3.199 if x==PITCH_X and z==PITCH_Z else 1.699
     tool=b.Cylinder(r,70).rotate(b.Axis.X,90).translate((x,0,z));assert volume(local.intersect(tool))<1e-5
    for x,y in [(PITCH_X-7,0),(PITCH_X+7,0),(PITCH_X,-7),(PITCH_X,7),(PITCH_X,0)]:
     r=3.199 if (x,y)==(PITCH_X,0) else 1.699
     assert volume(local.intersect(cyl(r,BRIDGE_BOTTOM-1,260).translate((x,y,0))))<1e-5
    pose=((17.6,-35.1,235.769),((.5773502691896257,-.5773502691896257,-.5773502691896258),119.99999999999999))
    v=volume(s.intersect(place(servo(),pose)));report['upper_pitch_servo_overlap_mm3']=v
    assert v<.001,v
    from lib.bearing_revision.adapter_bridge_v3 import vendor_parts
    passive = next(p for p in vendor_parts() if '从动' in p.label)
    report['passive_horn_overlap_mm3'] = volume(s.intersect(place(passive,pose)))
    assert report['passive_horn_overlap_mm3']<.001
    report['yaw_bearing_clearance_mm']=s.distance_to(place(bearing_envelope(),YAW))
    fixed=place(s,YAW,inverse=True);hits=[]
    for angle in range(-170,171,10):
     v=volume(fixed.intersect(base_adapter().rotate(b.Axis.Z,angle)))
     if v>.001:hits.append([angle,v])
    assert not hits,hits
    report['yaw_adapter_sampled_hits']=hits;report['yaw_sample_step_deg']=10
    # Check that the closer internal contour does not intrude on the upper servo
    # while pitching, using the detailed casing and both horn envelopes.
    upper_local = place(s, pose, inverse=True)
    hardware = servo()
    pitch_hits=[]
    for angle in range(-90,91,10):
        moving=upper_local.rotate(b.Axis.Z,angle)
        ov=volume(moving.intersect(hardware))+volume(moving.intersect(passive))
        if ov>.001:pitch_hits.append([angle,ov])
    report['pitch_sampled_hits']=pitch_hits
    report['pitch_sample_step_deg']=10
    assert not pitch_hits,pitch_hits
    from lib.head_bearings import head_frame, yaw_roll_link
    from lib.head_bearings import ROLL
    frame=head_frame();link=yaw_roll_link();adapter=place(base_adapter(),ROLL)
    report['frame_valid']=frame.is_valid
    report['frame_solids']=len(frame.solids())
    report['adapter_frame_overlap_mm3']=volume(frame.intersect(adapter))
    report['frame_servo_overlap_mm3']=volume(frame.intersect(place(servo(),ROLL)))
    assert report['adapter_frame_overlap_mm3']<.001
    assert report['frame_servo_overlap_mm3']<.001
    # Widened cable opening, including its centre and both new side strips.
    for x in (-20.4,-15.95,-11.5):
        probe=b.Box(.1,10,10).translate((x,-35.1,295.2))
        assert volume(link.intersect(probe))<1e-5
    report['cable_window_mm']=[9.5,19]
    report['insertion_hits']=[]
    for dz in range(0,61,5):
        for name,part in [('servo',place(servo(),ROLL)),('adapter',adapter)]:
            ov=volume(frame.intersect(part.translate((0,0,dz))))
            if ov>.001:report['insertion_hits'].append([name,dz,ov])
    assert not report['insertion_hits'],report['insertion_hits']
    # Trimmed arches: no lower tails or material beyond the 19 mm lateral band.
    for y in (DRIVEN_INNER_Y-.8,PASSIVE_INNER_Y+.8):
        for dx in (-10.8,10.8):
            gauge=b.Box(2.5,.1,9).translate((PITCH_X+dx,y,PITCH_Z+4.5))
            assert volume(local.intersect(gauge))<1e-5
        gauge=b.Box(24,.1,4.49).translate((PITCH_X,y,PITCH_Z+2.245))
        assert volume(local.intersect(gauge))<1e-5
    report['arch_side_and_lower_tail_checks']='pass'
    # Measure actual cylindrical seat surfaces, independently of the input constants.
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    seat_radii=[]
    for face in local.faces():
        surface=BRepAdaptor_Surface(face.wrapped)
        if surface.GetType()==GeomAbs_Cylinder:
            cylinder=surface.Cylinder();r=cylinder.Radius()
            if 9<r<11:seat_radii.append(r)
    assert any(abs(r-10.125)<1e-6 for r in seat_radii)
    assert any(abs(r-9.8)<1e-6 for r in seat_radii)
    report['measured_horn_seat_radii_mm']=sorted(set(round(r,6) for r in seat_radii))
    # Mounting faces remain in contact; cylindrical rims have 0.2 mm radial clearance.
    from lib.bearing_revision.adapter_bridge_v3 import horn_envelope
    for name,horn in [('active',horn_envelope()),('passive',passive)]:
        placed=place(horn,pose)
        gap=s.distance_to(placed)
        assert gap<1e-6,(name,gap)
        # Upper rim points at 30, 60, 90, 120 and 150 degrees: verify stock
        # immediately outside the horn and no stock immediately inside it.
        import math
        radius=(9.925 if name=='active' else 9.6)+.2
        y=DRIVEN_INNER_Y-.8 if name=='active' else PASSIVE_INNER_Y+.8
        for angle in (30,60,90,120,150):
            for offset,expected in ((.05,1),(-.05,0),(-.19,0)):
                x=PITCH_X+(radius+offset)*math.cos(math.radians(angle))
                z=PITCH_Z+(radius+offset)*math.sin(math.radians(angle))
                probe=b.Box(.02,.02,.02).translate((x,y,z))
                fraction=volume(local.intersect(probe))/.000008
                assert abs(fraction-expected)<1e-3,(name,angle,offset,fraction)
        report[name+'_mounting_face_gap_mm']=gap
        report[name+'_radial_clearance_mm']=.2
        report[name+'_radial_clearance_probes']='pass'
    # Verify the widened frame opening and preserved through holes.
    for x in (-23.6,-20,-17.45,-11.3):
        for z in (272.5,280.58,288.5):
            assert volume(frame.intersect(b.Box(.1,14,.1).translate((x,23,z))))<1e-5
    for x,z in ((-8.3,270.31),(-8.3,290.85),(-32.74,270.32)):
        assert volume(frame.intersect(b.Cylinder(1.199,14).rotate(b.Axis.X,90).translate((x,23,z))))<1e-5
    report['frame_window_mm']=[12.5,19]
    roll_link=place(link,ROLL,inverse=True)
    report['roll_sample_hits']=[]
    for angle in range(-25,26,5):
        ov=volume(roll_link.intersect(base_adapter().rotate(b.Axis.Z,angle)))
        if ov>.001:report['roll_sample_hits'].append([angle,ov])
    assert not report['roll_sample_hits']
    assert roll_link.distance_to(bearing_envelope())>=.19999
    report['passed']=True
    report['coverage']='Local fit and insertion; pitch +/-90 and yaw +/-170 sampled at 10 degrees; roll +/-25 at 5 degrees. Not physical fit or full combined motion.'
    Path(R/'reports/latest/head_neck_verification.json').write_text(json.dumps(report,indent=2))
    return report
