"""Approved V2 hip links: 5.4 mm walls and a continuous raised reinforcing bar.

Bearings remain outside the flat faces. Both joint centres and screw patterns
are unchanged. Horn screw grip is 5.925 mm, 3.225 mm longer than before.
See docs/hip-links.md for the approved dimensions and validation limits.
"""
def make_native(side=-1):
    from lib.cadquery_adapter import to_build123d
    import math
    import cadquery as cq
    SIDE = side
    z0 = 87.55
    z1 = 113.2
    end_r = 12.825
    edge_r = 0.9
    ya_x = 5.4
    ya_z = 100.375
    yp0 = 42.675
    yp1 = 48.075
    y_horn = 51.975
    xa_y = 23.625
    x_horn = 30.375
    xp0 = 34.275
    xp1 = 39.675
    in_cx = 27.275
    in_cy = 35.675
    in_r = 7.0
    bolt_r = 7.0
    hole_r = 1.7
    rec_r = 3.2
    rec_d = 3.2
    cb_r = 3.4
    x_cb = 37.6
    y_cb = 44.75
    BIG = 200.0
    
    def arc_pt(c, r, ang):
        return (c[0] + r * math.cos(ang), c[1] + r * math.sin(ang))
    # Continuous constant-wall elbow: both circular arcs share the same
    # centre. This replaces the old relieved wall and its overlapping rib.
    L = (cq.Workplane('XY', origin=(0, 0, z0))
         .moveTo(-20, yp0).lineTo(in_cx, yp0)
         .threePointArc(arc_pt((in_cx,in_cy),in_r,math.pi/4),(xp0,in_cy))
         .lineTo(xp0,0).lineTo(xp1,0).lineTo(xp1,in_cy)
         .threePointArc(arc_pt((in_cx,in_cy),in_r+5.4,math.pi/4),(in_cx,yp1))
         .lineTo(-20,yp1).close().extrude(z1-z0))
    ystad = cq.Workplane('XZ', origin=(0, 80, 0)).moveTo(ya_x, z0).lineTo(BIG, z0).lineTo(BIG, z1).lineTo(ya_x, z1).threePointArc((ya_x - end_r, ya_z), (ya_x, z0)).close().extrude(100)
    xstad = cq.Workplane('YZ', origin=(-30, 0, 0)).moveTo(xa_y, z0).lineTo(BIG, z0).lineTo(BIG, z1).lineTo(xa_y, z1).threePointArc((xa_y - end_r, ya_z), (xa_y, z0)).close().extrude(100)
    body = L.intersect(ystad).intersect(xstad)
    
    body = body.edges().fillet(edge_r)
    # Restore the original proud strengthening bar: 2.7 mm above the
    # main wall, 13.41 mm broad crest, smoothly blended to the wall.
    # It wraps the elbow and ends before the pitch bearing's radial envelope.
    def rib_section(plane, origin, height=2.7, width_scale=1.0):
        base_half=10.205*width_scale
        crest_half=6.705*width_scale
        return (cq.Workplane(plane,origin=origin)
                .moveTo(-.25,-base_half).lineTo(0,-base_half)
                .bezier([(0,-(base_half+crest_half)/2),
                         (height,-(base_half+crest_half)/2),
                         (height,-crest_half)],includeCurrent=True)
                .lineTo(height,crest_half)
                .bezier([(height,(base_half+crest_half)/2),
                         (0,(base_half+crest_half)/2),
                         (0,base_half)],includeCurrent=True)
                .lineTo(-.25,base_half).close())
    outer_r=in_r+5.4
    path=(cq.Workplane('XY',origin=(0,0,ya_z)).moveTo(xp1,xa_y)
          .lineTo(xp1,in_cy)
          .threePointArc(arc_pt((in_cx,in_cy),outer_r,math.pi/4),(in_cx,yp1))
          .lineTo(23.625,yp1))
    rib=rib_section('XZ',(xp1,xa_y,ya_z)).sweep(path,isFrenet=False,transition='round')
    start=cq.Solid.makeLoft([rib_section('XZ',(xp1,y,ya_z),h,w).val()
                           for y,h,w in [(15.4,.015,.25),(17,.7,.55),(20,2.3,.9),(xa_y,2.7,1)]])
    end=cq.Solid.makeLoft([rib_section('YZ',(x,yp1,ya_z),h,w).val()
                         for x,h,w in [(23.625,2.7,1),(22.5,2.2,.9),(21.2,.7,.55),(20.4,.015,.25)]])
    body=body.union(rib).union(start).union(end)
    assert len(body.solids().vals())==1 and body.val().isValid()
    # Flat 5.4 mm walls; only the screw land and the measured inner-ring
    # contact rise toward the fixed horn and bearing. No dished bearing pocket.
    xpad = cq.Solid.makeCylinder(9.7, 2.7, cq.Vector(x_horn + 1.3, xa_y, ya_z), cq.Vector(1, 0, 0))
    ypad = cq.Solid.makeCylinder(9.7, 2.7, cq.Vector(ya_x, y_horn - 1.3, ya_z), cq.Vector(0, -1, 0))
    xlip = cq.Solid.makeCylinder(10.8, .5, cq.Vector(x_horn + 3.5, xa_y, ya_z), cq.Vector(1, 0, 0))
    ylip = cq.Solid.makeCylinder(10.8, .5, cq.Vector(ya_x, y_horn - 3.5, ya_z), cq.Vector(0, -1, 0))
    body = body.union(xpad).union(ypad).union(xlip).union(ylip)
    xpts = [(xa_y + bolt_r, ya_z), (xa_y - bolt_r, ya_z), (xa_y, ya_z + bolt_r), (xa_y, ya_z - bolt_r)]
    ypts = [(ya_x + bolt_r, ya_z), (ya_x - bolt_r, ya_z), (ya_x, ya_z + bolt_r), (ya_x, ya_z - bolt_r)]
    body = body.cut(cq.Workplane('YZ', origin=(x_horn + .3, 0, 0)).center(xa_y, ya_z).circle(rec_r).extrude(rec_d + 1))
    body = body.cut(cq.Workplane('YZ', origin=(x_horn - 1, 0, 0)).pushPoints(xpts).circle(hole_r).extrude(12))
    body = body.cut(cq.Workplane('YZ', origin=(x_cb, 0, 0)).pushPoints(xpts).circle(cb_r).extrude(8))
    body = body.cut(cq.Workplane('XZ', origin=(0, y_horn - .3, 0)).center(ya_x, ya_z).circle(rec_r).extrude(rec_d + 1))
    body = body.cut(cq.Workplane('XZ', origin=(0, y_horn + 1, 0)).pushPoints(ypts).circle(hole_r).extrude(10))
    body = body.cut(cq.Workplane('XZ', origin=(0, y_cb, 0)).pushPoints(ypts).circle(cb_r).extrude(5))
    if SIDE < 0:
        body = body.mirror('XZ')
    from lib.cadquery_adapter import Model
    model = Model()
    model.add('printed_body', body)
    model.material('pla', color='#8a8a90')
    result = model.result()
    return to_build123d(model, 'hip_left_native')

