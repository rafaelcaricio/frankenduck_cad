"""Radxa 6530B dimensioned cooler representation, not vendor-exact CAD.

The official outside dimensions and PCB mounting grid are retained. Fin
spacing, fan casing and peripheral relief details are approximate. Check
the full outside envelope against the robot, and verify real GPIO/port
reliefs and fasteners physically. Manufacturer references are retained in
source/vendor/radxa_6530b/.
"""
from cadgen import step


def make():
    import cadquery as cq
    from cadgen import build123d as bd
    from lib.radxa_6530b_parameters import (WIDTH,SHORT_SIDE,HEIGHT,HOLE_GRID,
        BASE_BOTTOM,BASE_THICKNESS,FAN_CENTRE,FAN_OUTER)
    plate = cq.Workplane("XY").box(WIDTH,SHORT_SIDE,BASE_THICKNESS,
        centered=(True,True,False)).translate((0,0,BASE_BOTTOM)).val()
    # GPIO edge (+local Y) maps to the lower edge of the upright PCB.
    gpio = cq.Workplane("XY").box(53.0,7.0,HEIGHT+2,
        centered=(True,True,False)).translate((0,12.0,-1)).val()
    plate=plate.cut(gpio)
    for x in [-HOLE_GRID[0]/2,HOLE_GRID[0]/2]:
        for y in [-HOLE_GRID[1]/2,HOLE_GRID[1]/2]:
            post=cq.Solid.makeCylinder(2.4,BASE_BOTTOM,cq.Vector(x,y,0))
            plate=plate.fuse(post.cut(gpio))
    fan_pocket=cq.Workplane("XY").box(FAN_OUTER+.3,FAN_OUTER+.3,HEIGHT+2,
        centered=(True,True,False)).translate((*FAN_CENTRE,BASE_BOTTOM+BASE_THICKNESS)).val()
    for y in [-13.75,-11.0,-8.25,-5.5,-2.75,0,2.75,5.5,8.25,11.0,13.75]:
        fin=cq.Workplane("XY").box(WIDTH,.75,HEIGHT-BASE_BOTTOM-BASE_THICKNESS,
            centered=(True,True,False)).translate((0,y,BASE_BOTTOM+BASE_THICKNESS)).val()
        fin=fin.cut(gpio).cut(fan_pocket)
        if fin.Solids():plate=plate.fuse(fin)
    for x in [-HOLE_GRID[0]/2,HOLE_GRID[0]/2]:
        for y in [-HOLE_GRID[1]/2,HOLE_GRID[1]/2]:
            # Diameter 1.6 mm models an M2 tapped core, without helical threads.
            plate=plate.cut(cq.Solid.makeCylinder(.8,3.5,cq.Vector(x,y,-.01)))
    plate=plate.clean()
    fan_height=HEIGHT-BASE_BOTTOM-BASE_THICKNESS
    casing=cq.Workplane("XY").box(FAN_OUTER,FAN_OUTER,fan_height,
        centered=(True,True,False)).translate((*FAN_CENTRE,BASE_BOTTOM+BASE_THICKNESS)).val()
    casing=casing.cut(cq.Solid.makeCylinder(8.7,fan_height+2,
        cq.Vector(*FAN_CENTRE,BASE_BOTTOM+BASE_THICKNESS-1)))
    rotor=cq.Solid.makeCylinder(7.8,fan_height-.8,
        cq.Vector(*FAN_CENTRE,BASE_BOTTOM+BASE_THICKNESS+.3))
    # Pad location follows the large SoC face of the existing imported PCB.
    # Local -Y points upwards in the assembled board.
    pad=cq.Workplane("XY").box(16,16,1.5,centered=(True,True,False)).translate(
        (-4.375,-2.75,3.5)).val()
    children=[]
    for shape,label,color in [(plate,"6530b_aluminium_approx","#2b3037"),
        (casing,"6530b_fan_case_approx","#ccccd0"),
        (rotor,"6530b_fan_rotor_approx","#1f242a"),
        (pad,"6530b_thermal_pad_16x16x1_5","#7fab76")]:
        result=bd.Compound(shape.wrapped) if isinstance(shape,cq.Compound) else bd.Solid(shape.wrapped)
        result.label=label;result.color=bd.Color(color);children.append(result)
    result=bd.Compound(children=children);result.label="radxa_6530b_dimensioned_representation"
    return result


@step(out="../../STEP/parts/radxa_6530b.step")
def radxa_6530b():
    return make()


if __name__ == "__main__":
    radxa_6530b()
