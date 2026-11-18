"""Two internal M2 shell ties with recessed screw seats and blind forming pilots.

All layout coordinates below are installed world millimetres. The skin is
1 mm; the original exterior envelope trims every added boss. M2 x 8 mm screws
seat at x=-2 and engage 6 mm into the opposite shell. Pilot fit is provisional
for the same 1.6 mm printed M2 forming interface already used in this project.
"""
from cadgen import build123d as b
from .viewer_palette import paint_original

CENTERS=((32.3,170.3),(-24.0,140.0))  # (world Y,Z): upper rear under closed rim, lower front
BOSS_RADIUS=3.6
HEAD_BORE_D=4.6
SHANK_CLEARANCE_D=2.2
PILOT_D=1.6
HEAD_SEAT_X=-2.0
PILOT_END_X=7.5
RECEIVER_CORE_START_X=10.0

def cylinder(radius,x0,x1,y,z):
    return b.Solid.make_cylinder(radius,x1-x0,b.Plane(origin=(x0,y,z),z_dir=(1,0,0)))

def common(a,c):
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    op=BRepAlgoAPI_Common(a.wrapped,c.wrapped);op.Build()
    assert op.IsDone()
    return b.Compound(op.Shape())

# The lower/front junction has room for a broader blend. The upper fillet
# stays compact to preserve the adjacent top service opening.
FRONT_ROOT_FILLET = 1.5
UPPER_ROOT_FILLET = 0.8


def fillet_support_roots(shape):
    """Blend only the concave outer-tube junctions; leave bores and mating ends."""
    solid = shape.solids()[0]
    for upper, radius, expected_edges in (
        (False, FRONT_ROOT_FILLET, 4),
        (True, UPPER_ROOT_FILLET, 5),
    ):
        faces = list(solid.faces())
        selected = []
        for face in faces:
            if face.geom_type != b.GeomType.CYLINDER or face.radius is None:
                continue
            if abs(face.radius - BOSS_RADIUS) > 0.001:
                continue
            if (face.center().Z > 160) != upper:
                continue
            for edge in face.edges():
                bounds = edge.bounding_box()
                # Exclude the mating end and the skin's seam face, as well as
                # seam edges that have no second face (a cylinder's UV seam).
                if bounds.size.X < 2 and abs(edge.center().X) < 30:
                    continue
                adjacent = [other for other in faces
                            if not other.wrapped.IsSame(face.wrapped)
                            and any(edge.wrapped.IsSame(candidate.wrapped)
                                    for candidate in other.edges())]
                if not adjacent:
                    continue
                if any(edge.wrapped.IsSame(candidate.wrapped) for candidate in selected):
                    continue
                selected.append(edge)
        assert len(selected) == expected_edges, ('support root edges', upper, len(selected))
        solid = solid.fillet(radius, selected).clean()
        assert solid.is_valid and len(solid.solids()) == 1, ('support root fillet', upper)
    return solid


def add_fasteners(shell_native, exterior_native, side):
    shell=b.Compound(shell_native.wrapped).rotate(b.Axis.Z,-90)
    envelope=b.Compound(exterior_native.wrapped).rotate(b.Axis.Z,-90)
    stocks=[];cuts=[]
    for y,z in CENTERS:
        x0,x1=(-50.0,0.0) if side<0 else (0.0,50.0)
        stock=common(cylinder(BOSS_RADIUS,x0,x1,y,z),envelope)
        if side>0:
            # Core the support behind the blind pilot without cutting the outer skin.
            stock=stock-cylinder(HEAD_BORE_D/2,RECEIVER_CORE_START_X,55,y,z)
            cuts.append(cylinder(PILOT_D/2,-.05,PILOT_END_X,y,z))
            cuts.append(b.Solid.make_cone(1.1,PILOT_D/2,.3,b.Plane(origin=(0,y,z),z_dir=(1,0,0))))
        else:
            cuts.extend([cylinder(HEAD_BORE_D/2,-55,HEAD_SEAT_X,y,z),cylinder(SHANK_CLEARANCE_D/2,HEAD_SEAT_X-.01,.05,y,z)])
        stocks.append(stock)
    for stock in stocks:shell=shell.fuse(stock).clean()
    for cut in cuts:shell=(shell-cut).clean()
    shell=fillet_support_roots(shell)
    assert shell.is_valid and len(shell.solids())==1,('shell fasteners',side,len(shell.solids()))
    label='left_shell_native' if side>0 else 'right_shell_native'
    result=shell.rotate(b.Axis.Z,90);result.label=label
    return paint_original(result,label)
