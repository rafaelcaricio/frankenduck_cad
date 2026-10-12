"""Shared CadQuery feature construction for Frankenduck parts.

Geometry is converted to build123d shapes for cadgen exports.
"""
from __future__ import annotations
import math
from pathlib import Path
import cadquery as cq
from cadgen import build123d as bd
from OCP.gp import gp_Pnt
from OCP.Geom import Geom_BSplineCurve
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.TColgp import TColgp_Array1OfPnt
from OCP.TColStd import TColStd_Array1OfReal, TColStd_Array1OfInteger

ROOT = Path(__file__).resolve().parents[2]
from lib.import_paths import IMPORTS


def solidify(value):
    # CadQuery shell() can return a compound containing a closed shell.
    # Promote closed shells to solids before boolean operations.
    if isinstance(value, cq.Shell) and value.Closed():
        return cq.Solid.makeSolid(value)
    if isinstance(value, cq.Compound):
        return cq.Compound.makeCompound([solidify(child) for child in value])
    return value


def shape(value):
    if isinstance(value, cq.Workplane):
        vals = [v for v in value.vals() if isinstance(v, cq.Shape)]
        return solidify(vals[0] if len(vals) == 1 else cq.Compound.makeCompound(vals))
    if isinstance(value, Model):
        return value.result()
    return solidify(value)


def to_build123d(value, label):
    if isinstance(value, bd.Shape):
        return value
    if isinstance(value, Model):
        children = []
        for key, body in value.bodies.items():
            item = bd.Compound(body.wrapped, label=key)
            color = value.appearance.get(key, value.appearance.get('*', {})).get('color')
            if color:
                item.color = bd.Color(color)
            children.append(item)
        return bd.Compound(children=children, label=label)
    return bd.Compound(shape(value).wrapped, label=label)


class Sketch:
    def __init__(self, plane):
        self.plane = plane
        self.curves = []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def point(self, xy):
        return self.plane.toWorldCoords(cq.Vector(*xy, 0))

    def line(self, id, start, end, construction=False):
        if not construction:
            self.curves.append(cq.Edge.makeLine(self.point(start), self.point(end)))

    def circle(self, id, center, radius, construction=False):
        if not construction:
            self.curves.append(cq.Edge.makeCircle(radius, self.point(center), self.plane.zDir))

    def arc(self, id, center, radius, start_angle, end_angle, construction=False):
        if construction:
            return
        # Build in XY first: OCCT chooses its own X direction for an arbitrary normal.
        edge = cq.Edge.makeCircle(radius, cq.Vector(*center, 0), cq.Vector(0, 0, 1), start_angle, end_angle)
        self.curves.append(edge.transformShape(self.plane.rG))

    def spline(self, id, poles, degree=3, periodic=False, construction=False):
        if construction:
            return
        if periodic:
            raise NotImplementedError('Periodic native sketch spline not used by the pinned source')
        degree = min(degree, len(poles) - 1)
        points = TColgp_Array1OfPnt(1, len(poles))
        for i, p in enumerate(poles, 1):
            v = self.point(p)
            points.SetValue(i, gp_Pnt(*v.toTuple()))
        count = len(poles) - degree + 1
        knots = TColStd_Array1OfReal(1, count)
        mult = TColStd_Array1OfInteger(1, count)
        for i in range(1, count + 1):
            knots.SetValue(i, (i - 1) / (count - 1))
            mult.SetValue(i, degree + 1 if i in (1, count) else 1)
        curve = Geom_BSplineCurve(points, knots, mult, degree, False)
        self.curves.append(cq.Edge(BRepBuilderAPI_MakeEdge(curve).Edge()))

    def wires(self):
        return cq.Wire.combine(self.curves, tol=1e-6)

    def faces(self):
        # CadQuery sorts nested loops and keeps holes rather than filling them.
        from cadquery.occ_impl.shapes import sortWiresByBuildOrder
        return [cq.Face.makeFromWires(group[0], group[1:]) for group in sortWiresByBuildOrder(self.wires())]


class Model:
    def __init__(self, name=None):
        self.bodies = {}
        self.current = None
        self.planes = {}
        self.sketches = {}
        self.axes = {}
        self.features = {}
        self.appearance = {}

    def add(self, id, value, op=None, body=None, **kw):
        value = shape(value)
        key = body or self.current or id
        op = op or ('join' if key in self.bodies else 'new')
        if op == 'new':
            self.bodies[key] = value
        elif op == 'join':
            self.bodies[key] = self.bodies[key].fuse(value).clean() if key in self.bodies else value
        elif op == 'cut':
            self.bodies[key] = self.bodies[key].cut(value).clean()
        elif op == 'intersect':
            self.bodies[key] = self.bodies[key].intersect(value).clean()
        else:
            raise ValueError(op)
        self.current = key
        self.features[id] = (key, value)

    def box(self, id, length, width, height, at=(0, 0, 0), centered=True, **kw):
        self.add(id, cq.Workplane('XY').box(length, width, height, centered=centered).translate(at), **kw)

    def cylinder(self, id, radius, height, at=(0, 0, 0), direction=(0, 0, 1), **kw):
        self.add(id, cq.Solid.makeCylinder(radius, height, cq.Vector(*at), cq.Vector(*direction)), **kw)

    def cone(self, id, radius1, radius2, height, at=(0, 0, 0), direction=(0, 0, 1), **kw):
        self.add(id, cq.Solid.makeCone(radius1, radius2, height, cq.Vector(*at), cq.Vector(*direction)), **kw)

    def plane(self, id, on='XY', offset=0, **kw):
        p = self.planes.get(on) or cq.Plane.named(on)
        self.planes[id] = cq.Plane(p.origin + p.zDir * offset, p.xDir, p.zDir)

    def sketch(self, id, on='XY', **kw):
        p = self.planes.get(on) or cq.Plane.named(on)
        self.sketches[id] = Sketch(p)
        return self.sketches[id]

    def extrude(self, id, profile, distance, both=False, **kw):
        sk = self.sketches[profile]
        solids = []
        for face in sk.faces():
            if both:
                face = face.translate(sk.plane.zDir * -distance)
            solids.append(cq.Solid.extrudeLinear(face.outerWire(), face.innerWires(), sk.plane.zDir * distance * (2 if both else 1)))
        self.add(id, cq.Compound.makeCompound(solids) if len(solids) > 1 else solids[0], **kw)

    def loft(self, id, sections, ruled=False, **kw):
        wires = [self.sketches[s].wires() for s in sections]
        if any(len(w) != 1 for w in wires):
            raise ValueError('The pinned lofts require one closed wire per section')
        self.add(id, cq.Solid.makeLoft([w[0] for w in wires], ruled=ruled), **kw)

    def axis(self, id, through, direction=(0, 0, 1), **kw):
        self.axes[id] = (cq.Vector(*through), cq.Vector(*direction))

    def revolve(self, id, profile, axis, angle=360, **kw):
        sk = self.sketches[profile]
        p, v = self.axes[axis]
        solids = [cq.Solid.revolve(f.outerWire(), f.innerWires(), angle, p, p + v) for f in sk.faces()]
        self.add(id, cq.Compound.makeCompound(solids) if len(solids) > 1 else solids[0], **kw)

    def circular_pattern(self, id, axis, count, angle=360, features=None, **kw):
        p, v = self.axes[axis]
        for f in features:
            key, tool = self.features[f]
            for i in range(1, count):
                self.add(f'{id}_{i}', tool.rotate(p.toTuple(), (p + v).toTuple(), i * angle / count), body=key, op='join')

    def combine(self, id, target, tools, op='join', keep_tools=False, **kw):
        for key in tools:
            self.add(id, self.bodies[key], op=op, body=target)
            if not keep_tools:
                del self.bodies[key]
        self.current = target

    def material(self, material, bodies=None, finish=None, color=None):
        for key in bodies or ['*']:
            self.appearance[key] = {'material': material, 'finish': finish, 'color': color}

    def import_step(self, id, source, **kw):
        path = ROOT / IMPORTS[source]
        imported = cq.importers.importStep(str(path))
        values = imported.solids().vals()
        for i, v in enumerate(values):
            self.add(id if len(values) == 1 else f'{id}_{i}', v, op='new', body=id if len(values) == 1 else f'{id}_{i}')

    def result(self):
        return cq.Compound.makeCompound(list(self.bodies.values()))


class Component(Model):
    def __init__(self, id, name=None):
        super().__init__(name)
        self.id = id
