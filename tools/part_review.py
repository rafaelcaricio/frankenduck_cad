"""Pictures and exact sections of one printed part for review after a change.

Views are cadgen snapshots of the part's saved STEP. Sections are exact B-rep
cuts drawn with matplotlib: material is filled, the outline is black, and open
quality findings that cross the plane are marked in red (a cross at their worst
point, and their box when it is small), accepted ones in orange. Look at every picture and section before calling a geometry
change complete.
"""
from pathlib import Path
import json
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PLANES = {'YZ': ((1, 0, 0), (1, 2)), 'XZ': ((0, 1, 0), (0, 2)), 'XY': ((0, 0, 1), (0, 1))}
AXES = 'XYZ'


def section(shape, plane, offset):
    """Exact cut of the part on a principal plane: filled triangles and outline polylines in plane coordinates.

    The part is intersected with the half-space below the plane; its faces lying
    in the plane are the section, meshed for filling. The cut from above must give
    the same section area: a part face lying in the plane, or a face split on it,
    makes the two cuts disagree, and the picture would show a partial section.
    """
    import cadquery as cq
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopLoc import TopLoc_Location
    normal, (i, j) = PLANES[plane]
    k = 3 - i - j
    part = cq.Shape.cast(getattr(shape, 'wrapped', shape))
    b = part.BoundingBox()
    lo = [b.xmin - 1, b.ymin - 1, b.zmin - 1]
    hi = [b.xmax + 1, b.ymax + 1, b.zmax + 1]
    if not lo[k] < offset < hi[k]:
        return [], []

    def plane_faces(below):
        box_lo, box_hi = list(lo), list(hi)
        if below:
            box_hi[k] = offset
        else:
            box_lo[k] = offset
        half = cq.Solid.makeBox(*(box_hi[n] - box_lo[n] for n in range(3)), cq.Vector(*box_lo))
        faces = []
        for face in part.intersect(half).Faces():
            fb = face.BoundingBox()
            if face.geomType() == 'PLANE' and abs((fb.xmin, fb.ymin, fb.zmin)[k] - offset) <= 1e-6 and \
                    abs((fb.xmax, fb.ymax, fb.zmax)[k] - offset) <= 1e-6:
                faces.append(face)
        return faces

    faces = plane_faces(True)
    below_area = sum(face.Area() for face in faces)
    above_area = sum(face.Area() for face in plane_faces(False))
    assert abs(below_area - above_area) <= 1e-3 * max(below_area, above_area, 1.0), \
        f'section {plane} at {offset:g} mm is {below_area:.2f} mm2 from below but {above_area:.2f} mm2 ' \
        f'from above; move the plane off the part face lying on it'
    triangles, outlines = [], []
    for face in faces:
        BRepMesh_IncrementalMesh(face.wrapped, 0.01, False, 0.1, True)
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face.wrapped, location)
        nodes = [mesh.Node(n).Transformed(location.Transformation()).Coord() for n in range(1, mesh.NbNodes() + 1)]
        for n in range(1, mesh.NbTriangles() + 1):
            triangles.append([(nodes[m - 1][i], nodes[m - 1][j]) for m in mesh.Triangle(n).Get()])
        for edge in face.Edges():
            # Plain parameter sampling: the deflection samplers return only the end points of some section B-splines.
            curve = BRepAdaptor_Curve(edge.wrapped)
            count = max(2, int(edge.Length() / 0.1) + 1)
            params = np.linspace(curve.FirstParameter(), curve.LastParameter(), count)
            outlines.append([(curve.Value(u).Coord()[i], curve.Value(u).Coord()[j]) for u in params])
    return triangles, outlines


def draw_section(shape, plane, offset, path, findings=(), title='', limits=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Rectangle
    normal, (i, j) = PLANES[plane]
    k = 3 - i - j
    triangles, outlines = section(shape, plane, offset)
    # A plane through the part that yields no section is a failed cut, not an empty picture.
    assert triangles, f'{title} section {plane} at {offset:g} mm is empty; move the plane off a face boundary'
    fig, ax = plt.subplots(figsize=(9, 9), dpi=130)
    ax.add_collection(PolyCollection(triangles, facecolors='#99deef', edgecolors='none', zorder=1))
    for line in outlines:
        xs, ys = zip(*line)
        ax.plot(xs, ys, color='black', lw=0.6, zorder=3)
    shown = 0
    for f in findings:
        b = f['box']
        if b[k] - 0.25 <= offset <= b[k + 3] + 0.25:
            color = 'orange' if f.get('accepted_as') else 'red'
            if 'at' in f and f['at'] is not None:
                ax.plot(f['at'][i], f['at'][j], 'x', color=color, ms=7, mew=1.5, zorder=5)
            if max(b[i + 3] - b[i], b[j + 3] - b[j]) <= 15:
                ax.add_patch(Rectangle((b[i], b[j]), max(b[i + 3] - b[i], 0.2), max(b[j + 3] - b[j], 0.2),
                                       fill=False, ec=color, lw=1.0, zorder=4))
            shown += 1
    if limits:
        ax.set_xlim(*limits[0])
        ax.set_ylim(*limits[1])
    else:
        ax.autoscale_view()
    ax.set_aspect('equal')
    ax.grid(True, lw=0.3, color='#bbbbbb')
    ax.set_xlabel(f'{AXES[i]} (mm)')
    ax.set_ylabel(f'{AXES[j]} (mm)')
    ax.set_title(f'{title} section {plane} at {AXES[k]} = {offset:g} mm; {shown} findings (red open, orange accepted)', fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def views_job(step, out, target, size):
    """Six orthographic views of a part STEP."""
    directions = dict(front34=(-1, -1, 0.8), rear34=(1, 1, 0.8), side_px=(1, 0, 0), side_nx=(-1, 0, 0),
                      top=(0, 0.001, 1), bottom=(0, 0.001, -1), front=(0, -1, 0), back=(0, 1, 0))
    outputs = []
    for name, d in directions.items():
        d = np.array(d, float) / np.linalg.norm(d)
        outputs.append(dict(path=str(out / f'view_{name}.png'), camera=dict(
            projection='orthographic', position=(np.array(target) + d * 600).tolist(), target=list(target),
            up=[0, 0, 1] if abs(d[2]) < 0.9 else [0, 1, 0], orthographicHalfHeight=size)))
    return dict(input=step, mode='view', display=dict(mode='shaded_edges'), output=dict(sizeProfile='simple', padding=0.05),
                outputs=outputs)


def review_holder():
    """Views and exact sections of the Zeee 2S holder, with its quality findings drawn in."""
    import cadquery as cq
    sys.path[:0] = [str(ROOT / 'src'), str(ROOT)]
    from lib import lipo_shoulders as ls, lipo_mount as lm
    out = ROOT / 'review/lipo_pod_quality'
    out.mkdir(parents=True, exist_ok=True)
    step = 'STEP/parts/lipo_pod_native.step'
    shape = cq.importers.importStep(str(ROOT / step)).val()
    report_path = ROOT / 'reports/latest/lipo_pod_quality.json'
    findings = []
    if report_path.exists():
        report = json.loads(report_path.read_text())
        for key in ('geometry', 'wall', 'symmetry', 'edges', 'voids', 'holes'):
            findings += report.get(key, {}).get('findings', [])
    b = shape.BoundingBox()
    centre = [(b.xmin + b.xmax) / 2, (b.ymin + b.ymax) / 2, (b.zmin + b.zmax) / 2]
    job = out / 'views_job.json'
    job.write_text(json.dumps(dict(jobs=[views_job(step, out, centre, 50)]), indent=1))
    subprocess.run([str(ROOT / '.venv/bin/cadgen'), 'step', 'snapshot', '--job', str(job)], cwd=ROOT, check=True)
    job.unlink()
    # Centre, arm screw and shoulder planes; front panel, pack centre and rear wall; floor, arms,
    # rim, shoulders and roof. The pack-centre cut is 0.25 mm off Y = 48.5 mm, where the holder's
    # faces are split and the exact cut returns no section face. The outer screw cut is 0.25 mm
    # off X = 25 mm, where the shoulder's inner face lies in the plane.
    planes = [('YZ', 0.0), ('YZ', lm.SCREWS[2][0]), ('YZ', lm.SCREWS[3][0] + 0.25), ('YZ', 20.5), ('YZ', 27.0),
              ('XZ', lm.SCREWS[0][1]), ('XZ', ls.COWL_REAR_Y - ls.COWL_WALL / 2), ('XZ', ls.PACK_CENTER_Y + 0.25), ('XZ', 60.0),
              ('XY', 91.0), ('XY', 110.0), ('XY', 150.0), ('XY', 154.0), ('XY', 157.5), ('XY', 162.0), ('XY', 168.0)]
    for plane, offset in planes:
        draw_section(shape, plane, offset, out / f'section_{plane}_{offset:g}.png', findings, 'lipo_pod')
    print(f'Pictures: {out}')


if __name__ == '__main__':
    review_holder()
