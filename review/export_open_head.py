"""Checked test-print meshes for the open head review candidate; no canonical release.

Writes scratch/open-head-v1/print-candidate/: one 3MF and one STL per part and a manifest.
Run after review/open_head.py has passed:

    PYTHONPATH=src:. CADGEN_CACHE_DIR=.cadgen .venv/bin/python review/export_open_head.py

Parts keep their modeled orientation, moved so the lowest point is on Z 0. Print
orientation, supports and slicer settings are not chosen here.
"""
from pathlib import Path
import sys, json
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from cadgen import build123d as bd
from tools.print_export import write_export_3mf, write_gmsh_3mf, inspect_3mf, write_verified_stl, solid_volume
from lib import open_head as oh

OUT = ROOT / 'scratch/open-head-v1'
DEST = OUT / 'print-candidate'
MATERIAL = {'open_head_skeleton_native': 'black PLA or PETG', 'open_head_skin_native': 'translucent PLA'}


def run():
    report = json.loads((OUT / 'validation.json').read_text())
    assert report['passed'], 'review/open_head.py must pass before exporting'
    DEST.mkdir(parents=True, exist_ok=True)
    records = []
    for name, shape in (('open_head_skeleton_native', oh.make_skeleton()), ('open_head_skin_native', oh.make_skin())):
        b = shape.bounding_box()
        placed = bd.Part(shape.moved(bd.Location((-b.min.X, -b.min.Y, -b.min.Z))).wrapped)
        placed.label = name
        size = [round(v, 3) for v in (b.size.X, b.size.Y, b.size.Z)]
        assert all(a < c for a, c in zip(size, (256, 256, 256))), (name, size, 'printer envelope')
        target = DEST / f'{name}.3mf'
        try:
            receipt = write_export_3mf(placed, target)
        except ValueError as error:
            # same fallback as the other review exports: the independently checked Gmsh mesher
            print('ALTERNATE MESHER', name, str(error), flush=True)
            receipt = write_gmsh_3mf(placed, target, 0.35)
        mesh = inspect_3mf(target, allow_numerical_overlaps=True)
        # Thin walls: 0.2 % of the volume is about 2 um of surface offset. The adaptive
        # integral the canonical exporter uses does not converge on these trimmed spline
        # walls (it changes by 2 % with its tolerance), so the kernel's default integral
        # is the reference here, with a 0.5 % limit. Both values are recorded.
        reference = shape.volume
        adaptive = solid_volume(placed)
        error = abs(mesh['volume_mm3'] - reference) / reference
        assert error < 0.005, (name, 'mesh volume differs from CAD', error)
        for key in ('open_edges', 'nonmanifold_edges', 'inconsistent_winding_edges'):
            assert mesh[key] == 0, (name, key, mesh[key])
        mesh_size = [mesh['bounds_mm'][i + 3] - mesh['bounds_mm'][i] for i in range(3)]
        if True:
            assert all(abs(a - c) < 0.05 for a, c in zip(mesh_size, size)), (name, 'mesh bounds differ from CAD', mesh_size, size)
        stl = write_verified_stl(target, DEST / f'{name}.stl', mesh)
        records.append(dict(name=name, material_hint=MATERIAL[name], size_mm=size, cad_volume_mm3=round(reference, 1), cad_volume_adaptive_integral_mm3=round(adaptive, 1), mesh_size_mm=[round(v, 3) for v in mesh_size],
                            relative_cad_volume_error=error, mesh=mesh, stl=stl, tessellation=receipt))
        print('EXPORTED', name, size, 'triangles', stl.get('triangles'), 'volume error', round(error, 6), flush=True)
    (DEST / 'manifest.json').write_text(json.dumps({'review_candidate': True, 'revision': 'open-head-v1',
        'canonical_release': False, 'units': 'mm', 'parts': records}, indent=2) + '\n')
    print('COMPLETE')


if __name__ == '__main__':
    run()
