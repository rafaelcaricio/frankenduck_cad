"""Open development head, review candidate v1: build, measure and save.

Writes scratch/open-head-v1/: open.step (skeleton, skin off), closed.step (skin on),
the two parts alone, and validation.json. Run from the repository root:

    PYTHONPATH=src:. CADGEN_CACHE_DIR=.cadgen .venv/bin/python review/open_head.py

All values are millimetres at the modeled rest pose. The checks are local and nominal:
no head motion sweep is run here, and nothing is a physical fit or strength result.
"""
from pathlib import Path
import sys, json
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from importlib import import_module
from cadgen import step, build123d as bd
from lib.bearing_revision.collision_bridge_v2 import place
from lib.viewer_palette import apply_palette
from lib import open_head as oh

OUT = ROOT / 'scratch/open-head-v1'
AT = [[0, 0, 0], ['Z', -90]]
# Everything fastened to the head frame, plus the neck parts next to it.
HEAD = ['head_frame_r05', 'roll_front_bulkhead_r05', 'electronics_deck_r02', 'face_radxa_bracket_r02',
        'ubec_clamp_r02', 'bottom_head_shell_native', 'noenoeil_native', 'soft_mouth_top_native',
        'face_part_native', 'import_pcb_radxa_cubie_a7z', 'stemma3885', 'import_pcb_sph0645_mic',
        'mighty_camera', 'import_pcb_vl53l5cx_tof', 'import_pcb_ubec_henge_4a', 'import_pcb_bno055',
        'import_pcb_waveshare_adapter', 'servo_head_roll', 'servo_jaw_direct', 'jaw_r01',
        'jaw_soft_001_native', 'radxa_6530b', 'bearing_head_roll_adapter', 'bearing_6704_head_roll',
        'yaw_roll_link_r05', 'servo_head_yaw', 'bearing_head_yaw_adapter', 'bearing_6704_head_yaw',
        'neck_pitch_001_native']
MODULES = {'measured_tof': 'measured_tof_seated'}
TOL = 1e-5          # mm3, numerical overlap tolerance
# The only skin material inside the original shell wall is the two front tongues with
# their pads. These are the parts next to them.
TONGUE_NEIGHBOURS = ['face_part_native', 'face_radxa_bracket_r02', 'mighty_camera',
                     'import_pcb_radxa_cubie_a7z', 'radxa_6530b']


def context():
    data = json.loads((ROOT / 'source/assembly_data.json').read_text())
    out = []
    for i in data['instances']:
        if i['name'] in HEAD:
            module = import_module('parts.' + MODULES.get(i['component'], i['component']))
            s = place(module.make(), i['at']); s.label = i['name']; out.append(s)
    assert len(out) == len(HEAD), 'missing head occurrences'
    return out


_PARTS = []


def _assembly(label, with_skin):
    """Head parts with the skeleton, and the skin when asked; cadgen keeps the colours."""
    import copy
    if not _PARTS:
        _PARTS.extend(context())
    extra = [('open_head_skeleton', oh.make_skeleton(), '#1c1c1e')]
    if with_skin:
        extra.append(('open_head_skin', oh.make_skin(), '#99deef'))
    root = apply_palette(bd.Compound(children=[copy.copy(p) for p in _PARTS], label=label))
    placed = []
    for name, shape, colour in extra:
        s = place(shape, AT); s.label = name; s.color = bd.Color(colour); placed.append(s)
    return bd.Compound(children=[*root.children, *placed], label=label)


@step(out='../scratch/open-head-v1/open.step')
def open_head_open():
    return _assembly('open head v1, skin off', False)


@step(out='../scratch/open-head-v1/closed.step')
def open_head_closed():
    return _assembly('open head v1, skin on', True)


@step(out='../scratch/open-head-v1/open_head_skeleton.step')
def open_head_skeleton():
    return oh.make_skeleton()


@step(out='../scratch/open-head-v1/open_head_skin.step')
def open_head_skin():
    return oh.make_skin()


def volume(shape):
    return sum(s.volume for s in shape.solids()) if shape is not None else 0.0


def run():
    import time
    t0 = time.time()

    def log(text):
        print(f'OPEN_HEAD_STEP {time.time() - t0:6.1f}s {text}', flush=True)

    OUT.mkdir(parents=True, exist_ok=True)
    report, fail = {}, []

    def need(ok, text):
        if not ok:
            fail.append(text)

    lid, _ = oh._source()
    skeleton, skin = oh.make_skeleton(), oh.make_skin()
    body, tabs, bumps, tongues = oh._skin_parts()
    log('parts built')
    for name, s in (('skeleton', skeleton), ('skin', skin)):
        b = s.bounding_box()
        report[name] = {'valid': bool(s.is_valid), 'solids': len(s.solids()), 'volume_mm3': round(s.volume, 1),
                        'pla_g_solid': round(s.volume * 1.31e-3, 1),
                        'local_bounds': [round(v, 2) for v in (b.min.X, b.max.X, b.min.Y, b.max.Y, b.min.Z, b.max.Z)]}
        need(s.is_valid and len(s.solids()) == 1, f'{name} is not one valid solid')

    report['loose_fragments_removed'] = dict(oh.removed_debris)
    # The skeleton adds nothing to the original top head shell, so it is no closer to any
    # part, at any pose, than the accepted shell. No neighbour check is needed for it.
    removed = lid - skeleton
    added = volume(skeleton - lid)
    report['skeleton_outside_original_shell_mm3'] = round(added, 6)
    need(added < TOL, 'skeleton has material outside the original top head shell')
    log('skeleton inside original shell')

    # Face retention: the original shell is untouched in front of the band's rear edge.
    x_front = oh.BAND_X_HIGH + 0.5
    front = bd.Pos((x_front + 140.0) / 2, 0, 300.0) * bd.Box(140.0 - x_front, 200, 120)
    lost_front = volume(removed & front)
    report['face_band'] = {'original_shell_removed_in_front_of_x_mm3': round(lost_front, 6), 'x': x_front}
    need(lost_front < TOL, 'face slot region of the original shell was changed')

    # All four screw bosses are whole (self-forming M3 pilots, screws from below as before).
    bosses = {'front_left': (102.06, 52.65, 278.8, 296.0), 'front_right': (102.06, -52.65, 278.8, 296.0),
              'rear': (-30.0, -26.0, 277.2, 288.0), 'side': (-1.5, 43.5, 277.2, 290.0)}
    report['bosses'] = {}
    for name, (x, y, z0, z1) in bosses.items():
        probe = bd.Pos(x, y, (z0 + z1) / 2) * bd.Cylinder(4.3, z1 - z0)
        lost = volume(removed & probe)
        report['bosses'][name] = {'original_removed_mm3': round(lost, 6)}
        need(lost < TOL, f'{name} screw boss was cut')
    log('face band and bosses')

    # Skin against skeleton. The body is the outer layer outside the kept region grown by
    # the seam gap, so only the small added pieces are measured.
    pieces = {'body': [body], 'tab': tabs, 'bump': bumps, 'tongue': tongues}
    seated = {}
    for name, group in pieces.items():
        seated[name] = {'overlap_mm3': round(sum(volume(p & skeleton) for p in group), 6)}
        if name != 'body':   # the body's gaps are the nominal seam and lip gaps
            seated[name]['distance'] = round(min(p.distance_to(skeleton) for p in group), 3)
        log(f'seated {name}')
        need(seated[name]['overlap_mm3'] < TOL, f'seated skin {name} overlaps the skeleton')
    swing = {}
    for degrees in (1, 2, 3, 4, 5, 6):
        swing[str(degrees)] = {name: round(sum(volume(oh._swing(p, degrees) & skeleton) for p in group), 4)
                               for name, group in pieces.items() if name != 'body' or degrees in (2, 4)}
        if 'body' in swing[str(degrees)]:
            need(swing[str(degrees)]['body'] < 0.01, f'skin body meets the skeleton at {degrees} degrees of swing')
        log(f'swing {degrees}')
        need(swing[str(degrees)]['tab'] < TOL, f'tab meets the belt at {degrees} degrees of swing')
        if degrees <= 4:
            need(swing[str(degrees)]['tongue'] < 0.01, f'tongue binds under the band at {degrees} degrees of swing')
    report['skin_to_skeleton'] = {'seam_gap_nominal': oh.SEAM_GAP, 'lip_gap_nominal': oh.LIP_GAP, 'skin_wall': oh.SKIN_WALL, 'seated': seated, 'swing_overlap_mm3_by_degree': swing,
                                  'note': 'bump overlap during the swing is the snap riding over the belt wall; tongue overlap means the swing axis needs work'}
    log('skin to skeleton')

    parts = context()
    by_name = {p.label: p for p in parts}
    log('head parts built')
    tongues_w = [place(t, AT) for t in tongues]
    report['tongue_neighbours'] = {}
    for other in TONGUE_NEIGHBOURS:
        d = min(t.distance_to(by_name[other]) for t in tongues_w)
        report['tongue_neighbours'][other] = round(d, 3)
        need(d >= 1.0, f'tongue within 1 mm of {other}')
    log('tongue neighbours')

    report['passed'] = not fail
    report['failures'] = fail
    (OUT / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
    open_head_skeleton(); open_head_skin(); open_head_open(); open_head_closed()
    log('saved')
    print('OPEN_HEAD', json.dumps({'passed': report['passed'], 'failures': fail}))
    return report


if __name__ == '__main__':
    run()
