from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from cadgen import step,build123d as bd
from importlib import import_module
from lib.bearing_revision.collision_bridge_v2 import place
from lib.viewer_palette import apply_palette
NAMES={'neck_pitch_001_native','head_frame_r05','yaw_roll_link_r05','roll_front_bulkhead_r05','servo_neck_upper','servo_head_yaw','servo_head_roll','import_horn_upper_passive','bearing_head_yaw_adapter','bearing_head_roll_adapter','bearing_6704_head_yaw','bearing_6704_head_roll'}
@step(out='../STEP/head_assembly.step')
def head_assembly():
    data=json.loads((ROOT/'source/assembly_data.json').read_text())
    parts=[]
    for i in data['instances']:
        if i['name'] in NAMES:
            s=place(import_module('parts.'+i['component']).make(),i['at']);s.label=i['name'];parts.append(s)
    return apply_palette(bd.Compound(children=parts,label='Canonical head and neck'))
if __name__=='__main__':head_assembly()
