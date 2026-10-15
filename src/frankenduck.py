from __future__ import annotations
from cadgen import step, declare_input, build123d as bd
import math
import json
from pathlib import Path
from parts.trunk_base_native import trunk_base_native
from parts.stick_pod_native import stick_pod_native
from parts.head_frame_native import head_frame_native
from parts.roll_front_bulkhead_native import roll_front_bulkhead_native
from parts.yaw_roll_link_native import yaw_roll_link_native
from parts.electronics_deck_native import electronics_deck_native
from parts.face_radxa_bracket_native import face_radxa_bracket_native
from parts.ubec_clamp_native import ubec_clamp_native
from parts.jaw_native import jaw_native
from parts.bottom_head_shell_native import bottom_head_shell_native
from parts.top_head_shell_native import top_head_shell_native
from parts.noenoeil_native import noenoeil_native
from parts.soft_mouth_top_native import soft_mouth_top_native
from parts.jaw_soft_001_native import jaw_soft_001_native
from parts.face_part_native import face_part_native
from parts.import_pcb_radxa_cubie_a7z import import_pcb_radxa_cubie_a7z
from parts.radxa_6530b import radxa_6530b
from lib.radxa_6530b_parameters import HEATSINK_AT
from parts.stemma3885 import stemma3885
from parts.import_pcb_sph0645_mic import import_pcb_sph0645_mic
from parts.mighty_camera import mighty_camera
from parts.measured_tof_seated import measured_tof_seated
from parts.import_pcb_ubec_henge_4a import import_pcb_ubec_henge_4a
from parts.import_pcb_bno055 import import_pcb_bno055
from parts.import_pcb_waveshare_adapter import import_pcb_waveshare_adapter
from parts.neck_pitch_001_native import neck_pitch_001_native
from parts.neck_001_native import neck_001_native
from parts.neck_002_native import neck_002_native
from parts.neck_internal_yoke_native import neck_internal_yoke_native
from parts.import_horn_upper_passive import import_horn_upper_passive
from parts.import_horn_lower_passive import import_horn_lower_passive
from parts.imu_yoke_bridge_native import imu_yoke_bridge_native
from parts.power_switch_kcd1_native import power_switch_kcd1_native
from parts.mpu6050_trunk_native import mpu6050_trunk_native
from parts.left_shell_native import left_shell_native
from parts.right_shell_native import right_shell_native
from parts.hip_left_native import hip_left_native
from parts.yaw2roll_left_native import yaw2roll_left_native
from parts.upper_leg_left_native import upper_leg_left_native
from parts.leg_left_native import leg_left_native
from parts.ankle_left_native import ankle_left_native
from parts.ankle_eye_left_native import ankle_eye_left_native
from parts.ankle_bearing_outer_keeper_left_native import ankle_bearing_outer_keeper_left_native
from parts.ankle_bearing_inner_cap_left_native import ankle_bearing_inner_cap_left_native
from parts.sole_left_native import sole_left_native
from parts.foot_left_native_a import foot_left_native_a
from parts.foot_left_native_b import foot_left_native_b
from parts.hip_left_native_right import hip_left_native_right
from parts.yaw2roll_left_native_right import yaw2roll_left_native_right
from parts.upper_leg_left_native_right import upper_leg_left_native_right
from parts.leg_left_native_right import leg_left_native_right
from parts.ankle_left_native_right import ankle_left_native_right
from parts.ankle_eye_left_native_right import ankle_eye_left_native_right
from parts.ankle_bearing_outer_keeper_left_native_right import ankle_bearing_outer_keeper_left_native_right
from parts.ankle_bearing_inner_cap_left_native_right import ankle_bearing_inner_cap_left_native_right
from parts.sole_left_native_right import sole_left_native_right
from parts.foot_left_native_a_right import foot_left_native_a_right
from parts.foot_left_native_b_right import foot_left_native_b_right
from parts.sts3215_servo_native import sts3215_servo_native
from parts.parkside_papk_12_a4 import parkside_papk_12_a4
from lib.stick_pod import STICK_AT

ROOT = Path(__file__).resolve().parents[1]
ASSEMBLY_PATH = ROOT / "source/assembly_data.json"
ANIMATION_PATH = Path(__file__).with_name("animation.js")
ASSEMBLY = json.loads(ASSEMBLY_PATH.read_text())
KINEMATICS = {key: ASSEMBLY[key] for key in ("mates", "poses")}
ANIMATION = ANIMATION_PATH.read_text()

# Radxa support and PCB placement, with USB-C facing up.
RADXA_BRACKET_AT = ((-0.25, -169.015, 0.0), ('Z', 90.0))
RADXA_BOARD_AT = ((-12.00664031983, -170.214862460107, 604.45012907315), ((1, 1, 0), 180.0))



def place(part, name, at):
    from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir, gp_Vec
    trsf=gp_Trsf()
    if at:
        translation, (axis, degrees)=at
        axis={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}.get(axis,axis) if isinstance(axis,str) else axis
        trsf.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(*axis)),math.radians(degrees))
        trsf.SetTranslationPart(gp_Vec(*translation))
    result=part.moved(bd.Location(trsf))
    result.label=name
    return result

@step(out="../STEP/frankenduck.step", kinematics=KINEMATICS, animation=ANIMATION)
def frankenduck():
    declare_input(ASSEMBLY_PATH)
    declare_input(ANIMATION_PATH)
    children=[]
    children.append(place(trunk_base_native(), 'trunk_base', ((0, 0, 0), ('Z', -90))))
    children.append(place(head_frame_native(), 'head_frame_r05', ((0, 0, 0), ('Z', -90))))
    children.append(place(roll_front_bulkhead_native(), 'roll_front_bulkhead_r05', ((0, 0, 0), ('Z', -90))))
    children.append(place(yaw_roll_link_native(), 'yaw_roll_link_r05', ((0, 0, 0), ('Z', -90))))
    children.append(place(electronics_deck_native(), 'electronics_deck_r02', ((0, 0, 0), ('Z', -90))))
    children.append(place(face_radxa_bracket_native(), 'face_radxa_bracket_r02', RADXA_BRACKET_AT))
    children.append(place(ubec_clamp_native(), 'ubec_clamp_r02', ((0, 0, 0), ('Z', -90))))
    children.append(place(jaw_native(), 'jaw_r01', ((0, 0, 0), ('Z', -90))))
    children.append(place(bottom_head_shell_native(), 'bottom_head_shell_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(top_head_shell_native(), 'top_head_shell_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(noenoeil_native(), 'noenoeil_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(soft_mouth_top_native(), 'soft_mouth_top_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(jaw_soft_001_native(), 'jaw_soft_001_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(face_part_native(), 'face_part_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(import_pcb_radxa_cubie_a7z(), 'import_pcb_radxa_cubie_a7z', RADXA_BOARD_AT))
    children.append(place(stemma3885(), 'stemma3885', ((-40.0, -60.0, 318.8), ((1, 1, -1), 120))))
    children.append(place(import_pcb_sph0645_mic(), 'import_pcb_sph0645_mic', ((0, 0, 0), ('Z', -90))))
    children.append(place(mighty_camera(), 'mighty_camera', ((-16.62336, -98.2, 286.00477), ('X', 90))))
    children.append(place(measured_tof_seated(), 'import_pcb_vl53l5cx_tof', ((0, 0, 0), ('Z', -90))))
    children.append(place(import_pcb_ubec_henge_4a(), 'import_pcb_ubec_henge_4a', ((0, 0, 0), ('Z', -90))))
    children.append(place(import_pcb_bno055(), 'import_pcb_bno055', ((0, 0, 0), ('Z', -90))))
    children.append(place(import_pcb_waveshare_adapter(), 'import_pcb_waveshare_adapter', ((0, 0, 0), ('Z', -90))))
    children.append(place(neck_pitch_001_native(), 'neck_pitch_001_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(neck_001_native(), 'neck_001_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(neck_002_native(), 'neck_002_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(neck_internal_yoke_native(), 'import_neck_internal_yoke', ((0, 0, 0), ('Z', -90))))
    children.append(place(import_horn_upper_passive(), 'import_horn_upper_passive', ((0, 0, 0), ('Z', -90))))
    children.append(place(import_horn_lower_passive(), 'import_horn_lower_passive', ((0, 0, 0), ('Z', -90))))
    children.append(place(imu_yoke_bridge_native(), 'imu_yoke_bridge_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(power_switch_kcd1_native(), 'power_switch_kcd1_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(mpu6050_trunk_native(), 'mpu6050_trunk_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(left_shell_native(), 'left_shell_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(right_shell_native(), 'right_shell_native', ((0, 0, 0), ('Z', -90))))
    children.append(place(hip_left_native(), 'hip_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(yaw2roll_left_native(), 'yaw2roll_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(upper_leg_left_native(), 'upper_leg_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(leg_left_native(), 'leg_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(ankle_left_native(), 'ankle_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(ankle_eye_left_native(), 'ankle_eye_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(ankle_bearing_outer_keeper_left_native(), 'ankle_bearing_outer_keeper_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(ankle_bearing_inner_cap_left_native(), 'ankle_bearing_inner_cap_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(sole_left_native(), 'sole_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(foot_left_native_a(), 'foot_a_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(foot_left_native_b(), 'foot_b_left', ((0, 0, 0), ('Z', -90))))
    children.append(place(hip_left_native_right(), 'hip_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(yaw2roll_left_native_right(), 'yaw2roll_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(upper_leg_left_native_right(), 'upper_leg_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(leg_left_native_right(), 'leg_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(ankle_left_native_right(), 'ankle_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(ankle_eye_left_native_right(), 'ankle_eye_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(ankle_bearing_outer_keeper_left_native_right(), 'ankle_bearing_outer_keeper_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(ankle_bearing_inner_cap_left_native_right(), 'ankle_bearing_inner_cap_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(sole_left_native_right(), 'sole_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(foot_left_native_a_right(), 'foot_a_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(foot_left_native_b_right(), 'foot_b_right', ((0, 0, 0), ('Z', -90))))
    children.append(place(sts3215_servo_native(), 'servo_hip_l', ((23.625, -30.375, 100.375), ((0.0, -0.7071067811865475, -0.7071067811865476), 180.0))))
    children.append(place(sts3215_servo_native(), 'servo_hip_l_2', ((-23.625, -30.375, 100.375), ((0.0, -0.7071067811865475, -0.7071067811865476), 180.0))))
    children.append(place(sts3215_servo_native(), 'servo_yaw2roll', ((23.625, -8.1, 120.25), ((0.0, 0.0, -1.0), 180.0))))
    children.append(place(sts3215_servo_native(), 'servo_bearing_roll', ((-23.625, -8.1, 120.25), ((0.0, 0.0, -1.0), 180.0))))
    children.append(place(sts3215_servo_native(), 'servo_leg', ((51.975, 42.8991, 70.675), ((-0.5773502691896257, 0.5773502691896257, -0.5773502691896258), 119.99999999999999))))
    children.append(place(sts3215_servo_native(), 'servo_leg_2', ((-51.975, 42.8991, 70.675), ((0.5773502691896258, 0.5773502691896258, -0.5773502691896258), 239.99999999999997))))
    children.append(place(sts3215_servo_native(), 'servo_upper_leg_left', ((51.975, -5.4, 100.375), ((-0.7071067811865475, 0.0, -0.7071067811865476), 180.0))))
    children.append(place(sts3215_servo_native(), 'servo_upper_leg_right', ((-51.975, -5.4, 100.375), ((0.7071067811865475, 0.0, -0.7071067811865476), 180.0))))
    children.append(place(sts3215_servo_native(), 'servo_ankle_left', ((87.075, 42.8991, 3.975), ((0.5773502691896258, 0.5773502691896258, -0.5773502691896258), 239.99999999999997))))
    children.append(place(sts3215_servo_native(), 'servo_ankle_right', ((-87.0751, 42.8991, 3.9749), ((-0.5773502691896257, 0.5773502691896257, -0.5773502691896258), 119.99999999999999))))
    children.append(place(sts3215_servo_native(), 'servo_neck_upper', ((17.6, -35.1, 235.769), ((0.5773502691896257, -0.5773502691896257, -0.5773502691896258), 119.99999999999999))))
    children.append(place(sts3215_servo_native(), 'servo_neck_lower', ((17.6, -35.1, 163.769), ((0.5773502691896258, 0.5773502691896258, -0.5773502691896258), 239.99999999999997))))
    children.append(place(sts3215_servo_native(), 'servo_head_roll', ((0.0, -14.935, 280.5798), ((-0.5773502691896257, -0.5773502691896257, -0.5773502691896258), 119.99999999999999))))
    children.append(place(sts3215_servo_native(), 'servo_head_yaw', ((0.0, -35.1, 258.0048), ('Z', -90))))
    children.append(place(sts3215_servo_native(), 'servo_jaw_direct', ((53.0, -35.235, 284.6348), ((-0.052265535382890235, -0.9972577629795188, 0.05239530502082981), 90.01400599288137))))
    children.append(place(radxa_6530b(), 'radxa_6530b', HEATSINK_AT))
    children.append(place(parkside_papk_12_a4(), 'parkside_battery', STICK_AT))
    children.append(place(stick_pod_native(), 'stick_pod', ((0, 0, 0), ('Z', 0))))
    return bd.Compound(children=children, label="frankenduck")

if __name__ == "__main__":
    frankenduck()
