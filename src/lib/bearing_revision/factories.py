"""Canonical part factories. Native frame is world rotated +90 degrees about Z."""
from cadgen import build123d as bd
from .runtime import cached_shape
from .collision_bridge_v2 import place
from .upper_leg_carrier_v10 import instances, inner_carrier
from .hip_servos_bearing_revision import left_hip_dual, right_hip_dual
from .hip_cradle_thread_forming_v8 import fastened
from .shared_hip_yaw_blended_v2 import blended_adapter
from .leg_bearing_revision import left_leg, right_leg
from .ankle_bearing_revision import left_ankle
from .right_leg_bearing_revision import right_ankle, right_carrier
from .leg_foot_bearing_review_v2 import delta
from .adapter_bridge_v3 import adapter_print, COLLAR_FRONT, bearing_envelope, servo_adapter_review, world_parts

def native(shape):
    s=shape.rotate(bd.Axis.Z,90)
    s.color='#1d1d1f'
    for child in s.children: child.color=s.color
    return s

@cached_shape
def revised(name):
    i=instances()
    side='right' if name.endswith('_right') else 'left'
    if name.startswith('hip_left_native'):return native((right_hip_dual if side=='right' else left_hip_dual)())
    if name.startswith('yaw2roll_left_native'):return native(fastened(side)[1])
    knee='servo_leg_2' if side=='right' else 'servo_leg'
    ankle='servo_ankle_right' if side=='right' else 'servo_ankle_left'
    if name.startswith('leg_left_native'):return native(place((right_leg if side=='right' else left_leg)(),i[knee]['at']))
    if name.startswith('ankle_left_native'):return native(place((right_ankle if side=='right' else left_ankle)(),i[ankle]['at']).translate(delta(i[knee]['at'])))
    raise ValueError(name)

@cached_shape
def added(name):
    if name=='bearing_carrier_left':return native(inner_carrier())
    if name=='bearing_carrier_right':return native(right_carrier())
    if name=='bearing_roll_adapter_left':return native(fastened('left')[0])
    if name=='bearing_roll_adapter_right':return native(fastened('right')[0])
    if name=='bearing_shared_yaw_adapter':return native(blended_adapter())
    if name=='bearing_ankle_adapter':
        s=adapter_print().translate((0,0,COLLAR_FRONT));s.color='#1d1d1f';return s
    if name=='bearing_6704':
        s=bearing_envelope();s.color='#bfc3c7';return s
    if name.startswith('bearing_screws_'):
        return native(bd.Compound(children=fastened(name.rsplit('_',1)[1])[2]))
    raise ValueError(name)

@cached_shape
def servo():
    parts=[p.rotate(bd.Axis.X,180) for p in world_parts(servo_adapter_review()) if 'PRINTED' not in p.label and '从动' not in p.label]
    for p in parts:p.color='#bfc3c7' if 'horn OD' in p.label else '#101010'
    return bd.Compound(children=parts,label='STS3215 vendor casing and measured horn envelope')
