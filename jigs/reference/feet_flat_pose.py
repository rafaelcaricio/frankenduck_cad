"""Write feet_flat_pose.stl: both feet and lower legs of the robot model with the soles flat.

    uv run --project ~/development/microduck_rl --no-sync python feet_flat_pose.py

Source: microduck_rl simulation/geometry/scene.xml (the MuJoCo model, visual meshes). Pose: hip
roll 4.84 degrees inward on both legs, which lays the wedge soles flat, pitch joints at home, hip
yaw zero, trunk level. Frame of the file, mm: x forward, y to the robot's left, z up from the sole
plane, origin under the trunk_base origin. Reference only; the board is checked against it.
"""
import struct
from pathlib import Path

import mujoco
import numpy as np

ROLL_DEG = 4.84
SCENE = Path.home() / "development/microduck_rl/simulation/geometry/scene.xml"
BODIES = ("ankle_left", "leg", "ankle_right", "leg_2")
HOME = {"left_hip_pitch": -0.4579, "left_knee": -0.0049, "left_ankle": 0.453,
        "right_hip_pitch": 0.4579, "right_knee": 0.0049, "right_ankle": -0.453}

m = mujoco.MjModel.from_xml_path(str(SCENE))
d = mujoco.MjData(m)
adr = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
mujoco.mj_resetData(m, d)
d.qpos[3] = 1
for joint, value in HOME.items():
    d.qpos[adr(joint)] = value
d.qpos[adr("left_hip_roll")] = -np.radians(ROLL_DEG)
d.qpos[adr("right_hip_roll")] = np.radians(ROLL_DEG)
mujoco.mj_forward(m, d)

tris = []
for g in range(m.ngeom):
    body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
    # The ankle plate mesh carries one stray vertex far from the part; the sole and foot meshes
    # give the shape the board needs, so that mesh is left out.
    if body not in BODIES or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or "plate_and_cradle" in name:
        continue
    mesh = m.geom_dataid[g]
    v = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
    v = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
    f = m.mesh_face[m.mesh_faceadr[mesh]:m.mesh_faceadr[mesh] + m.mesh_facenum[mesh]]
    tris.append(v[f])
t = np.concatenate(tris) * 1000
trunk = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "trunk_base")] * 1000
t -= [trunk[0], trunk[1], t[:, :, 2].min()]

out = Path(__file__).with_name("feet_flat_pose.stl")
with out.open("wb") as fh:
    fh.write(b"feet_flat_pose mm".ljust(80, b" "))
    fh.write(struct.pack("<I", len(t)))
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    for normal, tri in zip(n, t):
        fh.write(struct.pack("<12fH", *normal, *tri.reshape(-1), 0))
low = t.reshape(-1, 3)
print(f"{out.name}: {len(t)} triangles; x {low[:,0].min():.1f}..{low[:,0].max():.1f} y {low[:,1].min():.1f}..{low[:,1].max():.1f} z {low[:,2].min():.1f}..{low[:,2].max():.1f}")
left = low[low[:, 1] > 0]
for z in (1.0, 2.5, 4.0):
    s = left[left[:, 2] <= z]
    print(f"  left foot below z {z}: x {s[:,0].min():+.1f}..{s[:,0].max():+.1f}  y {s[:,1].min():+.1f}..{s[:,1].max():+.1f}")
band = left[(left[:, 0] > 31) & (left[:, 0] < 45)]
print(f"  left foot, strap band x 31..45: y {band[:,1].min():+.1f}..{band[:,1].max():+.1f}  z up to {band[:,2].max():.1f}")
front = left[left[:, 0] > 19]
print(f"  left leg ahead of x 19: z up to {front[:,2].max():.1f}; highest point ahead of x 28: {left[left[:,0] > 28][:,2].max():.1f}")
