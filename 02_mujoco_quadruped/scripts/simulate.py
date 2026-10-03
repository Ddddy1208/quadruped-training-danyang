import mujoco
import time

from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
XML_PATH = SCRIPT_DIR.parent / "model" / "flat_scene.xml"

model = mujoco.MjModel.from_xml_path(str(XML_PATH))

data = mujoco.MjData(model)

import mujoco

for i in range(model.njnt):
    print("joint:", i, mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_JOINT, i))

print("-----")

for i in range(model.nu):
    print("actuator:", i, mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, i))

print("nq =", model.nq)
print("nv =", model.nv)
print("nu =", model.nu)
print("njnt =", model.njnt)

print("\n--- joint information ---")

for jid in range(model.njnt):

    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        jid
    )

    print(
        "jid =", jid,
        "name =", name,
        "qpos_adr =", model.jnt_qposadr[jid],
        "dof_adr =", model.jnt_dofadr[jid]
    )
def set_joint_position(model, data, joint_name, value):
    jid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name
    )

    qpos_adr = model.jnt_qposadr[jid]
    data.qpos[qpos_adr] = value

# 左前腿 FL
set_joint_position(model, data, "FL_hip_joint", 0.0)
set_joint_position(model, data, "FL_thigh_joint", 0.8)
set_joint_position(model, data, "FL_calf_joint", -1.6)

# 右前腿 FR
set_joint_position(model, data, "FR_hip_joint", 0.0)
set_joint_position(model, data, "FR_thigh_joint", -0.8)
set_joint_position(model, data, "FR_calf_joint", 1.6)

# 右后腿 RR
set_joint_position(model, data, "RR_hip_joint", 0.0)
set_joint_position(model, data, "RR_thigh_joint", -0.8)
set_joint_position(model, data, "RR_calf_joint", 1.6)

# 左后腿 RL
set_joint_position(model, data, "RL_hip_joint", 0.0)
set_joint_position(model, data, "RL_thigh_joint", 0.8)
set_joint_position(model, data, "RL_calf_joint", -1.6)

data.qpos[0:3] = [0.0, 0.0, 0.20]
data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

data.qvel[:] = 0.0

mujoco.mj_forward(model, data)

import mujoco.viewer
import time

count = 0

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        step_start = time.time()

        # 任务要求：12 个 motor 全部零输入
        data.ctrl[:] = 0.0

        # 推进一个物理时间步
        mujoco.mj_step(model, data)
        count +=1
        if count % 500 == 0:
            print(
            "time =", round(data.time, 2),
            "base_z =", round(data.qpos[2], 4),
            "max_speed =", round(max(abs(v) for v in data.qvel), 6),
            "max_ctrl =", round(max(abs(u) for u in data.ctrl), 6)
            )
        # 更新画面
        viewer.sync()

        # 尽量保持接近实时速度
        time_left = model.opt.timestep - (time.time() - step_start)

        if time_left > 0:
            time.sleep(time_left)

