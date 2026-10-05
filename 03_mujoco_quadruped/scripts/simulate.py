import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("../model/flat_scene.xml")
data = mujoco.MjData(model)

JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]

ACTUATOR_NAMES = [
    "FL_hip_motor", "FL_thigh_motor", "FL_calf_motor",
    "FR_hip_motor", "FR_thigh_motor", "FR_calf_motor",
    "RR_hip_motor", "RR_thigh_motor", "RR_calf_motor",
    "RL_hip_motor", "RL_thigh_motor", "RL_calf_motor",
]

q_adrs = []
dq_adrs = []
act_ids = []

for joint_name, actuator_name in zip(JOINT_NAMES, ACTUATOR_NAMES):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)

    q_adrs.append(model.jnt_qposadr[jid])
    dq_adrs.append(model.jnt_dofadr[jid])
    act_ids.append(aid)


def set_joint_position(joint_name, value):
    jid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name
    )
    data.qpos[model.jnt_qposadr[jid]] = value


Q_INIT = np.array([
    0.0, 0.8, -1.6,
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6,
    0.0, 0.8, -1.6
])

Q_STAND = np.array([
    0.0, 0.75, -1.65,
    0.0, -0.75, 1.65,
    0.0, -0.75, 1.55,
    0.0, 0.75, -1.55
])

data.qpos[q_adrs] = Q_INIT
data.qpos[0:3] = [0.0, 0.0, 0.40]
data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
data.qvel[:] = 0.0

mujoco.mj_forward(model, data)


DAMPING = 0 #阻尼模式
STAND = 1   #站立模式

state = DAMPING

KP = 80.0
KD = 3.0
TAU_MAX = 33.5

STAND_DURATION = 1.0

q_start = np.zeros(12)
stand_start_time = 0.0


def key_callback(keycode):
    global state, q_start, stand_start_time

    if keycode == ord('Z'):
        q_start = data.qpos[q_adrs].copy()
        stand_start_time = data.time
        state = STAND

    elif keycode == ord('X'):
        state = DAMPING


# 在while循环外面新增渲染计数器
render_interval = 20  # 每20个物理步渲染一次，可调
render_cnt = 0

with mujoco.viewer.launch_passive(
model,
data,
key_callback=key_callback
) as viewer:
    while viewer.is_running():
        step_start = time.time()
        
        # ========== 物理仿真（一直全速跑） ==========
        if state == DAMPING:
            for dq_adr, act_id in zip(dq_adrs, act_ids):
                dq = data.qvel[dq_adr]
                tau = -KD * dq
                data.ctrl[act_id] = np.clip(tau, -TAU_MAX, TAU_MAX)
        elif state == STAND:
            alpha = (data.time - stand_start_time) / STAND_DURATION
            alpha = np.clip(alpha, 0.0, 1.0)
            s = alpha * alpha * (3 - 2 * alpha)
            q_des = q_start + s * (Q_STAND - q_start)
            for i in range(12):
                q = data.qpos[q_adrs[i]]
                dq = data.qvel[dq_adrs[i]]
                tau = KP * (q_des[i] - q) - KD * dq
                data.ctrl[act_ids[i]] = np.clip(tau, -TAU_MAX, TAU_MAX)

        mujoco.mj_step(model, data)

        # ========== 仅达到计数才渲染sync ==========
        render_cnt += 1
        if render_cnt >= render_interval:
            viewer.sync()
            render_cnt = 0

        # 时间同步，保证物理真实时间流速
        delay = model.opt.timestep - (time.time() - step_start)
        if delay > 0:
            time.sleep(delay)
