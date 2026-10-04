# 任务二：MuJoCo 四足机器人仿真

## 任务内容
使用 Python 和 MuJoCo 完成 `black` 四足机器人基础仿真。将原始 URDF 转换并整理为 MJCF，为机器人添加 `freejoint`，将 12 个腿部关节配置为 `motor` 力矩执行器，并建立平坦地面场景。仿真中设置 `data.ctrl[:] = 0.0`，机器人在重力和地面接触作用下最终稳定趴卧。

## 环境
- Ubuntu
- Python 3.10.12
- MuJoCo 3.14.0

## 运行
```bash
pip install -r requirements.txt

cd 02_mujoco_quadruped/scripts
python3 simulate.py

模型来源：N-W-wolf/Training_Materials。