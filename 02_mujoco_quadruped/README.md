# 任务二：MuJoCo 四足机器人基础仿真

## 1. 任务简介

本任务使用 Python 和 MuJoCo 完成四足机器人的基础仿真。

- 安装并了解 MuJoCo 的基本使用方法；
- 获取 `black` 四足机器人的 URDF 模型；
- 将 URDF 转换并整理为 MJCF；
- 为机器人添加自由基座 `freejoint`；
- 将 12 个腿部关节执行器配置为 `motor` 力矩模式；
- 创建平坦地面仿真场景；
- 使用 Python 加载模型并运行 MuJoCo 仿真；
- 设置所有关节控制输入 `data.ctrl[:] = 0.0`；
- 设置合理的初始趴卧姿态，使机器人在零主动驱动力矩情况下落到地面并逐渐稳定。  

---

## 2. 环境

测试环境：

- Ubuntu Linux
- Python 3.10.12
- MuJoCo 3.14.0

安装 MuJoCo：

```bash
python3 -m pip install mujoco

