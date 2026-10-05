# 任务二：实体电机控制（宇树 Go-M8010-6）
## 任务内容
基于宇树 `unitree_actuator_sdk` 开发Go-M8010-6电机位置控制程序，实现以下功能：
1. 基于宇树官方SDK，驱动GO-M8010-6电机正常运转；
2. 实现电机缓慢回归零点；支持键盘输入目标角度，电机平滑转动至指定输出端角度；
3. 标记原始零点，将零点正向偏移30°，验证给定目标角度后，电机实际输出角度与终端打印位置是否匹配预期；
4. 处理零点跳变问题：手动将电机输出端拨至零点前后位置，再控制电机运动到目标位置，验证零点无跳变。

## 硬件与环境
- 硬件：宇树 Go-M8010-6 伺服电机
- 编译环境：
  - x86平台：gcc >= 5.4.0
  - Arm平台：gcc >= 7.5.0
  - 查看gcc版本命令：
  ```bash
  gcc --version

## 编译
mkdir build
cd build
cmake ..
make
说明：编译生成的`.so`动态库文件不纳入本仓库，需要本地编译 SDK 生成。

## 运行
sudo ./example_goM8010_6_motor_test

## 代码说明
- `example/example_goM8010_6_motor_test.cpp`：**自主编写**，实现回零、键盘角度输入、平滑位置控制、零点偏移、零点跳变处理逻辑。
- 底层电机通信、驱动 API 来自宇树官方 `unitree_actuator_sdk`。

> ⚠️重要提示：SDK 内 cmd 指令全部针对**电机转子侧 (rotor)**；我们任务计算的角度是**输出端 (output)**角度，赋值时需要考虑减速比带来的换算。

## 参考来源
宇树科技 unitree_actuator_sdk 官方 SDK

> 
> 原版 SDK 文档：If you have any questions or need assistance with usage, feel free to contact support@unitree.com.
> 支持电机：GO-M8010-6、A1、B1