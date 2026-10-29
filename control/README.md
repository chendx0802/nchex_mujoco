# NCHex 控制器移植说明

本目录把 ArduPilot NCHex 的控制调用链提炼为纯 Python，直接驱动
`model/nchex.xml`，不依赖飞控固件、MAVROS 或 ROS 1。

## 文件对应关系

- `src/rotors_deliver.py`：来自 `AP_Motors_NCHex.cpp` 的六电机动力分配矩阵、
  平移/姿态项限幅，以及 ArduPilot M1～M6 到 MuJoCo rotor_0～rotor_5 的映射。
- `src/position.py`：来自公共 `AC_PosControl` 的位置 P、速度 PID、垂向加速度 PID。
- `src/attitude.py`：来自 `AC_AttitudeControl`/`AC_AttitudeControl_Multi` 的姿态角 P
  和角速度 PID；NCHex 保持 roll/pitch 水平，用倾斜桨直接产生水平力。
- `src/pid.py`：来自 `AC_PID` 的低通、积分限幅和抗积分饱和逻辑。
- `src/hover.py`：坐标系、单位与执行器适配，并把三层控制器串起来。
- `src/rotor_visual.py`：按推力平方根和 CW/CCW 方向更新六个桨叶显示角度。
- `src/command_interface.py`：运行中修改位置与偏航目标的中英文终端指令。
- `main.py`：MuJoCo 闭环仿真入口。

## 运行中发送指令

执行 `./run_control_demo.sh` 后，保持该终端打开，直接输入：

```text
forward 0.5
back 0.3
left 0.2
right 0.2
up 0.2
down 0.1
yaw 30
target 1.0 0.5 0.8 45
quit
```

`forward/back/left/right` 按当前目标偏航角所定义的机头方向移动；
`move dx dy dz` 按世界 ENU 坐标增加目标。中英文命令均可，例如
`前进 0.5`、`上升 0.2`、`偏航 30`。输入 `help` 查看完整帮助。
使用 `--no-stdin` 可关闭交互终端。

## 重要适配

ArduPilot 使用 NED/FRD，模型使用 ROS/MuJoCo ENU/FLU；代码已统一成 x 前、y 左、
z 上。ArduPilot 的油门是归一化的非线性电机量，而 MJCF 执行器输入是线性牛顿，
所以 `hover.py` 单独使用当前 4.6136 kg 模型的力效标定。模型质量、桨倾角或最大推力
改变后，应重新标定这些常量。

当前 MJCF 的实际桨轴/质心与 ArduPilot 理想矩阵存在小交叉项：前向力会附带俯仰
力矩，横向力会附带滚转力矩。`hover.py` 在仿真适配层加入了静态前馈解耦；原始
ArduPilot 矩阵和 PID 参数均未改动。

当前静态配平默认开启，用六个很小的不同 trim 补偿机械臂和配重带来的质心偏移。
它不是 ArduPilot 原矩阵的一部分，可用 `--no-hover-trim` 关闭。

六个飞行执行器仍然是力级模型（输入单位 N），不是电机转速模型。桨叶关节的旋转
仅用于正确显示，并按 `omega ∝ sqrt(thrust)` 计算；它不会额外施加反扭矩，因为
MJCF 的 thrust actuator 已包含偏航力矩。后续做电机级强化学习时，可再加入电机时间
常数、`k_f*omega²` 和转速观测量。

仿真入口同时兼容当前 `ramps` 环境里的 MuJoCo 2.3.3 和新版 MuJoCo 3.x。
2.3.3 的被动窗口接口不仅没有 ViewerHandle，后台 GLFW 窗口线程还可能在
`Ctrl+C` 时段错误；因此旧版把 GLFW 固定在主线程，在加锁的工作线程运行控制与
物理，新版采用 ViewerHandle。两条路径都能在关闭窗口或按 `Ctrl+C` 时完成资源清理。

## 来源和许可

核心算法由 ArduPilot 的 NCHex 定制源码提炼，主要来源：

- `libraries/AP_Motors/AP_Motors_NCHex.cpp`
- `libraries/AC_AttitudeControl/AC_AttitudeControl_NCHex.cpp`
- `libraries/AC_AttitudeControl/AC_AttitudeControl.cpp`
- `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h`
- `libraries/AC_AttitudeControl/AC_PosControl.cpp`
- `libraries/AC_PID/AC_PID.cpp`

这些移植代码按 ArduPilot 的 GPLv3（或更新版本）处理；完整许可证见
仓库根目录 [COPYING](../COPYING)，来源范围见 [NOTICE.md](../NOTICE.md)。

## 当前移植边界

已经接入的是实际飞行控制核心：位置 P、速度 PID、垂向加速度 PID、姿态角 P、
三轴角速度 PID、六电机分配、饱和缩放和积分抗饱和。没有搬入依赖飞控传感器栈的
EKF 重置、地形跟随、振动补偿、失效电机检测和可选 ESO；NCHex 源码中 ESO 默认
也是关闭的。这些功能不影响当前 MuJoCo 中的位置/姿态控制验证。
