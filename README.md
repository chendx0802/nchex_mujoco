# nchex_mujoco

NCHex 固定倾角六旋翼与 Delta 并联机械臂的 MuJoCo 仿真模块，包含 MJCF 模型、网格资源、六电机动力分配、位置/姿态串级控制，以及中英文终端目标指令。

**当前定位：可运行的独立 MuJoCo 仿真原型。** 运行不依赖 ROS、Gazebo、MAVROS 或 ArduPilot 固件。当前没有 ROS 2 节点、话题或 launch；位置跟踪和飞行机械臂协同控制仍需完善，详见文末「目前的不足」。

## 已实现内容

- 六旋翼机体、前置 Delta 机械臂、配重和起落架模型。
- 6 路旋翼推力执行器、3 路 Delta 关节位置执行器。
- 5 个闭链铰接通过 10 个 MuJoCo 点连接约束表示。
- 位置 P → 速度 PID → 垂向加速度 PID；姿态角环 → 三轴角速度 PID。
- NCHex 动力分配、当前模型的静态悬停配平和水平力/力矩前馈解耦。
- 运行时修改位置和偏航目标，支持中英文命令。
- 无界面仿真、模型编译检查和直接执行器控制示例。

当前模型总质量为 **4.6136 kg**，仿真步长为 **0.002 s**。旋翼输入是**牛顿推力**，不是 PWM、油门百分比或转速；Delta 输入是**弧度位置目标**。

## 目录结构

```text
nchex_mujoco/
├── README.md                         # 中文说明
├── COPYING / NOTICE.md               # 控制代码许可与资源来源
├── requirements.txt                  # Python 依赖
├── setup_env.sh                      # 创建项目 .venv
├── open_model.sh                     # 打开模型，不启用飞行闭环
├── run_hover_demo.sh                 # 固定推力开环演示
├── run_control_demo.sh               # 闭环位置/姿态控制
├── model/nchex.xml                   # 可直接运行的 MJCF
├── meshes/                          # 模型网格
├── control/
│   ├── main.py                      # 仿真主循环
│   └── src/
│       ├── hover.py                 # FlightState / NCHexFlightController
│       ├── position.py              # 位置、速度和垂向加速度控制
│       ├── attitude.py              # 姿态和角速度控制
│       ├── pid.py                   # PID 与滤波
│       ├── rotors_deliver.py        # 分配矩阵与电机编号映射
│       ├── rotor_visual.py          # 桨叶显示动画
│       └── command_interface.py     # 终端指令解析
├── examples/direct_actuators.py     # 直接设置推力与关节目标
└── tools/
    ├── validate_model.py            # 模型编译检查
    ├── convert_urdf_to_mjcf.py       # URDF 转换工具
    ├── hover_demo.py                # 开环演示入口
    └── python_env.sh                # 启动时选择 Python 环境
```

## 环境配置

### 推荐环境

- Linux / Ubuntu，Python **3.10 或以上**；实际验证过 Python 3.10、3.12。
- 新环境使用 `mujoco>=3.3,<4`、`numpy>=1.24`，依赖范围见 `requirements.txt`。
- 交互窗口需要桌面显示和 OpenGL；无界面物理仿真不需要桌面。
- **不需要 CUDA、PyTorch 或 Gymnasium**，它们不是当前模块的运行依赖。

MuJoCo 的 Python 包自带仿真库，无需另行下载 MuJoCo 二进制程序，参见 [MuJoCo 官方安装说明](https://mujoco.readthedocs.io/en/stable/python.html#installation)。

### 方式 A：项目独立环境

```bash
git clone https://github.com/chendx0802/nchex_mujoco.git
cd nchex_mujoco

# Ubuntu 缺少 venv、pip 或图形运行库时安装
sudo apt update
sudo apt install -y python3-venv python3-pip libgl1 libglfw3

# 创建 .venv 并安装 requirements.txt
./setup_env.sh

# 验证模型可加载
./.venv/bin/python tools/validate_model.py

# 无界面运行 5 秒
./run_control_demo.sh --headless --duration 5 --quiet
```

模型检查正常输出包含 `PASS`、`actuators=9`、`equality_constraints=10`、`sensors=3`。这表示模型可编译，不表示控制性能已经通过验收。

也可以手动安装：

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install -r requirements.txt
```

### 方式 B：已有 Conda / Python 环境

```bash
conda create -n nchex_mujoco python=3.12 -y
conda activate nchex_mujoco
python -m pip install -r requirements.txt

export MUJOCO_PYTHON="$(command -v python)"
./run_control_demo.sh --headless --duration 5
```

三个启动脚本按以下顺序选择解释器：

1. `MUJOCO_PYTHON` 指定的解释器。
2. 仓库内 `.venv/bin/python`。
3. 当前 `PATH` 中的 `python3`。

若已有 `.venv`，但希望使用 Conda，请显式设置 `MUJOCO_PYTHON`。该变量只控制 shell 启动器；直接运行 Python 文件时，使用自己选定的解释器。

### 已验证版本与平台边界

| 环境 | Python | MuJoCo | NumPy | 验证范围 |
|---|---|---|---|---|
| 原有兼容环境 | 3.10 | 2.3.3 | 1.23.3 | 模型加载、无界面控制场景 |
| 3.x 环境 | 3.12 | 3.14.0 | 2.5.3 | 模型加载、无界面控制场景 |
| 发布时全新安装 | 3.12 | 3.17.0 | 2.5.4 | 安装脚本、模型加载、5 秒悬停、直接执行器示例 |

旧环境记录不代表新安装应使用旧版 NumPy；新安装遵循 `requirements.txt`。GUI 在不同系统上的兼容性未全面验证。macOS 的被动 viewer 要使用 `mjpython`，例如 `./.venv/bin/mjpython control/main.py`，参见 [官方 viewer 说明](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer)。本仓库 shell 启动方式以 Linux 为主。

## 启动方式

### 仅查看模型

```bash
./open_model.sh
```

这只打开 MuJoCo 标准 viewer，不运行位置控制器。启动物理后，未施加足够推力的机体会在重力作用下下落。

### 固定推力开环演示

```bash
./run_hover_demo.sh
```

六路推力采用当前模型的静态配平值，机械臂三个关节目标为 0 rad。它不根据位置误差调节推力，不能等同于闭环悬停或抗扰控制。

### 闭环控制与终端交互

```bash
./run_control_demo.sh

# 指定世界系位置和偏航目标
./run_control_demo.sh --x 0.5 --y 0.2 --z 0.8 --yaw-deg 20

# 无界面运行；duration 必须为有限正数
./run_control_demo.sh --headless --duration 5 --quiet

# 禁用终端交互
./run_control_demo.sh --no-stdin
```

默认初始位置和目标均为 `(0, 0, 0.62)` m，偏航目标为 0°。这不是从地面自动起飞流程。图形模式默认持续运行，关闭窗口或输入 `quit` 结束。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--x` / `--y` / `--z` | `0` / `0` / `0.62` | 世界系目标位置，m |
| `--yaw-deg` | `0` | 绝对偏航目标，° |
| `--duration` | `0` | 仿真时长；0 表示图形模式持续运行 |
| `--headless` | 关闭 | 无界面运行，必须配合正的时长 |
| `--quiet` | 关闭 | 隐藏周期状态，仍输出结束结果 |
| `--no-stdin` | 关闭 | 禁用终端目标输入；无界面模式本身不读终端命令 |
| `--no-hover-trim` | 关闭 | 关闭当前质心对应的静态配平 |

窗口打开后，在**启动它的同一终端**输入命令并回车：

| 指令 | 中文别名 | 功能 |
|---|---|---|
| `forward 0.5` | `前进 0.5` | 沿目标机头方向增加 0.5 m |
| `back 0.3` | `后退 0.3` | 沿目标机尾方向移动 |
| `left 0.2` / `right 0.2` | `左移 0.2` / `右移 0.2` | 向目标机体左/右侧移动 |
| `up 0.2` / `down 0.1` | `上升 0.2` / `下降 0.1` | 增减目标高度 |
| `move 0.1 0.2 0.0` | `移动 0.1 0.2 0.0` | 世界系增量 dx、dy、dz，m |
| `target 0.5 0.2 0.8 20` | `目标 0.5 0.2 0.8 20` | 绝对位置与可选绝对偏航角 |
| `yaw 30` | `偏航 30` | 设置绝对偏航角，° |
| `turn 15` | `转向 15` | 增加相对偏航角，° |
| `status` | `状态` | 查看当前目标，不是实际位置 |
| `help` | `帮助` | 查看帮助 |
| `quit` / `exit` | `退出` | 结束仿真 |

前后左右按**目标偏航角**计算方向，不是实际瞬时偏航；省略单个移动距离时默认 0.2 m。终端高度目标下限 0.05 m 只是数值限幅，不保证模型不触地。当前校验不完整，请只输入有效有限数值。

## 接口名称与单位

以下是 **MuJoCo 名称和 Python 接口**，当前没有对应的 ROS 2 话题或服务。

### 六路旋翼推力接口

| 执行器名称 | 推力作用点 | 桨叶显示关节 | ArduPilot 编号 | 输入 |
|---|---|---|---|---|
| `rotor_0_thrust` | `rotor_0_thrust_site` | `rotor_0_joint` | M1 | 0～30 N |
| `rotor_1_thrust` | `rotor_1_thrust_site` | `rotor_1_joint` | M4 | 0～30 N |
| `rotor_2_thrust` | `rotor_2_thrust_site` | `rotor_2_joint` | M2 | 0～30 N |
| `rotor_3_thrust` | `rotor_3_thrust_site` | `rotor_3_joint` | M5 | 0～30 N |
| `rotor_4_thrust` | `rotor_4_thrust_site` | `rotor_4_joint` | M6 | 0～30 N |
| `rotor_5_thrust` | `rotor_5_thrust_site` | `rotor_5_joint` | M3 | 0～30 N |

旋翼沿各自 site 的局部 `+Z` 施力，因桨轴倾斜，不能把六路推力直接相加当作竖直升力。反扭矩已包含在 actuator 的 `gear` 中，系数为 ±0.06 m。`rotor_i_joint` 用于桨叶显示，**不是转速控制接口**。

### Delta 关节位置接口

| 执行器名称 | 被控关节 | 目标范围 | 执行器力矩范围 |
|---|---|---|---|
| `delta_motor1_position` | `delta_motor1` | −1.1～1.1 rad | −20～20 N·m |
| `delta_motor2_position` | `delta_motor2` | −1.1～1.1 rad | −20～20 N·m |
| `delta_motor3_position` | `delta_motor3` | −1.1～1.1 rad | −20～20 N·m |

位置执行器 `kp=80`。三个输入是主动关节角目标，不是末端 XYZ；范围不等同于完整可达、无碰撞工作空间。

**`control/main.py` 每步都会将三个 Delta 目标重新设为 0。** 若要在此入口驱动机械臂，需要替换 `data.ctrl[delta_ids] = 0.0`，接入自己的目标生成器。

### 传感器与状态

| 名称 / 字段 | 内容 | 坐标系与单位 |
|---|---|---|
| `base_orientation` | 四元数，4 维 | `w,x,y,z`，机体到世界 |
| `base_angular_velocity` | 陀螺仪，3 维 | `imu_site` 局部轴，rad/s |
| `base_linear_acceleration` | 加速度计，3 维 | `imu_site` 局部轴比力，m/s²；不等于世界系惯性加速度 |
| `imu_site` | 传感器安装点 | 当前位于 `base_link` 原点，轴与机体一致 |
| `base_freejoint` | 机体自由关节 | 当前模型的第一关节 |
| `data.qpos[:3]` | 机体位置 | 世界系，m |
| `data.qpos[3:7]` | 机体姿态 | `w,x,y,z` |
| `data.qvel[:3]` | 机体线速度 | 世界系，m/s |
| `data.qacc[:3]` | 机体惯性加速度 | 世界系，m/s²；垂向控制使用此项 |

世界系采用 z 向上的 ROS/ENU 约定，机体系为 FLU（x 前、y 左、z 上）。切片对应当前布局；若重新排列关节，应按名称获取地址。

按名称读取传感器（`model` 和 `data` 为已创建的 MuJoCo 对象）：

```python
sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,
                              "base_angular_velocity")
assert sensor_id >= 0
start = int(model.sensor_adr[sensor_id])
size = int(model.sensor_dim[sensor_id])
gyro_body = data.sensordata[start:start + size].copy()
```

### Python 控制器接口

从仓库根目录导入：

```python
from control.src import FlightState, NCHexFlightController
from control.src.rotors_deliver import NCHexMotorMixer
from control.src.command_interface import TargetCommandState
```

| 接口 | 输入 / 输出 |
|---|---|
| `FlightState(...)` | `position_world`、`velocity_world`、`acceleration_world`、`orientation_body_to_world`、`angular_velocity_body` |
| `NCHexFlightController(max_thrust_n=30.0, use_hover_trim=True)` | 创建包含 PID 内部状态的控制器 |
| `controller.update(state, target_position_world, target_yaw_rad, dt)` | 位置 m、偏航 rad、周期 s；返回 `ControlOutput` |
| `output.motor_thrusts_n` | 长度 6 的推力数组，按 `rotor_0`～`rotor_5` 排列，N |
| `controller.reset()` | 重置内部积分/滤波状态；不重置模型或仿真时间 |
| `NCHexMotorMixer.mix(forward=..., lateral=..., throttle=..., roll=..., pitch=..., yaw=...)` | 六维分配指令，不是直接输入 N 或 N·m；单位转换由 `hover.py` 处理 |
| `TargetCommandState.execute(text)` | 更新目标，返回 `(提示文本, 是否退出)` |
| `TargetCommandState.snapshot()` | 返回目标位置副本与偏航角 rad |

## 如何直接控制电机

### 方式 A：运行现成示例

独立示例只推进物理和桨叶显示，不运行位置/姿态闭环，不会覆盖你指定的推力或关节目标：

```bash
# 默认配平推力，三个关节目标为零；无界面运行 1 秒
./.venv/bin/python examples/direct_actuators.py

# 顺序：rotor_0、1、2、3、4、5；delta_motor1、2、3
./.venv/bin/python examples/direct_actuators.py \
  --duration 1 \
  --rotor-thrusts 10.713325 10.578724 10.675978 10.614516 10.748285 10.675652 \
  --delta-joints 0.1 0.1 0.1
```

要修改某一电机，就修改 `--rotor-thrusts` 中对应位置的数值。改变推力或机械臂姿态会破坏原有配平；示例用于验证接口，不保证保持悬停。使用 Conda 时将 `./.venv/bin/python` 换成环境内的 `python`。

### 方式 B：在自己的循环中写执行器

以下代码在仓库根目录运行：

```python
from pathlib import Path
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path(str(Path("model/nchex.xml").resolve()))
data = mujoco.MjData(model)

def actuator_id(name):
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if idx < 0:
        raise ValueError(f"找不到执行器: {name}")
    return idx

rotor_ids = [actuator_id(f"rotor_{i}_thrust") for i in range(6)]
delta_ids = [actuator_id(f"delta_motor{i}_position") for i in range(1, 4)]
thrusts = np.array([10.713325, 10.578724, 10.675978,
                   10.614516, 10.748285, 10.675652])
data.ctrl[rotor_ids] = thrusts                 # N
data.ctrl[delta_ids] = [0.1, 0.1, 0.1]        # rad

# 单独改变 rotor_0 的推力；这将改变机体运动
data.ctrl[actuator_id("rotor_0_thrust")] = 11.0

mujoco.mj_forward(model, data)
for _ in range(500):                         # 当前模型共 1 秒
    mujoco.mj_step(model, data)

joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "delta_motor1")
angle = data.qpos[model.jnt_qposadr[joint_id]]  # 实际角度，不一定等于目标
print("机体位置:", data.qpos[:3], "关节1角度:", angle)
```

关节 ID、执行器 ID、`qpos` 地址是不同概念，应分别查找。`data.ctrl` 只写指令，`mj_step` 才推进动力学。在闭环演示循环外修改旋翼控制，下一步控制器输出会再次覆盖它。

## 修改模型与重新标定

已生成的 `model/nchex.xml` 与 `meshes/` 足够运行。原始 ROS1 URDF 未包含在本仓库；只有重新转换时才需要自行提供源文件。

转换工具当前先清空仓库 `meshes/` 再复制源资源，**请在独立工作副本中操作，并先确认源目录有效、与目标目录不同**：

```bash
./.venv/bin/python tools/convert_urdf_to_mjcf.py \
  --urdf /path/to/nchex/urdf/nchex_rotors.urdf \
  --mesh-dir /path/to/nchex/meshes
./.venv/bin/python tools/validate_model.py
```

当前不建议改变 `--output` 的目录布局：网格输出固定在仓库 `meshes/`，XML 使用 `../meshes` 相对引用。默认 URDF 路径仍遵循开发时相邻 `ros1/模型文件/nchex` 布局，克隆后请显式提供源路径。

质量、载荷、旋翼倾角或机械臂构型变化后，需要重新检查：

- `control/src/rotors_deliver.py` 的 `MUJOCO_HOVER_THRUSTS_N` 和电机映射。
- `control/src/hover.py` 的质量、水平力效和力矩解耦常量。
- `tools/hover_demo.py` 的固定悬停推力。
- `model/nchex.xml` 的惯性、执行器范围、碰撞和闭链约束。

## 目前的不足

本次发布保留原控制算法与模型行为；以下问题已知，不能将原型视为完整飞控或已经完成 ROS 2 集成的系统。

1. **位置控制有明显超调与持续振荡。** X 目标 10 m 的实测峰值约 16.1 m；目标 `(0.5,0.2,0.8)`、偏航 20° 时，60 秒后 X 仍在约 0.465～0.536 m 往复。XY 加速度限幅没有反馈到速度 PID 的积分约束，需要完善抗积分饱和和闭环整定；小位移振荡根因尚未完全定位。
2. **ROS 2 集成未实现。** 缺少 `package.xml`、ament 构建、节点、launch、`/clock`、里程计/关节状态发布和目标订阅。文中的接口名称不是 ROS 话题。
3. **Delta 仅有关节执行器。** 主闭环将三个关节目标置零；无末端正逆运动学、轨迹规划、工作空间校验或飞行—机械臂协同控制。直接控制示例不等于完成这些功能。
4. **终端和主入口参数校验不足。** `nan` / `inf` 可能污染目标或导致退出；错误的 `target` 偏航参数可能在报错前修改位置。新增直接控制示例会校验有限数值，原入口的问题尚未修复。
5. **转换缺少失败保护。** 源网格不可用时可能先删除已有网格；自定义输出路径的资源布局未完善。重新生成前需要独立副本和有效源资源。
6. **没有完整自动验收。** 编译成功、正常退出和最终单帧误差不能证明收敛；`max_speed_mps` 字段实际记录结束速度。缺少连续稳态误差、超调、关节轨迹和故障恢复测试。
7. **动力学与传感器有简化。** 旋翼使用直接力模型，无电机时间常数、真实转速闭环或详细气动；桨叶转动主要用于显示。直接读取仿真状态，无完整噪声、延迟、状态估计和电机失效处理。
8. **标定依赖当前构型。** 固定质量、配平和解耦系数不会随机械臂或载荷自动更新；部分闭链连接碰撞体初始存在约 2 mm 接触穿透，仍需核对建模意图。
9. **工程交付信息仍需完善。** 尚无通用安装包、完整 CI、配置文件化参数及逐文件模型资源来源/许可信息。GUI 和非 Linux 平台未完整验证。

### 当前验证结果

| 场景 | 结果 |
|---|---|
| MuJoCo 2.3.3 / 3.14.0 加载 | 均成功；9 个执行器、10 个闭链约束、3 个传感器 |
| 初始位置悬停 30 s | 最终位置误差约 0.5 mm |
| 小位移 + 偏航目标 | 能跟踪但仍有振荡，单帧到点不代表稳定 |
| 10 m 位移目标 | 峰值约 16.1 m，明显超调 |
| 机械臂小幅正弦输入 | 可推进仿真，机体有约 0.1 m 水平偏移，协同补偿未完成 |

以上来自确定性、无界面、当前参数下的测试，不构成真实硬件性能结论。

## 来源与许可

控制器提炼自 ArduPilot NCHex 定制实现及公共控制器，代码对应关系见 [control/README.md](control/README.md)。标记 GPL-3.0-or-later 的代码遵循该许可，完整文本见 [COPYING](COPYING)。模型和网格的已知来源及许可信息边界见 [NOTICE.md](NOTICE.md)。
