# SPDX-License-Identifier: GPL-3.0-or-later
"""把位置环、姿态环和 NCHex 动力分配串成完整飞行控制器。"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .attitude import ArduPilotAttitudeController, AttitudeOutput
from .position import ArduPilotPositionController, PositionOutput
from .rotors_deliver import MixResult, NCHexMotorMixer


def quaternion_to_rotation_matrix(quaternion_wxyz: np.ndarray) -> np.ndarray:
    """MuJoCo 的 wxyz 四元数转换为机体系到世界系的旋转矩阵。"""

    w, x, y, z = np.asarray(quaternion_wxyz, dtype=float)
    norm = max(float(np.linalg.norm([w, x, y, z])), 1.0e-12)
    w, x, y, z = np.array([w, x, y, z]) / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@dataclass(frozen=True)
class FlightState:
    position_world: np.ndarray
    velocity_world: np.ndarray
    acceleration_world: np.ndarray
    orientation_body_to_world: np.ndarray
    angular_velocity_body: np.ndarray


@dataclass(frozen=True)
class ControlOutput:
    motor_thrusts_n: np.ndarray
    position: PositionOutput
    attitude: AttitudeOutput
    mixer: MixResult
    forward_command: float
    lateral_command: float
    roll_decoupling: float
    pitch_decoupling: float


class NCHexFlightController:
    """可直接被 MuJoCo、ROS 2 节点或强化学习环境调用的无状态接口外壳。"""

    # 由当前 MJCF 质量与 B*Mixer 结果得到，避免把 ArduPilot 的非线性油门模型
    # 错当成 MuJoCo 的线性牛顿执行器。更换质量/桨叶角度后应重新标定。
    MODEL_MASS_KG = 4.6136
    FORWARD_WRENCH_PER_MOTOR_N = 7.363651
    LATERAL_WRENCH_PER_MOTOR_N = -7.333106
    # 当前模型 B*ArduPilotMixer 的交叉项。理想矩阵里这些项应为 0，
    # 但 URDF 实际桨轴和质心会让水平力同时带来小力矩。
    LATERAL_TO_ROLL_DECOUPLING = 0.3482 / 0.258126
    FORWARD_TO_PITCH_DECOUPLING = -0.348304 / 0.258907

    def __init__(self, max_thrust_n: float = 30.0, use_hover_trim: bool = True):
        self.mixer = NCHexMotorMixer(max_thrust_n, use_hover_trim)
        self.position_controller = ArduPilotPositionController(
            self.mixer.hover_throttle
        )
        self.attitude_controller = ArduPilotAttitudeController()
        self._last_saturated = False

    def reset(self) -> None:
        self.position_controller.reset()
        self.attitude_controller.reset()
        self._last_saturated = False

    def update(
        self,
        state: FlightState,
        target_position_world: np.ndarray,
        target_yaw_rad: float,
        dt: float,
    ) -> ControlOutput:
        position = self.position_controller.update(
            position_world=state.position_world,
            velocity_world=state.velocity_world,
            acceleration_world=state.acceleration_world,
            target_position_world=target_position_world,
            dt=dt,
            actuator_saturated=self._last_saturated,
        )
        attitude = self.attitude_controller.update(
            orientation_body_to_world=state.orientation_body_to_world,
            angular_velocity_body=state.angular_velocity_body,
            target_yaw_rad=target_yaw_rad,
            dt=dt,
            actuator_saturated=self._last_saturated,
        )

        rotation_body_to_world = quaternion_to_rotation_matrix(
            state.orientation_body_to_world
        )
        acceleration_body = rotation_body_to_world.T @ position.acceleration_world
        forward = (
            self.MODEL_MASS_KG
            * acceleration_body[0]
            / (self.mixer.max_thrust_n * self.FORWARD_WRENCH_PER_MOTOR_N)
        )
        # ROS/MuJoCo 的 +y 向左，而 ArduPilot NCHex 的 lateral 正方向向右；
        # 负的物理效率已在分母中完成这个符号转换。
        lateral = (
            self.MODEL_MASS_KG
            * acceleration_body[1]
            / (self.mixer.max_thrust_n * self.LATERAL_WRENCH_PER_MOTOR_N)
        )
        roll_decoupling = self.LATERAL_TO_ROLL_DECOUPLING * lateral
        pitch_decoupling = self.FORWARD_TO_PITCH_DECOUPLING * forward
        mixed = self.mixer.mix(
            forward=float(forward),
            lateral=float(lateral),
            throttle=position.throttle,
            roll=attitude.mixer_roll + roll_decoupling,
            pitch=attitude.mixer_pitch + pitch_decoupling,
            yaw=attitude.mixer_yaw,
        )
        self._last_saturated = mixed.saturated
        return ControlOutput(
            motor_thrusts_n=mixed.thrust_newtons,
            position=position,
            attitude=attitude,
            mixer=mixed,
            forward_command=float(forward),
            lateral_command=float(lateral),
            roll_decoupling=float(roll_decoupling),
            pitch_decoupling=float(pitch_decoupling),
        )
