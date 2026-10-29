# SPDX-License-Identifier: GPL-3.0-or-later
"""ArduPilot NCHex 的姿态角环和角速度 PID。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from .pid import ArduPilotPID, PIDConfig


def quaternion_conjugate(quaternion: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternion, dtype=float).copy()
    result[1:] *= -1.0
    return result


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def yaw_quaternion(yaw_rad: float) -> np.ndarray:
    return np.array([math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)])


def quaternion_error_rotation_vector(
    current_body_to_world: np.ndarray, target_body_to_world: np.ndarray
) -> np.ndarray:
    """用最短旋转返回当前机体系表达的姿态误差旋转向量。"""

    error_quat = quaternion_multiply(
        quaternion_conjugate(current_body_to_world), target_body_to_world
    )
    error_quat /= max(float(np.linalg.norm(error_quat)), 1.0e-12)
    if error_quat[0] < 0.0:
        error_quat *= -1.0
    vector_norm = float(np.linalg.norm(error_quat[1:]))
    if vector_norm < 1.0e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, float(error_quat[0]))
    return error_quat[1:] * (angle / vector_norm)


def sqrt_controller(error: float, p_gain: float, acceleration_limit: float, dt: float) -> float:
    """移植 AP_Math ``sqrt_controller``，大角度时限制角加速度。"""

    if acceleration_limit <= 0.0:
        correction_rate = error * p_gain
    elif p_gain == 0.0:
        correction_rate = math.copysign(
            math.sqrt(2.0 * acceleration_limit * abs(error)), error
        ) if error != 0.0 else 0.0
    else:
        linear_distance = acceleration_limit / (p_gain * p_gain)
        if error > linear_distance:
            correction_rate = math.sqrt(
                2.0 * acceleration_limit * (error - linear_distance / 2.0)
            )
        elif error < -linear_distance:
            correction_rate = -math.sqrt(
                2.0 * acceleration_limit * (-error - linear_distance / 2.0)
            )
        else:
            correction_rate = error * p_gain
    if dt > 0.0:
        correction_rate = float(
            np.clip(correction_rate, -abs(error) / dt, abs(error) / dt)
        )
    return correction_rate


@dataclass(frozen=True)
class AttitudeOutput:
    desired_body_rate: np.ndarray
    attitude_error: np.ndarray
    # 下面三个量已经按当前 MuJoCo/ArduPilot 矩阵的符号关系转换。
    mixer_roll: float
    mixer_pitch: float
    mixer_yaw: float


class ArduPilotAttitudeController:
    """水平机体姿态角 P 环 + 三轴角速度 PID 环。"""

    def __init__(self):
        self.angle_p = 4.5
        # 源码默认 ACCEL_R/P_MAX=1100deg/s²；角环内部使用它的一半。
        self.angle_accel_rp = math.radians(550.0)
        # 默认 yaw 为 270deg/s²，之后受 120deg/s² 上限约束。
        self.angle_accel_yaw = math.radians(120.0)
        rate_rp = PIDConfig(
            kp=0.135,
            ki=0.135,
            kd=0.0036,
            imax=0.5,
            target_filter_hz=20.0,
            derivative_filter_hz=20.0,
            output_limit=1.0,
        )
        self.rate_roll = ArduPilotPID(rate_rp)
        self.rate_pitch = ArduPilotPID(rate_rp)
        self.rate_yaw = ArduPilotPID(
            PIDConfig(
                kp=0.180,
                ki=0.018,
                kd=0.0,
                imax=0.5,
                target_filter_hz=20.0,
                error_filter_hz=2.5,
                derivative_filter_hz=20.0,
                output_limit=1.0,
            )
        )

    def reset(self) -> None:
        self.rate_roll.reset()
        self.rate_pitch.reset()
        self.rate_yaw.reset()

    def update(
        self,
        *,
        orientation_body_to_world: np.ndarray,
        angular_velocity_body: np.ndarray,
        target_yaw_rad: float,
        dt: float,
        actuator_saturated: bool = False,
    ) -> AttitudeOutput:
        # NCHex 的横向移动由倾斜电机直接完成，所以 roll/pitch 始终以水平为目标。
        target_orientation = yaw_quaternion(target_yaw_rad)
        attitude_error = quaternion_error_rotation_vector(
            orientation_body_to_world, target_orientation
        )
        desired_rate = np.array(
            [
                sqrt_controller(
                    attitude_error[0], self.angle_p, self.angle_accel_rp, dt
                ),
                sqrt_controller(
                    attitude_error[1], self.angle_p, self.angle_accel_rp, dt
                ),
                sqrt_controller(
                    attitude_error[2], self.angle_p, self.angle_accel_yaw, dt
                ),
            ]
        )
        logical_roll = self.rate_roll.update(
            desired_rate[0], angular_velocity_body[0], dt, actuator_saturated
        )
        logical_pitch = self.rate_pitch.update(
            desired_rate[1], angular_velocity_body[1], dt, actuator_saturated
        )
        logical_yaw = self.rate_yaw.update(
            desired_rate[2], angular_velocity_body[2], dt, actuator_saturated
        )

        # 对当前 MJCF 做过 B*M 验证：AP roll 列产生 -Mx，pitch/yaw 分别产生 +My/+Mz。
        return AttitudeOutput(
            desired_body_rate=desired_rate,
            attitude_error=attitude_error,
            mixer_roll=-logical_roll,
            mixer_pitch=logical_pitch,
            mixer_yaw=logical_yaw,
        )
