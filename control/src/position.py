# SPDX-License-Identifier: GPL-3.0-or-later
"""由 ArduPilot ``AC_PosControl`` 提炼的位置串级 PID。"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .pid import ArduPilotPID, PIDConfig


@dataclass(frozen=True)
class PositionOutput:
    acceleration_world: np.ndarray
    throttle: float
    velocity_target: np.ndarray
    position_error: np.ndarray


class ArduPilotPositionController:
    """位置 P → 速度 PID → 垂向加速度 PID。

    MuJoCo/ROS 使用 ENU（z 向上），这里已把 ArduPilot 的 NED（z 向下）符号
    转换掉，因此调用者不需要反复取负号。
    """

    def __init__(self, hover_throttle: float):
        self.hover_throttle = float(hover_throttle)
        # Copter 默认值，来自 AC_PosControl.cpp 的非 Plane/Sub 分支。
        self.position_p_xy = 1.0
        self.position_p_z = 1.0
        velocity_xy = PIDConfig(
            kp=2.0,
            ki=1.0,
            kd=0.25,
            imax=10.0,
            error_filter_hz=5.0,
            derivative_filter_hz=5.0,
        )
        self.velocity_x = ArduPilotPID(velocity_xy)
        self.velocity_y = ArduPilotPID(velocity_xy)
        self.velocity_z = ArduPilotPID(
            PIDConfig(
                kp=5.0,
                imax=10.0,
                error_filter_hz=5.0,
                derivative_filter_hz=5.0,
            )
        )
        self.acceleration_z = ArduPilotPID(
            PIDConfig(
                kp=0.05,
                ki=0.1,
                kd=0.0,
                # ArduPilot 会确保 IMAX 至少能够覆盖悬停油门。
                imax=max(0.8, self.hover_throttle),
                error_filter_hz=20.0,
                output_limit=0.65,
            )
        )
        self.max_velocity_xy = 5.0
        self.max_velocity_up = 2.5
        self.max_velocity_down = 1.5
        self.max_acceleration_xy = 1.0
        self.max_acceleration_z = 2.5

    def reset(self) -> None:
        for pid in (
            self.velocity_x,
            self.velocity_y,
            self.velocity_z,
            self.acceleration_z,
        ):
            pid.reset()

    @staticmethod
    def _limit_vector(vector: np.ndarray, max_norm: float) -> np.ndarray:
        length = float(np.linalg.norm(vector))
        if length > max_norm and length > 0.0:
            return vector * (max_norm / length)
        return vector

    def update(
        self,
        *,
        position_world: np.ndarray,
        velocity_world: np.ndarray,
        acceleration_world: np.ndarray,
        target_position_world: np.ndarray,
        dt: float,
        actuator_saturated: bool = False,
    ) -> PositionOutput:
        error = np.asarray(target_position_world) - np.asarray(position_world)

        velocity_target_xy = self._limit_vector(
            error[:2] * self.position_p_xy, self.max_velocity_xy
        )
        velocity_target_z = float(
            np.clip(
                error[2] * self.position_p_z,
                -self.max_velocity_down,
                self.max_velocity_up,
            )
        )
        velocity_target = np.array(
            [velocity_target_xy[0], velocity_target_xy[1], velocity_target_z]
        )

        accel_xy = np.array(
            [
                self.velocity_x.update(
                    velocity_target[0], velocity_world[0], dt, actuator_saturated
                ),
                self.velocity_y.update(
                    velocity_target[1], velocity_world[1], dt, actuator_saturated
                ),
            ]
        )
        accel_xy = self._limit_vector(accel_xy, self.max_acceleration_xy)
        accel_z_target = self.velocity_z.update(
            velocity_target[2], velocity_world[2], dt, actuator_saturated
        )
        accel_z_target = float(
            np.clip(accel_z_target, -self.max_acceleration_z, self.max_acceleration_z)
        )

        throttle_correction = self.acceleration_z.update(
            accel_z_target,
            float(acceleration_world[2]),
            dt,
            actuator_saturated,
        )
        throttle = float(np.clip(self.hover_throttle + throttle_correction, 0.0, 1.0))
        return PositionOutput(
            acceleration_world=np.array([accel_xy[0], accel_xy[1], accel_z_target]),
            throttle=throttle,
            velocity_target=velocity_target,
            position_error=error,
        )
