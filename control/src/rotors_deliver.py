# SPDX-License-Identifier: GPL-3.0-or-later
"""NCHex 六旋翼动力分配器。

矩阵逐项取自 ``AP_Motors_NCHex.cpp::setup_motors``。ArduPilot 的电机序号
与本项目 MJCF 的 rotor 序号不同，所以本文件集中处理编号映射，防止再次出现
“六个电机数值一样但一离地就翻”的问题。
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


# 行：ArduPilot M1..M6；列：前向、横向、总升力、滚转、俯仰、偏航。
ARDUPILOT_NCHEX_MATRIX = np.array(
    [
        [1.732038, -3.0, 1.0, -0.33338, 0.577367, -0.5],
        [1.732038, 3.0, 1.0, 0.33338, 0.577367, 0.5],
        [-3.464076, 0.0, 1.0, 0.66662, 0.0, -0.5],
        [1.732038, -3.0, 1.0, 0.33338, -0.577367, 0.5],
        [1.732038, 3.0, 1.0, -0.33338, -0.577367, -0.5],
        [-3.464076, 0.0, 1.0, -0.66666, 0.0, 0.5],
    ],
    dtype=float,
)

# AP M1..M6 分别接到 MJCF rotor_0, rotor_2, rotor_5, rotor_1, rotor_3, rotor_4。
AP_TO_MUJOCO = np.array([0, 2, 5, 1, 3, 4], dtype=int)

# 当前模型静力平衡解。平均值是悬停油门，差值补偿机械臂/配重造成的质心偏移。
MUJOCO_HOVER_THRUSTS_N = np.array(
    [
        10.7133249216,
        10.5787244781,
        10.6759778841,
        10.6145162018,
        10.7482848071,
        10.6756516395,
    ],
    dtype=float,
)


@dataclass(frozen=True)
class MixResult:
    normalized_mujoco: np.ndarray
    thrust_newtons: np.ndarray
    rpy_scale: float
    forward_lateral_scale: float
    saturated: bool


class NCHexMotorMixer:
    """ArduPilot NCHex 分配矩阵及其优先级限幅。"""

    def __init__(self, max_thrust_n: float = 30.0, use_hover_trim: bool = True):
        self.max_thrust_n = float(max_thrust_n)
        hover_normalized = MUJOCO_HOVER_THRUSTS_N / self.max_thrust_n
        self.hover_throttle = float(np.mean(hover_normalized))
        # trim 只保留六个电机相对平均值的差，不改变总悬停油门。
        self.trim_mujoco = (
            hover_normalized - self.hover_throttle
            if use_hover_trim
            else np.zeros(6, dtype=float)
        )

    @staticmethod
    def _safe_ratio(numerator: float, denominator: float, fallback: float) -> float:
        if abs(denominator) < 1.0e-12:
            return fallback
        return numerator / denominator

    def mix(
        self,
        *,
        forward: float,
        lateral: float,
        throttle: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> MixResult:
        """把六自由度控制量变成 MuJoCo 六路牛顿推力。

        先缩放姿态项，再缩放平移项的策略与
        ``AP_MotorsNCHex::output_armed_stabilizing`` 一致。
        """

        throttle = float(np.clip(throttle, 0.0, 1.0))
        fl = ARDUPILOT_NCHEX_MATRIX[:, :2] @ np.array([forward, lateral])
        rpy = ARDUPILOT_NCHEX_MATRIX[:, 3:] @ np.array([roll, pitch, yaw])

        rpy_scale = 1.0
        fl_scale = 1.0
        eps = 0.001
        for index in range(6):
            output = fl[index] * fl_scale + rpy[index] * rpy_scale + throttle
            if -eps <= output <= 1.0 + eps:
                continue
            if output < -eps:
                if fl[index] >= 0.0:
                    rpy_scale = self._safe_ratio(
                        -(throttle + fl[index] * fl_scale), rpy[index], rpy_scale
                    )
                elif rpy[index] > 0.0:
                    fl_scale = self._safe_ratio(
                        -(throttle + rpy[index] * rpy_scale), fl[index], fl_scale
                    )
            elif output > 1.0 + eps:
                if fl[index] * fl_scale + throttle < 1.0:
                    rpy_scale = self._safe_ratio(
                        1.0 - throttle - fl[index] * fl_scale,
                        rpy[index],
                        rpy_scale,
                    )
                elif rpy[index] * rpy_scale + throttle < 1.0:
                    fl_scale = self._safe_ratio(
                        1.0 - throttle - rpy[index] * rpy_scale,
                        fl[index],
                        fl_scale,
                    )
            rpy_scale = float(np.clip(rpy_scale, 0.0, 1.0))
            fl_scale = float(np.clip(fl_scale, 0.0, 1.0))

        output_ap = np.clip(fl * fl_scale + rpy * rpy_scale + throttle, 0.0, 1.0)
        output_mujoco = np.empty(6, dtype=float)
        output_mujoco[AP_TO_MUJOCO] = output_ap
        output_mujoco = np.clip(output_mujoco + self.trim_mujoco, 0.0, 1.0)
        saturated = bool(
            rpy_scale < 0.999999
            or fl_scale < 0.999999
            or np.any(output_mujoco <= 1.0e-9)
            or np.any(output_mujoco >= 1.0 - 1.0e-9)
        )
        return MixResult(
            normalized_mujoco=output_mujoco,
            thrust_newtons=output_mujoco * self.max_thrust_n,
            rpy_scale=rpy_scale,
            forward_lateral_scale=fl_scale,
            saturated=saturated,
        )
