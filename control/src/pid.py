# SPDX-License-Identifier: GPL-3.0-or-later
"""ArduPilot ``AC_PID`` 的轻量 Python 移植。

这里只保留 NCHex 位置环和姿态环实际用到的部分：一阶低通、P/I/D、
积分限幅以及执行器饱和时的抗积分饱和。算法来源见 ``control/README.md``。
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PIDConfig:
    """PID 参数；单位由调用者决定，输出与 ArduPilot 一样不自动归一化。"""

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    imax: float = math.inf
    target_filter_hz: float = 0.0
    error_filter_hz: float = 0.0
    derivative_filter_hz: float = 0.0
    output_limit: float = math.inf


def lowpass_alpha(dt: float, cutoff_hz: float) -> float:
    """返回一阶低通系数；截止频率为 0 时与 ArduPilot 一样不过滤。"""

    if dt <= 0.0 or cutoff_hz <= 0.0:
        return 1.0
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (dt + rc)


class ArduPilotPID:
    """复现 ``AC_PID::update_all`` 的核心行为。"""

    def __init__(self, config: PIDConfig):
        self.config = config
        self.integrator = 0.0
        self.target = 0.0
        self.error = 0.0
        self.derivative = 0.0
        self._reset_filter = True
        self.last_output = 0.0

    def reset(self, integrator: float = 0.0) -> None:
        self.integrator = float(
            max(-self.config.imax, min(self.config.imax, integrator))
        )
        self.target = 0.0
        self.error = 0.0
        self.derivative = 0.0
        self.last_output = 0.0
        self._reset_filter = True

    def update(
        self,
        target: float,
        measurement: float,
        dt: float,
        saturated: bool = False,
    ) -> float:
        """更新 PID。

        ``saturated`` 对应 ArduPilot 的 ``limit``：饱和时积分只能朝减小绝对值的
        方向变化，避免飞行器翻倒后积分继续累积。
        """

        if not math.isfinite(target) or not math.isfinite(measurement):
            return 0.0

        if self._reset_filter:
            self.target = target
            self.error = target - measurement
            self.derivative = 0.0
            self._reset_filter = False
        else:
            previous_error = self.error
            self.target += lowpass_alpha(
                dt, self.config.target_filter_hz
            ) * (target - self.target)
            raw_error = self.target - measurement
            self.error += lowpass_alpha(
                dt, self.config.error_filter_hz
            ) * (raw_error - self.error)
            if dt > 0.0:
                raw_derivative = (self.error - previous_error) / dt
                self.derivative += lowpass_alpha(
                    dt, self.config.derivative_filter_hz
                ) * (raw_derivative - self.derivative)

        # 对应 AC_PID::update_i：饱和时只允许积分回到零附近。
        can_grow = not saturated or (
            (self.integrator > 0.0 and self.error < 0.0)
            or (self.integrator < 0.0 and self.error > 0.0)
        )
        if self.config.ki != 0.0 and dt > 0.0 and can_grow:
            self.integrator += self.error * self.config.ki * dt
            self.integrator = max(
                -self.config.imax, min(self.config.imax, self.integrator)
            )

        output = (
            self.config.kp * self.error
            + self.integrator
            + self.config.kd * self.derivative
        )
        output = max(
            -self.config.output_limit,
            min(self.config.output_limit, output),
        )
        self.last_output = output
        return output
