# SPDX-License-Identifier: GPL-3.0-or-later
"""MuJoCo 螺旋桨关节的视觉转动。

飞行模型使用力级执行器：控制量已经是牛顿推力，偏航反扭矩也已经包含在
``general`` actuator 的 gear 中。因此这里只更新桨叶关节角度，不能再通过关节
电机施加一次力矩，否则会把偏航反扭矩计算两遍。
"""

from __future__ import annotations

import math
import mujoco
import numpy as np


# 来自模型 allocation_matrix.csv：-1/1 分别表示两种相反旋向。
ROTOR_SPIN_DIRECTIONS = np.array([-1.0, 1.0, 1.0, -1.0, 1.0, -1.0])


class RotorVisualAnimator:
    """根据推力平方根更新六个 rotor hinge 的显示角度。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        *,
        max_thrust_n: float = 30.0,
        max_visual_speed_rad_s: float = 55.0,
    ):
        self.max_thrust_n = float(max_thrust_n)
        # 真机桨速会高得多；显示速度特意降低，避免 60 Hz 画面出现严重混叠。
        self.max_visual_speed_rad_s = float(max_visual_speed_rad_s)
        qpos_addresses = []
        for index in range(6):
            joint_name = f"rotor_{index}_joint"
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            if joint_id < 0:
                raise RuntimeError(f"模型缺少螺旋桨关节: {joint_name}")
            qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
        self.qpos_addresses = np.asarray(qpos_addresses, dtype=int)

    def advance(
        self,
        data: mujoco.MjData,
        thrusts_n: np.ndarray,
        dt: float,
    ) -> None:
        normalized_thrust = np.clip(
            np.asarray(thrusts_n, dtype=float) / self.max_thrust_n, 0.0, 1.0
        )
        angular_speed = (
            ROTOR_SPIN_DIRECTIONS
            * self.max_visual_speed_rad_s
            * np.sqrt(normalized_thrust)
        )
        angles = data.qpos[self.qpos_addresses] + angular_speed * dt
        # 角度保持在 [-pi, pi]，避免长时间仿真后数值无限增大。
        data.qpos[self.qpos_addresses] = (angles + math.pi) % (2.0 * math.pi) - math.pi
