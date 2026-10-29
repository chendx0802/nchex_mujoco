#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""无界面直接设置六路旋翼推力和三路 Delta 关节目标，不运行位置控制器。"""

from pathlib import Path
import argparse
import math
import sys

import mujoco
import numpy as np

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))
from control.src.rotors_deliver import MUJOCO_HOVER_THRUSTS_N
from control.src.rotor_visual import RotorVisualAnimator


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=1.0, help="仿真秒数，必须为正")
    parser.add_argument(
        "--rotor-thrusts", type=float, nargs=6,
        default=MUJOCO_HOVER_THRUSTS_N.tolist(),
        metavar=("T0", "T1", "T2", "T3", "T4", "T5"),
        help="rotor_0 到 rotor_5 推力，单位 N，各项范围 0 到 30",
    )
    parser.add_argument(
        "--delta-joints", type=float, nargs=3, default=[0.0, 0.0, 0.0],
        metavar=("Q1", "Q2", "Q3"),
        help="delta_motor1 到 delta_motor3 位置目标，单位 rad，各项范围 -1.1 到 1.1",
    )
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration 必须是有限正数")
    thrusts = np.asarray(args.rotor_thrusts, dtype=float)
    delta = np.asarray(args.delta_joints, dtype=float)
    if not np.all(np.isfinite(thrusts)) or np.any((thrusts < 0) | (thrusts > 30)):
        parser.error("--rotor-thrusts 各项必须为 0 到 30 N 的有限数值")
    if not np.all(np.isfinite(delta)) or np.any(np.abs(delta) > 1.1):
        parser.error("--delta-joints 各项必须为 -1.1 到 1.1 rad 的有限数值")

    model = mujoco.MjModel.from_xml_path(str(PROJECT_DIR / "model" / "nchex.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rotor_ids = [model.actuator(f"rotor_{i}_thrust").id for i in range(6)]
    delta_ids = [model.actuator(f"delta_motor{i}_position").id for i in range(1, 4)]
    visual = RotorVisualAnimator(model)

    # data.ctrl 的索引必须通过执行器名称查找，不能把关节编号当作执行器编号。
    data.ctrl[rotor_ids] = thrusts
    data.ctrl[delta_ids] = delta
    while data.time < args.duration:
        visual.advance(data, thrusts, float(model.opt.timestep))
        mujoco.mj_step(model, data)

    print(f"仿真时间: {data.time:.3f} s")
    print(f"旋翼指令 rotor_0..5 (N): {data.ctrl[rotor_ids]}")
    print(f"Delta 关节目标 1..3 (rad): {data.ctrl[delta_ids]}")
    print(f"最终机体位置 (m): {data.qpos[:3]}")
    for i in range(1, 4):
        joint_id = model.joint(f"delta_motor{i}").id
        angle = data.qpos[model.jnt_qposadr[joint_id]]
        print(f"delta_motor{i} 实际角度: {angle:.6f} rad")


if __name__ == "__main__":
    main()
