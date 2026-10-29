#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""在 MuJoCo 中运行 NCHex 闭环位置/姿态控制。"""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import time
import sys
import threading

import mujoco
import numpy as np

try:
    import mujoco.viewer
except ImportError:  # headless 测试时 viewer 不是必需品
    pass

from src.hover import FlightState, NCHexFlightController
from src.rotor_visual import RotorVisualAnimator
from src.command_interface import TargetCommandState


PROJECT_DIR = Path(__file__).resolve().parents[1]
MODEL_PATH = PROJECT_DIR / "model" / "nchex.xml"


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    result = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if result < 0:
        raise RuntimeError(f"模型缺少执行器: {name}")
    return result


def sensor_slice(model: mujoco.MjModel, name: str) -> slice:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sensor_id < 0:
        raise RuntimeError(f"模型缺少传感器: {name}")
    start = int(model.sensor_adr[sensor_id])
    return slice(start, start + int(model.sensor_dim[sensor_id]))


def run(args: argparse.Namespace) -> dict[str, float]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    rotor_ids = [actuator_id(model, f"rotor_{index}_thrust") for index in range(6)]
    delta_ids = [
        actuator_id(model, f"delta_motor{index}_position") for index in range(1, 4)
    ]
    gyro_data = sensor_slice(model, "base_angular_velocity")
    controller = NCHexFlightController(use_hover_trim=not args.no_hover_trim)
    rotor_visual = RotorVisualAnimator(model)
    target_commands = TargetCommandState(
        np.array([args.x, args.y, args.z], dtype=float),
        np.deg2rad(args.yaw_deg),
    )
    stop_requested = threading.Event()
    timestep = float(model.opt.timestep)
    maximum_error = 0.0
    last_output = None
    first_sample = True
    next_log_time = 0.0

    def control_once() -> None:
        nonlocal maximum_error, last_output, first_sample, next_log_time
        # qacc[:3] 是世界系惯性加速度；第一次 forward 尚未施加悬停推力，
        # 因此首次按静止状态初始化，避免垂向 D/加速度环的启动冲击。
        acceleration = np.zeros(3) if first_sample else data.qacc[:3].copy()
        first_sample = False
        state = FlightState(
            position_world=data.qpos[:3].copy(),
            velocity_world=data.qvel[:3].copy(),
            acceleration_world=acceleration,
            orientation_body_to_world=data.qpos[3:7].copy(),
            angular_velocity_body=data.sensordata[gyro_data].copy(),
        )
        target_position, target_yaw = target_commands.snapshot()
        last_output = controller.update(state, target_position, target_yaw, timestep)
        data.ctrl[rotor_ids] = last_output.motor_thrusts_n
        data.ctrl[delta_ids] = 0.0
        error_norm = float(np.linalg.norm(last_output.position.position_error))
        maximum_error = max(maximum_error, error_norm)
        if not args.quiet and data.time >= next_log_time:
            position_text = np.array2string(data.qpos[:3], precision=3)
            motor_text = np.array2string(last_output.motor_thrusts_n, precision=2)
            print(
                f"t={data.time:6.2f}s 位置={position_text} "
                f"误差={error_norm:.3f}m 电机N={motor_text}"
            )
            next_log_time += 0.5

    def finished() -> bool:
        return stop_requested.is_set() or (
            args.duration > 0.0 and data.time >= args.duration
        )

    def terminal_command_loop() -> None:
        print("\n仿真运行中，可在本终端发送飞行指令。")
        print("例如: forward 0.5   back 0.5   left 0.2   up 0.2   yaw 30")
        print("输入 help 查看全部指令，输入 quit 关闭仿真。")
        while not stop_requested.is_set():
            try:
                line = input("nchex> ")
            except EOFError:
                return
            try:
                response, should_quit = target_commands.execute(line)
            except ValueError as exc:
                print(f"指令错误: {exc}")
                continue
            if response:
                print(response)
            if should_quit:
                stop_requested.set()
                return

    if not args.headless and not getattr(args, "no_stdin", False):
        threading.Thread(
            target=terminal_command_loop,
            name="nchex-terminal-command",
            daemon=True,
        ).start()

    if args.headless:
        if args.duration <= 0.0:
            raise ValueError("无界面模式必须指定正的 --duration")
        while not finished():
            control_once()
            rotor_visual.advance(data, last_output.motor_thrusts_n, timestep)
            mujoco.mj_step(model, data)
    else:
        mujoco_major = int(mujoco.__version__.split(".", maxsplit=1)[0])
        if mujoco_major < 3:
            # 2.3.x 的 passive viewer 在后台运行 GLFW，Ctrl+C/再次启动时容易
            # 段错误。改为让 GUI 留在主线程，我们自己的 physics thread 在
            # Simulate 锁内运行控制和 step。这样不依赖 2.3.3 中不稳定的
            # Python mjcb_control，同时能在 SIGINT 时请求干净退出。
            simulate = mujoco.viewer.Simulate()
            previous_sigint_handler = signal.getsignal(signal.SIGINT)
            physics_error: list[BaseException] = []

            def request_viewer_exit(_signum=None, _frame=None) -> None:
                simulate.exitrequest = True

            def legacy_physics_loop() -> None:
                try:
                    # 给 _launch_internal 的模型加载线程一个很短的启动窗口。
                    time.sleep(0.05)
                    while not simulate.exitrequest and not finished():
                        frame_start = time.time()
                        with simulate.lock():
                            control_once()
                            rotor_visual.advance(
                                data, last_output.motor_thrusts_n, timestep
                            )
                            mujoco.mj_step(model, data)
                        remaining = timestep - (time.time() - frame_start)
                        if remaining > 0.0:
                            time.sleep(remaining)
                    if finished():
                        request_viewer_exit()
                except BaseException as exc:  # 把工作线程异常带回主线程。
                    physics_error.append(exc)
                    request_viewer_exit()

            signal.signal(signal.SIGINT, request_viewer_exit)
            physics_thread = threading.Thread(
                target=legacy_physics_loop,
                name="nchex-physics",
                daemon=False,
            )
            physics_thread.start()
            try:
                mujoco.viewer._launch_internal(
                    model,
                    data,
                    run_physics_thread=False,
                    simulate=simulate,
                )
            finally:
                request_viewer_exit()
                physics_thread.join(timeout=5.0)
                signal.signal(signal.SIGINT, previous_sigint_handler)
            if physics_error:
                raise physics_error[0]
        else:
            # 3.x 返回 ViewerHandle，可以由当前线程安全地 step/sync/close。
            viewer = mujoco.viewer.launch_passive(model, data)
            viewer.cam.distance = 2.2
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -22
            try:
                while viewer.is_running() and not finished():
                    frame_start = time.time()
                    control_once()
                    rotor_visual.advance(data, last_output.motor_thrusts_n, timestep)
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    remaining = timestep - (time.time() - frame_start)
                    if remaining > 0.0:
                        time.sleep(remaining)
            finally:
                viewer.close()

    final_target_position, _ = target_commands.snapshot()
    final_error = float(np.linalg.norm(data.qpos[:3] - final_target_position))
    final_tilt = (
        float(np.linalg.norm(last_output.attitude.attitude_error[:2]))
        if last_output is not None
        else 0.0
    )
    result = {
        "time_s": float(data.time),
        "final_error_m": final_error,
        "max_error_m": maximum_error,
        "final_tilt_rad": final_tilt,
        "max_speed_mps": float(np.linalg.norm(data.qvel[:3])),
    }
    print(
        "控制仿真结束: "
        f"t={result['time_s']:.2f}s, 最终位置误差={final_error:.4f}m, "
        f"最终倾斜误差={final_tilt:.4f}rad"
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="不打开窗口")
    parser.add_argument("--duration", type=float, default=0.0, help="运行秒数；0 表示持续运行")
    parser.add_argument("--x", type=float, default=0.0, help="世界系目标 x/m")
    parser.add_argument("--y", type=float, default=0.0, help="世界系目标 y/m")
    parser.add_argument("--z", type=float, default=0.62, help="世界系目标 z/m")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="目标偏航角/度")
    parser.add_argument("--no-hover-trim", action="store_true", help="关闭质心静态配平")
    parser.add_argument("--quiet", action="store_true", help="隐藏周期状态输出")
    parser.add_argument("--no-stdin", action="store_true", help="关闭运行时终端指令")
    return parser.parse_args()


if __name__ == "__main__":
    try:
        run(parse_args())
    except KeyboardInterrupt:
        print("\n已停止。")
    except Exception as exc:
        print(f"控制仿真失败: {exc}", file=sys.stderr)
        raise
