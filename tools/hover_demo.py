#!/usr/bin/env python3
"""Open the model and apply a simple open-loop hover thrust test."""

from pathlib import Path
import signal
import sys
import time
import threading

import mujoco
import mujoco.viewer


MODEL_PATH = Path(__file__).resolve().parents[1] / "model" / "nchex.xml"
CONTROL_PATH = Path(__file__).resolve().parents[1] / "control"
sys.path.insert(0, str(CONTROL_PATH))

from src.rotor_visual import RotorVisualAnimator


ROTOR_COUNT = 6
# Static allocation result for the current mass, COM and tilted rotor geometry.
# The values are intentionally not identical: they compensate the small
# gravity moment caused by the model COM offset from base_link.
HOVER_THRUSTS_N = (
    10.7133249216,
    10.5787244781,
    10.6759778841,
    10.6145162018,
    10.7482848071,
    10.6756516395,
)


model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)
rotor_visual = RotorVisualAnimator(model)

for index in range(ROTOR_COUNT):
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"rotor_{index}_thrust")
    data.ctrl[actuator_id] = HOVER_THRUSTS_N[index]
for joint_name in ("delta_motor1", "delta_motor2", "delta_motor3"):
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint_name}_position")
    data.ctrl[actuator_id] = 0.0

mujoco_major = int(mujoco.__version__.split(".", maxsplit=1)[0])
if mujoco_major < 3:
    simulate = mujoco.viewer.Simulate()
    previous_sigint_handler = signal.getsignal(signal.SIGINT)
    physics_error = []

    def request_viewer_exit(_signum=None, _frame=None) -> None:
        simulate.exitrequest = True

    def physics_loop() -> None:
        try:
            time.sleep(0.05)
            while not simulate.exitrequest:
                start = time.time()
                with simulate.lock():
                    rotor_visual.advance(
                        data, HOVER_THRUSTS_N, float(model.opt.timestep)
                    )
                    mujoco.mj_step(model, data)
                remaining = model.opt.timestep - (time.time() - start)
                if remaining > 0:
                    time.sleep(remaining)
        except BaseException as exc:
            physics_error.append(exc)
            request_viewer_exit()

    signal.signal(signal.SIGINT, request_viewer_exit)
    physics_thread = threading.Thread(
        target=physics_loop,
        name="nchex-hover-physics",
        daemon=False,
    )
    physics_thread.start()
    try:
        # GLFW 留在主线程；物理线程通过 Simulate 锁和渲染同步。
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
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.cam.distance = 2.2
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -22
    try:
        while viewer.is_running():
            start = time.time()
            rotor_visual.advance(data, HOVER_THRUSTS_N, float(model.opt.timestep))
            mujoco.mj_step(model, data)
            viewer.sync()
            remaining = model.opt.timestep - (time.time() - start)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        viewer.close()
