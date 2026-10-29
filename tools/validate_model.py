#!/usr/bin/env python3
"""Compile the generated MuJoCo model and print its dimensions."""

from pathlib import Path
import sys

try:
    import mujoco
except ImportError:
    print("mujoco is not installed; run ./setup_env.sh first", file=sys.stderr)
    raise SystemExit(2)


model_path = Path(__file__).resolve().parents[1] / "model" / "nchex.xml"
model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

print(f"PASS: {model_path}")
print(f"bodies={model.nbody} joints={model.njnt} dof={model.nv}")
print(f"actuators={model.nu} equality_constraints={model.neq} sensors={model.nsensor}")
print(f"initial_time={data.time:.3f} initial_qpos={model.nq}")
