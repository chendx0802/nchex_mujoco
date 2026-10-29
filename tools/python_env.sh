#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# 由项目根目录的启动脚本 source；project_dir 由调用者设置。
if [[ -n "${MUJOCO_PYTHON:-}" ]]; then
  python_bin="${MUJOCO_PYTHON}"
elif [[ -x "${project_dir}/.venv/bin/python" ]]; then
  python_bin="${project_dir}/.venv/bin/python"
else
  python_bin="python3"
fi

if ! command -v "${python_bin}" >/dev/null 2>&1; then
  echo "找不到 Python: ${python_bin}" >&2
  echo "请运行 ./setup_env.sh，或设置 MUJOCO_PYTHON=/path/to/python。" >&2
  exit 1
fi

if ! "${python_bin}" -c 'import mujoco, numpy' >/dev/null 2>&1; then
  echo "${python_bin} 缺少 MuJoCo / NumPy，或依赖无法加载。" >&2
  echo "请运行 ./setup_env.sh，或为指定解释器安装 requirements.txt。" >&2
  exit 1
fi
