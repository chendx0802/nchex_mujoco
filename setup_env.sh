#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -m venv "${project_dir}/.venv"
"${project_dir}/.venv/bin/python" -m pip install --upgrade pip
"${project_dir}/.venv/bin/python" -m pip install -r "${project_dir}/requirements.txt"

echo "MuJoCo environment is ready: ${project_dir}/.venv"
echo "运行模型检查: ${project_dir}/.venv/bin/python ${project_dir}/tools/validate_model.py"
echo "运行闭环演示: ${project_dir}/run_control_demo.sh"
