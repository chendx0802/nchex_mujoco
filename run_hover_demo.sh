#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${project_dir}/tools/python_env.sh"

exec "${python_bin}" "${project_dir}/tools/hover_demo.py"
