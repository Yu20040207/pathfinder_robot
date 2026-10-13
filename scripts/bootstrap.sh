#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
/usr/bin/python3 -m venv --system-site-packages "$root/.venv"
PIP_CONFIG_FILE=/dev/null PIP_INDEX_URL="${FWR_PIP_INDEX_URL:-https://pypi.org/simple}" \
  "$root/.venv/bin/python" -m pip install -r "$root/requirements-runtime.txt" -r "$root/requirements-dev.txt"
echo "Source /opt/ros/humble/setup.bash, then .venv/bin/activate"
