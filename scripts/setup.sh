#!/usr/bin/env bash
# Environment setup: pip deps + colcon build (bundled ddt_msgs). See README to run.
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -f /opt/ros/humble/setup.bash ] || {
    echo "error: ROS 2 Humble required (/opt/ros/humble not found)." >&2
    echo "  install: https://docs.ros.org/en/humble/Installation.html" >&2
    exit 1; }

echo "[1/2] installing pip deps..."
python3 -m pip install -r "$REPO/requirements.txt"
# ROS Humble binds against the system numpy; drop the pip one to avoid an ABI clash.
python3 -m pip uninstall -y numpy >/dev/null 2>&1 || true
python3 -c 'import numpy' 2>/dev/null || python3 -m pip install 'numpy>=1.24,<2'

echo "[2/2] colcon build (ddt_msgs)..."
source "$REPO/env.sh" >/dev/null
(cd "$REPO" && colcon build)

echo "setup complete."
