#!/usr/bin/env bash
# One-shot env setup: pip deps + colcon build (builds only the bundled ddt_msgs).
# Then: term1 ./scripts/run_sim.sh · term2 ./scripts/run_policy.sh · term3 ./scripts/run_teleop.sh
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -f /opt/ros/humble/setup.bash ] || {
    echo "error: ROS 2 Humble required (/opt/ros/humble not found)." >&2
    echo "  install: https://docs.ros.org/en/humble/Installation.html" >&2
    exit 1; }

echo "[1/3] installing pip deps..."
# If pip aborts with "externally-managed-environment" (PEP 668): re-run with --break-system-packages, or use a venv.
python3 -m pip install -r "$REPO/requirements.txt"
# ROS Humble's Python bindings are built against the system numpy; a newer pip numpy would ABI-clash.
# Drop the pip numpy to fall back to the system one; install a Humble-compatible numpy only if none is left.
python3 -m pip uninstall -y numpy >/dev/null 2>&1 || true
python3 -c 'import numpy' 2>/dev/null || python3 -m pip install 'numpy>=1.24,<2'

echo "[2/3] colcon build (ddt_msgs)..."
source "$REPO/env.sh" >/dev/null
(cd "$REPO" && colcon build)

echo "[3/3] done. run:"
echo "  term1: ./scripts/run_sim.sh                    # local Mujoco sim"
echo "  term2: ./scripts/run_policy.sh rl_flat_lab d1  # policy (--list for options)"
echo "  term3: ./scripts/run_teleop.sh                 # keyboard teleop"
