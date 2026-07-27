#!/usr/bin/env bash
# Environment setup: pip deps + colcon build. See README to run.
#   ./setup.sh                 # default: self-contained, mujoco backend only
#   ./setup.sh --with-gazebo   # also build gazebo_bridge (needs gazebo classic + ros-humble-gazebo-ros2-control)
#   ./setup.sh --with-webots   # also build webots_bridge (needs Webots R2025a + ros-humble-webots-ros2)
#   ./setup.sh --with-all      # build all three backends
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Heavy sim backends are OFF by default so the default build stays self-contained
# (only pip mujoco). Opt in per backend; each needs its simulator + ROS packages
# installed or colcon fails on the missing find_package.
WITH_GAZEBO=0
WITH_WEBOTS=0
for arg in "$@"; do
    case "$arg" in
        --with-gazebo) WITH_GAZEBO=1 ;;
        --with-webots) WITH_WEBOTS=1 ;;
        --with-all)    WITH_GAZEBO=1; WITH_WEBOTS=1 ;;
        *) echo "unknown arg: $arg (use --with-gazebo | --with-webots | --with-all)" >&2; exit 1 ;;
    esac
done
IGNORE=()
BACKENDS="mujoco"
if [ "$WITH_GAZEBO" = 1 ]; then BACKENDS="$BACKENDS +gazebo"; else IGNORE+=(gazebo_bridge); fi
if [ "$WITH_WEBOTS" = 1 ]; then BACKENDS="$BACKENDS +webots"; else IGNORE+=(webots_bridge); fi

[ -f /opt/ros/humble/setup.bash ] || {
    echo "error: ROS 2 Humble required (/opt/ros/humble not found)." >&2
    echo "  install: https://docs.ros.org/en/humble/Installation.html" >&2
    exit 1; }

echo "[1/2] installing pip deps..."
python3 -m pip install -r "$REPO/requirements.txt"
# ROS Humble binds against the system numpy; drop the pip one to avoid an ABI clash.
python3 -m pip uninstall -y numpy >/dev/null 2>&1 || true
python3 -c 'import numpy' 2>/dev/null || python3 -m pip install 'numpy>=1.24,<2'

echo "[2/2] colcon build (backends: $BACKENDS)..."
source "$REPO/env.sh" >/dev/null
IGNORE_ARG=()
if [ ${#IGNORE[@]} -gt 0 ]; then IGNORE_ARG=(--packages-ignore "${IGNORE[@]}"); fi
# mujoco_bridge links pip MuJoCo directly and reuses the bundled lodepng, so
# its build needs neither network access nor a separate MuJoCo source tree.
(cd "$REPO" && colcon build "${IGNORE_ARG[@]}" \
    --cmake-args -DFETCHCONTENT_SOURCE_DIR_LODEPNG="$REPO/vendor/lodepng")

echo "setup complete."
