#!/usr/bin/env bash
# Usage: source env.sh
# Self-contained env: depends only on system ROS 2 Humble + the repo-built ddt_msgs.
# No external workspace required.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Strip conda from PATH so its python/libs do not shadow the system ROS ones.
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda' | paste -sd:)"
source /opt/ros/humble/setup.bash
[ -f "$REPO/install/setup.bash" ] && source "$REPO/install/setup.bash"
# The vendored MuJoCo sim bridge links pip mujoco's libmujoco.so by SONAME;
# put pip's lib dir on the loader path so mujoco_sim finds it at runtime.
MUJOCO_LIB="$(python3 -c 'import os, mujoco; print(os.path.dirname(mujoco.__file__))' 2>/dev/null)"
[ -n "$MUJOCO_LIB" ] && export LD_LIBRARY_PATH="$MUJOCO_LIB:${LD_LIBRARY_PATH:-}"
# Network isolation: loopback only, so other machines on the LAN sharing domain 0 do not cross-talk.
# For cross-network use with a real robot, comment out the next line and use a dedicated ROS_DOMAIN_ID.
export ROS_LOCALHOST_ONLY=1
echo "[ddt_rl_deploy] ROS2 Humble | ddt_msgs=$([ -f "$REPO/install/setup.bash" ] && echo built || echo 'NOT built (run colcon build)') | python3 -> $(which python3)"
