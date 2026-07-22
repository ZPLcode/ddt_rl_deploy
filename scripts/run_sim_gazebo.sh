#!/usr/bin/env bash
# Gazebo 仿真(推荐:gzserver/gzclient 分进程,渲染不干扰状态流)
#   ./run_sim_gazebo.sh            # d1，带 GUI
#   ./run_sim_gazebo.sh d1 gui:=false   # 无头
set -eo pipefail
WS="${DDT_WS:-/home/zhepeng/DDT/ddt_ros2_ws}"
ROBOT="${1:-d1}"; shift || true
source "$WS/ddt_env.sh" >/dev/null
exec ros2 launch topic_command_controller sim_gazebo_topic.launch.py robot:="$ROBOT" "$@"
