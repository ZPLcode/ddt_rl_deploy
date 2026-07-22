#!/usr/bin/env bash
# Mujoco 仿真(ros2_control 版,厂商 mujoco_ros2_control 桥)
# 注意:此桥物理+发布+渲染同循环,GUI 下 joint_states 会成串(~17ms 卡顿)。
# 想要平顺请用 run_sim_gazebo.sh。
#   ./run_sim_mujoco.sh [robot]
set -eo pipefail
WS="${DDT_WS:-/home/zhepeng/DDT/ddt_ros2_ws}"
ROBOT="${1:-d1}"
source "$WS/ddt_env.sh" >/dev/null
exec ros2 launch topic_command_controller sim_mujoco_topic.launch.py robot:="$ROBOT"
