#!/usr/bin/env bash
# 用法: source env.sh
# 自包含环境:只依赖系统 ROS 2 Humble + 本仓库自己 build 的 ddt_msgs。
# 不依赖任何外部工作空间。
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 剥掉 conda,避免它的 python/库遮住系统 ROS 的
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda' | paste -sd:)"
source /opt/ros/humble/setup.bash
[ -f "$REPO/install/setup.bash" ] && source "$REPO/install/setup.bash"
# 网络隔离:只走本机回环,防止局域网其他机器共用 domain0 串扰。
# 跨网连真机时注释掉下一行,改用独占 ROS_DOMAIN_ID。
export ROS_LOCALHOST_ONLY=1
echo "[ddt_rl_deploy] ROS2 Humble | ddt_msgs=$([ -f "$REPO/install/setup.bash" ] && echo built || echo 'NOT built(先 colcon build)') | python3 -> $(which python3)"
