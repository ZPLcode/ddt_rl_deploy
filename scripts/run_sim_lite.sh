#!/usr/bin/env bash
# 轻量 Mujoco 仿真(无 ros2_control,纯 DDS,照云深处参考实现)
# 渲染用 passive viewer(不阻塞物理),状态流规律。适合无 ros2_control 环境。
#   ./run_sim_lite.sh                 # 带 GUI
#   ./run_sim_lite.sh --no-viewer     # 无头
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${DDT_WS:-/home/zhepeng/DDT/ddt_ros2_ws}"
source "$WS/ddt_env.sh" >/dev/null
exec python3 "$REPO/sim/mujoco_sim.py" "$@"
