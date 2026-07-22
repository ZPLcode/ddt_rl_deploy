#!/usr/bin/env bash
# RL 策略一键启动(上层部署 App)
#
#   ./run_policy.sh                          # d1 + rl_height_cargo_out(默认)
#   ./run_policy.sh rl_flat_lab              # 换策略
#   ./run_policy.sh rl_flat_lab d1h          # 换机器人
#   ./run_policy.sh rl_height_cargo_out d1 -p key:=val   # 额外 ROS 参数透传
#
# 依赖:已 build 的 ddt_ros2_ws(提供 ddt_msgs 等)。用 DDT_WS 覆盖其路径。
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${DDT_WS:-/home/zhepeng/DDT/ddt_ros2_ws}"

POLICY="${1:-rl_height_cargo_out}"
ROBOT="${2:-d1}"
shift $(( $# > 2 ? 2 : $# )) || true

source "$WS/ddt_env.sh" >/dev/null

YAML="$REPO/config/$ROBOT/controllers.yaml"
if [ ! -f "$YAML" ]; then
    echo "错误: 找不到配置 $YAML" >&2
    exit 1
fi

echo "[ddt_rl_deploy] robot=$ROBOT policy=$POLICY"
exec python3 "$REPO/deploy/rl_inference.py" --ros-args \
    -p config_file:="$YAML" -p policy_name:="$POLICY" "$@"
