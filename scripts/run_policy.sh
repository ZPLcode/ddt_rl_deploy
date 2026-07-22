#!/usr/bin/env bash
# RL 策略部署(上层 App,讲 topic → sim2sim 与 sim2real 通用)
#   ./run_policy.sh                          # rl_flat_lab + d1(默认)
#   ./run_policy.sh rl_flat_lab              # 换策略
#   ./run_policy.sh rl_flat_lab d1 -p k:=v   # 额外 ROS 参数透传
#
# 自包含:只需系统 ROS 2 + 本仓库 build 的 ddt_msgs。sim2sim 就配 run_sim.sh;
# sim2real 就直接跑本脚本,对面是真机(代码不变)。
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

POLICY="${1:-rl_flat_lab}"
ROBOT="${2:-d1}"
shift $(( $# > 2 ? 2 : $# )) || true

source "$REPO/env.sh" >/dev/null

YAML="$REPO/config/$ROBOT/controllers.yaml"
[ -f "$YAML" ] || { echo "错误: 找不到配置 $YAML" >&2; exit 1; }

echo "[ddt_rl_deploy] robot=$ROBOT policy=$POLICY"
exec python3 "$REPO/deploy/rl_inference.py" --ros-args \
    -p config_file:="$YAML" -p policy_name:="$POLICY" "$@"
