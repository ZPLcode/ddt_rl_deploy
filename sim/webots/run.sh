#!/usr/bin/env bash
# Webots sim2sim 后端入口(被 run_sim.sh --backend webots 自动发现)。
# 起 Webots + extern 控制器(webots_sim.py,讲 BACKEND.md 契约)。
#   默认无头(--no-rendering --minimize --batch);
#   加 --gui 显示 Webots 窗口用于可视化调试:
#     ./scripts/run_sim.sh --backend webots --gui
# 需要 Webots 本体:默认 $HOME/webots,或设 WEBOTS_HOME。
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

export WEBOTS_HOME="${WEBOTS_HOME:-$HOME/webots}"
[ -x "$WEBOTS_HOME/webots" ] || {
    echo "错误: 未找到 Webots($WEBOTS_HOME/webots)。" >&2
    echo "  下载解压 Webots 后设 WEBOTS_HOME,或放到 ~/webots。" >&2
    exit 1; }

# ROS + ddt_msgs(run_sim.sh 已 source,这里幂等,便于单独跑)
source "$REPO/env.sh" >/dev/null

# --gui 切可视化;--robot 选 world(worlds/<robot>.wbt);其余参数透传给控制器
GUI=0
ROBOT="d1"
ARGS=()
prev=""
for a in "$@"; do
    if [ "$a" = "--gui" ]; then GUI=1
    else
        [ "$prev" = "--robot" ] && ROBOT="$a"
        ARGS+=("$a")
    fi
    prev="$a"
done

WORLD="$HERE/worlds/$ROBOT.wbt"
[ -f "$WORLD" ] || {
    echo "错误: 缺 $WORLD —— 该机型还没有 Webots world/proto。" >&2
    echo "  参考 worlds/d1.wbt + protos/D1.proto 生成(见 docs/ADD_ROBOT.md)。" >&2
    exit 1; }
LOG="${TMPDIR:-/tmp}/webots_ddt.$$.log"

if [ "$GUI" = 1 ]; then
    echo "[webots] GUI 模式:NVIDIA GPU 硬件渲染(PRIME offload),realtime 不掉帧"
    # 混合显卡笔记本(Intel 核显 + NVIDIA 独显):把 GL 上下文 offload 到独显。
    # 若无独显/驱动,可注释掉这两行退回核显(仍是硬件 GL);别用 LIBGL_ALWAYS_SOFTWARE。
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export DISPLAY="${DISPLAY:-:0}"
    "$WEBOTS_HOME/webots" --mode=realtime --stdout --stderr "$WORLD" >"$LOG" 2>&1 &
else
    "$WEBOTS_HOME/webots" --batch --mode=realtime --no-rendering --minimize \
        --stdout --stderr "$WORLD" >"$LOG" 2>&1 &
fi
WB=$!
trap 'kill $WB 2>/dev/null' EXIT INT TERM
echo "[webots] 启动中(日志 $LOG)... 等仿真就绪"
sleep 5

# extern 控制器连上 Webots;webots-controller 会把 Webots 的 python 控制器库
# 前置到 PYTHONPATH,ROS 的 rclpy/ddt_msgs 已在 env 里,两者共存。
exec "$WEBOTS_HOME/webots-controller" "$HERE/webots_sim.py" "${ARGS[@]}"
