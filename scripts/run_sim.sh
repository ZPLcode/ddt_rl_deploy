#!/usr/bin/env bash
# sim2sim 仿真后端启动器。后端只需讲 sim/BACKEND.md 的 topic 契约,deploy 不变。
#   ./run_sim.sh                          # 默认 mujoco 后端(GUI)
#   ./run_sim.sh --backend webots --gui   # 换后端(webots GUI 走独显渲染)
#   ./run_sim.sh --robot d1cargo_out      # 换机型(透传给后端,自动找对应模型)
#   ./run_sim.sh --no-viewer              # 其余参数一律透传给后端
#   ./run_sim.sh --list-backends          # 列出已装好的后端
#
# 后端发现规则(加后端 = 丢个文件,不用改本脚本):
#   sim/<name>_sim.py   -> python3 跑它            (轻量后端,如 mujoco)
#   sim/<name>/run.sh   -> 执行它                  (框架后端,如 webots)
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

list_backends() {
    for f in "$REPO"/sim/*_sim.py; do
        [ -e "$f" ] || continue; b="$(basename "$f")"; echo "  ${b%_sim.py}"
    done
    for d in "$REPO"/sim/*/run.sh; do
        [ -e "$d" ] || continue; echo "  $(basename "$(dirname "$d")")"
    done
}

BACKEND="mujoco"
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --backend)   BACKEND="$2"; shift 2 ;;
        --backend=*) BACKEND="${1#*=}"; shift ;;
        --list-backends) echo "已装后端:"; list_backends; exit 0 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done

[ -f "$REPO/install/setup.bash" ] || {
    echo "错误: 仓库还没 build(缺 install/)—— 先运行 ./scripts/setup.sh" >&2; exit 1; }
source "$REPO/env.sh" >/dev/null

PY="$REPO/sim/${BACKEND}_sim.py"
SH="$REPO/sim/${BACKEND}/run.sh"
if [ -f "$PY" ]; then
    exec python3 "$PY" "${ARGS[@]}"
elif [ -x "$SH" ]; then
    exec "$SH" "${ARGS[@]}"
else
    echo "错误: 未找到后端 '$BACKEND'" >&2
    echo "已装后端:" >&2; list_backends >&2
    echo "新增后端见 sim/BACKEND.md" >&2
    exit 1
fi
