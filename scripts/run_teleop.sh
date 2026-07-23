#!/usr/bin/env bash
# 键盘遥控(sim2sim / sim2real 通用)
#   w/s 前后 · a/d 转向 · q/e 平移 · r/f 升降 · 空格停 · x 退出
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/env.sh" >/dev/null
exec python3 "$REPO/scripts/teleop.py" "$@"
