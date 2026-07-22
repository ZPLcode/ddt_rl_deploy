#!/usr/bin/env bash
# 轻量 Mujoco 仿真(sim2sim 后端,纯 DDS,自包含)
#   ./run_sim.sh                          # 裸机 d1(scene.xml),带 GUI
#   ./run_sim.sh --no-viewer              # 无头
#   ./run_sim.sh --scene /path/to/other.xml   # 换别的 mjcf
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/env.sh" >/dev/null
exec python3 "$REPO/sim/mujoco_sim.py" "$@"
