#!/usr/bin/env bash
# 一键环境准备:pip 依赖 + colcon build(只编仓库自带的 ddt_msgs)。
# 之后:终端1 ./scripts/run_sim.sh · 终端2 ./scripts/run_policy.sh · 终端3 ./scripts/run_teleop.sh
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -f /opt/ros/humble/setup.bash ] || {
    echo "错误: 需要 ROS 2 Humble(未找到 /opt/ros/humble)。" >&2
    echo "  安装: https://docs.ros.org/en/humble/Installation.html" >&2
    exit 1; }

echo "[1/3] 安装 pip 依赖..."
python3 -m pip install -r "$REPO/requirements.txt"
# ROS Humble 的 Python 绑定编译在系统 numpy 上;pip 顺带装的新 numpy 会 ABI 冲突。
# 卸掉 pip 的 numpy 回退到系统版;若系统确实没有,再装一个兼容版本。
python3 -m pip uninstall -y numpy >/dev/null 2>&1 || true
python3 -c 'import numpy' 2>/dev/null || python3 -m pip install 'numpy<1.25'

echo "[2/3] colcon build(ddt_msgs)..."
source "$REPO/env.sh" >/dev/null
(cd "$REPO" && colcon build)

echo "[3/3] 完成。运行:"
echo "  终端1: ./scripts/run_sim.sh                    # 本地 Mujoco 仿真"
echo "  终端2: ./scripts/run_policy.sh rl_flat_lab d1  # 策略(--list 看可选)"
echo "  终端3: ./scripts/run_teleop.sh                 # 键盘遥控"
