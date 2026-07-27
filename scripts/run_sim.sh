#!/usr/bin/env bash
# sim2sim launcher. Brings up the ros2_control bridge for one simulator; the
# robot enters from its <robot>_description (URDF/xacro), a passthrough
# controller consumes command/joint_command from run_policy.sh.
#   ./run_sim.sh                      # mujoco + d1
#   ./run_sim.sh --backend gazebo     # switch simulator (mujoco | gazebo | webots)
#   ./run_sim.sh --robot d1h          # switch robot (needs src/robot/<robot>_description)
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BACKEND=mujoco
ROBOT=d1
while [ $# -gt 0 ]; do
    case "$1" in
        --backend) BACKEND="$2"; shift 2 ;;
        --robot)   ROBOT="$2";   shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

[ -f "$REPO/install/setup.bash" ] || {
    echo "error: repo not built — run ./scripts/setup.sh first" >&2; exit 1; }
source "$REPO/env.sh" >/dev/null
export DISPLAY="${DISPLAY:-:0}"    # the sim opens a GUI window

exec ros2 launch "${BACKEND}_bridge" "${BACKEND}_bridge.launch.py" robot:="$ROBOT"
