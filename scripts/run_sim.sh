#!/usr/bin/env bash
# sim2sim launcher. Brings up the ros2_control bridge for one simulator; the
# robot enters from its <robot>_description (URDF/xacro), a passthrough
# controller consumes command/joint_command from run_policy.sh.
#   ./run_sim.sh                                    # mujoco + d1
#   ./run_sim.sh --backend gazebo                   # switch simulator (mujoco | gazebo | webots)
#   ./run_sim.sh --backend webots --terrain stairs  # webots world worlds/<terrain>.wbt
#   ./run_sim.sh --robot <robot>                    # needs matching description + deploy packages
# gazebo/webots must be built first: ./scripts/setup.sh --with-gazebo | --with-webots
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BACKEND=mujoco
ROBOT=d1
TERRAIN=empty_world
while [ $# -gt 0 ]; do
    case "$1" in
        --backend) BACKEND="$2"; shift 2 ;;
        --robot)   ROBOT="$2";   shift 2 ;;
        --terrain) TERRAIN="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

[ -f "$REPO/install/setup.bash" ] || {
    echo "error: repo not built — run ./scripts/setup.sh first" >&2; exit 1; }
source "$REPO/env.sh" >/dev/null
export DISPLAY="${DISPLAY:-:0}"    # the sim opens a GUI window
# Config is a plain folder src/config/<robot>/ (no ROS package), resolved by path.
CONFIG_FILE="$REPO/src/config/$ROBOT/deploy.yaml"
[ -f "$CONFIG_FILE" ] || {
    echo "error: no config for robot '$ROBOT': expected $CONFIG_FILE" >&2
    exit 1
}

if [ "$BACKEND" = gazebo ]; then
    GAZEBO_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
    GAZEBO_PORT="${GAZEBO_URI##*:}"
    GAZEBO_PORT="${GAZEBO_PORT%%/*}"
    if [[ ! "$GAZEBO_PORT" =~ ^[0-9]+$ ]]; then
        echo "error: cannot determine Gazebo port from GAZEBO_MASTER_URI=$GAZEBO_URI" >&2
        exit 1
    fi
    if command -v ss >/dev/null &&
       ss -H -ltn "sport = :$GAZEBO_PORT" 2>/dev/null | grep -q .; then
        echo "error: Gazebo master port $GAZEBO_PORT is already in use." >&2
        ss -H -ltnp "sport = :$GAZEBO_PORT" >&2 || true
        echo "stop the old Gazebo process, or set a free GAZEBO_MASTER_URI." >&2
        echo "example: GAZEBO_MASTER_URI=http://127.0.0.1:11346 $0 --backend gazebo" >&2
        exit 1
    fi
fi

LAUNCH_ARGS=(robot:="$ROBOT" config_file:="$CONFIG_FILE")
if [ "$BACKEND" = webots ]; then LAUNCH_ARGS+=(terrain:="$TERRAIN"); fi   # terrain: webots only
exec ros2 launch "${BACKEND}_bridge" "${BACKEND}_bridge.launch.py" "${LAUNCH_ARGS[@]}"
