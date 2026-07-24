#!/usr/bin/env bash
# Webots sim2sim backend entrypoint (auto-discovered by run_sim.sh --backend webots).
# Starts Webots + the extern controller (webots_sim.py, per the topic contract).
#   headless by default (--no-rendering --minimize --batch);
#   add --gui for a visible Webots window (visual debugging):
#     ./scripts/run_sim.sh --backend webots --gui
# Needs a Webots install: defaults to $HOME/webots, or set WEBOTS_HOME.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

export WEBOTS_HOME="${WEBOTS_HOME:-$HOME/webots}"
[ -x "$WEBOTS_HOME/webots" ] || {
    echo "error: Webots not found ($WEBOTS_HOME/webots)." >&2
    echo "  Download and extract Webots, then set WEBOTS_HOME (or put it in ~/webots)." >&2
    exit 1; }

# ROS + ddt_msgs (run_sim.sh already sources this; idempotent, so this also works standalone)
source "$REPO/env.sh" >/dev/null

# --gui toggles rendering; --robot picks the world (worlds/<robot>.wbt); other args pass through to the controller
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
    echo "error: missing $WORLD -- this robot has no Webots world/proto yet." >&2
    echo "  Base one on worlds/d1.wbt + protos/D1.proto." >&2
    exit 1; }
LOG="${TMPDIR:-/tmp}/webots_ddt.$$.log"

if [ "$GUI" = 1 ]; then
    echo "[webots] GUI mode: NVIDIA GPU hardware render (PRIME offload), realtime without frame drops"
    # Hybrid-GPU laptop (Intel iGPU + NVIDIA dGPU): offload the GL context to the dGPU.
    # Without a dGPU or its driver, comment out these two lines to fall back to the iGPU (still hardware GL); do not use LIBGL_ALWAYS_SOFTWARE.
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
echo "[webots] starting up (log: $LOG)... waiting for the simulation to become ready"
sleep 5

# The extern controller connects to Webots; webots-controller prepends the Webots
# Python controller library to PYTHONPATH. ROS rclpy/ddt_msgs are already on the env, so the two coexist.
exec "$WEBOTS_HOME/webots-controller" "$HERE/webots_sim.py" "${ARGS[@]}"
