#!/usr/bin/env bash
# sim2sim backend launcher. Default mujoco; --backend webots to switch.
# Other args pass through to the backend (--gui / --no-viewer / --robot ...).
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BACKEND=mujoco
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --backend) BACKEND="$2"; shift 2 ;;
        *)         ARGS+=("$1"); shift ;;
    esac
done

[ -f "$REPO/install/setup.bash" ] || {
    echo "error: repo not built — run ./scripts/setup.sh first" >&2; exit 1; }
source "$REPO/env.sh" >/dev/null

PY="$REPO/sim/${BACKEND}_sim.py"
SH="$REPO/sim/${BACKEND}/run.sh"
if   [ -f "$PY" ]; then exec python3 "$PY" "${ARGS[@]}"
elif [ -x "$SH" ]; then exec "$SH" "${ARGS[@]}"
else echo "error: backend '$BACKEND' not found (mujoco, webots)" >&2; exit 1
fi
