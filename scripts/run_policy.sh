#!/usr/bin/env bash
# RL policy deploy 
#   ./run_policy.sh [policy] [robot]   # defaults: rl_flat_lab d1
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -f "$REPO/install/setup.bash" ] || {
    echo "error: repo not built — run ./scripts/setup.sh first" >&2; exit 1; }

POLICY="${1:-rl_flat_lab}"
ROBOT="${2:-d1}"
shift $(( $# > 2 ? 2 : $# )) || true
source "$REPO/env.sh" >/dev/null

CONFIG_PACKAGE="${ROBOT}_deploy"
if ! CONFIG_SHARE="$(ros2 pkg prefix --share "$CONFIG_PACKAGE" 2>/dev/null)"; then
    echo "error: no deploy package for '$ROBOT': $CONFIG_PACKAGE" >&2
    exit 1
fi
YAML="$CONFIG_SHARE/config/deploy.yaml"
[ -f "$YAML" ] || { echo "error: no config for '$ROBOT': $YAML" >&2; exit 1; }

echo "[ddt_rl_deploy] robot=$ROBOT policy=$POLICY"
exec python3 "$REPO/src/control/inference/rl_inference.py" --ros-args \
    -p config_file:="$YAML" -p policy_name:="$POLICY" "$@"
