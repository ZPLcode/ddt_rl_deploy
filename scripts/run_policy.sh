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

YAML="$REPO/config/$ROBOT/controllers.yaml"
[ -f "$YAML" ] || { echo "error: no config for '$ROBOT': $YAML" >&2; exit 1; }

echo "[ddt_rl_deploy] robot=$ROBOT policy=$POLICY"
exec python3 "$REPO/deploy/rl_inference.py" --ros-args \
    -p config_file:="$YAML" -p policy_name:="$POLICY" "$@"
