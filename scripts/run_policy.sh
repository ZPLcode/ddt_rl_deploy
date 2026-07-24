#!/usr/bin/env bash
# RL policy deployment (top-level app, speaks topics — same for sim2sim and sim2real)
#   ./run_policy.sh                          # rl_flat_lab + d1 (defaults)
#   ./run_policy.sh rl_flat_lab              # pick a policy
#   ./run_policy.sh rl_flat_lab d1 -p k:=v   # pass through extra ROS params
#
# Self-contained: needs only system ROS 2 + the repo-built ddt_msgs. For sim2sim pair with
# run_sim.sh; for sim2real run this against the real robot (same code).
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -f "$REPO/install/setup.bash" ] || {
    echo "error: repo not built (no install/) — run ./scripts/setup.sh first" >&2; exit 1; }

POLICY="${1:-rl_flat_lab}"
ROBOT="${2:-d1}"
shift $(( $# > 2 ? 2 : $# )) || true

source "$REPO/env.sh" >/dev/null

# ./run_policy.sh --list [robot]  list available policies
if [ "$POLICY" = "--list" ] || [ "$POLICY" = "-l" ]; then
    ROBOT="${ROBOT:-d1}"
    YAML="$REPO/config/$ROBOT/controllers.yaml"
    [ -f "$YAML" ] || { echo "error: config not found: $YAML" >&2; exit 1; }
    python3 - "$YAML" <<'PY'
import sys, yaml
def find(d, k):
    if isinstance(d, dict):
        if k in d:
            return d[k]
        for v in d.values():
            r = find(v, k)
            if r is not None:
                return r
full = yaml.safe_load(open(sys.argv[1]))
print(f'available policies ({sys.argv[1]}):')
for n in (find(full, 'rl_policy_names') or []):
    print('  ', n)
PY
    exit 0
fi

YAML="$REPO/config/$ROBOT/controllers.yaml"
[ -f "$YAML" ] || {
    echo "error: config not found: $YAML" >&2
    echo "available robots: $(ls "$REPO/config" 2>/dev/null | tr '\n' ' ')" >&2
    exit 1; }

echo "[ddt_rl_deploy] robot=$ROBOT policy=$POLICY"
exec python3 "$REPO/deploy/rl_inference.py" --ros-args \
    -p config_file:="$YAML" -p policy_name:="$POLICY" "$@"
