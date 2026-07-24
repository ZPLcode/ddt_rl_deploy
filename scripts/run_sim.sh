#!/usr/bin/env bash
# sim2sim backend launcher. A backend implements the topic contract in sim/BACKEND.md; deploy is unchanged.
#   ./run_sim.sh                          # default mujoco backend (GUI)
#   ./run_sim.sh --backend webots --gui   # pick a backend (webots GUI renders on the discrete GPU)
#   ./run_sim.sh --robot d1cargo_out      # pick a robot (passed through; backend resolves the model)
#   ./run_sim.sh --no-viewer              # any other args pass straight through to the backend
#   ./run_sim.sh --list-backends          # list installed backends
#
# Backend discovery (add a backend = drop a file, no edits here):
#   sim/<name>_sim.py   -> run with python3   (lightweight backend, e.g. mujoco)
#   sim/<name>/run.sh   -> execute it         (framework backend, e.g. webots)
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

list_backends() {
    for f in "$REPO"/sim/*_sim.py; do
        [ -e "$f" ] || continue; b="$(basename "$f")"; echo "  ${b%_sim.py}"
    done
    for d in "$REPO"/sim/*/run.sh; do
        [ -e "$d" ] || continue; echo "  $(basename "$(dirname "$d")")"
    done
}

BACKEND="mujoco"
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --backend)   BACKEND="$2"; shift 2 ;;
        --backend=*) BACKEND="${1#*=}"; shift ;;
        --list-backends) echo "installed backends:"; list_backends; exit 0 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done

[ -f "$REPO/install/setup.bash" ] || {
    echo "error: repo not built (no install/) — run ./scripts/setup.sh first" >&2; exit 1; }
source "$REPO/env.sh" >/dev/null

PY="$REPO/sim/${BACKEND}_sim.py"
SH="$REPO/sim/${BACKEND}/run.sh"
if [ -f "$PY" ]; then
    exec python3 "$PY" "${ARGS[@]}"
elif [ -x "$SH" ]; then
    exec "$SH" "${ARGS[@]}"
else
    echo "error: backend '$BACKEND' not found" >&2
    echo "installed backends:" >&2; list_backends >&2
    echo "adding a backend: see sim/BACKEND.md" >&2
    exit 1
fi
