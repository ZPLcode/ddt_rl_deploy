#!/usr/bin/env bash
# Keyboard teleop (same for sim2sim and sim2real)
#   w/s fwd-back · a/d turn · q/e strafe · r/f raise-lower · space stop · x quit
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/env.sh" >/dev/null
exec python3 "$REPO/deploy/teleop.py" "$@"
