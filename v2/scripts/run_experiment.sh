#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PKG_DIR="$(cd -- "$SCRIPT_DIR/../.." && pwd -P)"
# env -i prevents ~/.bashrc/ROS2 overlay leakage; only task-specific variables survive.
exec env -i HOME="$HOME" USER="${USER:-robot}" LOGNAME="${LOGNAME:-robot}" PATH=/usr/local/bin:/usr/bin:/bin LC_ALL=C.UTF-8 V2_MASTER_URI="${V2_MASTER_URI:-http://127.0.0.1:11321}" \
  bash --noprofile --norc -c 'source "$1"; shift; cd "$1"; shift; exec python3 -B -m v2.scripts.experiment "$@"' _ "$SCRIPT_DIR/env_noetic.sh" "$PKG_DIR" "$@"
