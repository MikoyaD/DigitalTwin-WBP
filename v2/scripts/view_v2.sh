#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PKG="$(cd -- "$HERE/../.." && pwd -P)"
URI=http://127.0.0.1:11411
CONFIG_MODE=0
for arg in "$@"; do if [[ "$arg" == "--gazebo-csv" ]]; then URI="${V2_MASTER_URI:-http://127.0.0.1:11511}"; CONFIG_MODE=1; fi; done
if [[ "$CONFIG_MODE" == 0 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-robot}" LOGNAME="${LOGNAME:-robot}" PATH=/usr/local/bin:/usr/bin:/bin LC_ALL=C.UTF-8 DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" V2_MASTER_URI=http://127.0.0.1:11411 \
    bash --noprofile --norc -c 'source "$1" >/dev/null; shift; cd "$1"; shift; exec python3 -B -m v2.scripts.view_entry "$@"' _ "$HERE/env_noetic.sh" "$PKG" "$@"
fi
exec env -i HOME="$HOME" USER="${USER:-robot}" LOGNAME="${LOGNAME:-robot}" PATH=/usr/local/bin:/usr/bin:/bin LC_ALL=C.UTF-8 DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" ROS_HOME="${ROS_HOME:-/tmp/v2_csv_replay_ros}" V2_PRP_CONFIG_MODE="$CONFIG_MODE" V2_MASTER_URI="$URI" \
  bash --noprofile --norc -c 'source "$1" >/dev/null; shift; cd "$1"; shift; exec python3 -B -m v2.scripts.view_entry "$@"' _ "$HERE/env_noetic.sh" "$PKG" "$@"
