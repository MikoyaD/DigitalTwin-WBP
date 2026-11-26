#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PKG="$(cd -- "$HERE/../.." && pwd -P)"
verbose=0
if [[ "${1:-}" == --verbose ]]; then verbose=1; shift; fi
log_dir="${V2_LOG_DIR:-$HOME/.ros/v2_interaction_logs}"
mkdir -p "$log_dir"
export V2_BACKGROUND_LOG="$(mktemp "$log_dir/${0##*/}.XXXXXXXX.log")"
printf '后台日志：%s\n' "$V2_BACKGROUND_LOG"
export V2_MASTER_URI="${V2_MASTER_URI:-http://127.0.0.1:11411}"
source "$HERE/env_noetic.sh" >/dev/null
cd "$PKG"
if [[ "${V2_NO_RVIZ:-0}" != 1 ]]; then
  rviz -d "$PKG/v2/rviz/v2_overlay.rviz" >>"$V2_BACKGROUND_LOG" 2>&1 &
  rviz_pid=$!
  if [[ "$verbose" == 1 ]]; then tail -n +1 -f "$V2_BACKGROUND_LOG" & tail_pid=$!; fi
  trap 'kill "$rviz_pid" ${tail_pid:-} 2>/dev/null || true' EXIT
  sleep 4
  if grep -Eqi "PluginlibFactory.*failed to load|plugin.*failed to load" "$V2_BACKGROUND_LOG"; then printf "错误：RViz 插件加载失败，请检查日志 %s\n" "$V2_BACKGROUND_LOG" >&2; exit 1; fi
  if ! kill -0 "$rviz_pid" 2>/dev/null; then printf "错误：RViz 启动失败，请检查日志 %s\n" "$V2_BACKGROUND_LOG" >&2; exit 1; fi
fi
if [[ "$#" -eq 0 ]]; then
  /usr/bin/python3 -B -m v2.scripts.dynamic_overlay
else
  /usr/bin/python3 -B -m v2.scripts.trajectory_overlay "$@"
fi
