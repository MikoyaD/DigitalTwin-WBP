#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PKG="$(cd -- "$HERE/../.." && pwd -P)"
MASTER=http://127.0.0.1:11411
CONFIG=""; MODE=1; PREV=""; EXPORT=0
for arg in "$@"; do
  if [[ "$PREV" == --config ]]; then CONFIG="$arg"; fi
  case "$arg" in
    --config=*) CONFIG="${arg#--config=}" ;;
    --stage|--stage=*|--until|--until=*|--from-r|--from-r=*|--from-p|--from-p=*) MODE=0 ;;
    --export-csv|--export-csv=*) EXPORT=1; MODE=0 ;;
  esac
  PREV="$arg"
done
if [[ "$EXPORT" == 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-robot}" LOGNAME="${LOGNAME:-robot}" PATH=/usr/local/bin:/usr/bin:/bin LC_ALL=C.UTF-8 V2_PRP_CONFIG_MODE=1 V2_EXTRA_SETUP="${V2_EXTRA_SETUP:-}" V2_MASTER_URI="${V2_MASTER_URI:-${ROS_MASTER_URI:-$MASTER}}" OPENBLAS_NUM_THREADS=1 \
    bash --noprofile --norc -c 'source "$1" >/dev/null; shift; cd "$1"; shift; exec /usr/bin/python3 -B -m v2.scripts.stage_v2 "$@"' _ "$HERE/env_noetic.sh" "$PKG" "$@"
fi
if [[ "$MODE" == 1 && -n "$CONFIG" ]]; then
  # Read just the bootstrap fields with a known Noetic-compatible Python.
  BOOT="$(/usr/bin/python3 - "$CONFIG" <<'PYBOOT'
import os,sys,yaml
from pathlib import Path
p=Path(sys.argv[1]).resolve();c=yaml.safe_load(p.read_text())
if not isinstance(c,dict) or c.get('schema')!='v2-prp-v1': raise SystemExit('Expected schema: v2-prp-v1')
r=c.get('runtime',{});overlay=r.get('overlay_setup') or os.environ.get('V2_EXTRA_SETUP','')
if overlay and not Path(overlay).is_absolute(): overlay=str((p.parent/overlay).resolve())
python=r.get('python') or '/usr/bin/python3'
if not Path(python).is_absolute(): raise SystemExit('runtime.python must be an absolute path')
print(overlay);print(python)
PYBOOT
  )"
  OVERLAY="${BOOT%%$'\n'*}"; PYTHON="${BOOT#*$'\n'}"
  CONFIG_ABS="$(/usr/bin/python3 -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).resolve(strict=True))' "$CONFIG")"
  ARGS=("$@"); for ((i=0;i<${#ARGS[@]};i++)); do
    if [[ "${ARGS[i]}" == --config ]]; then ARGS[i+1]="$CONFIG_ABS"; break; fi
    if [[ "${ARGS[i]}" == --config=* ]]; then ARGS[i]="--config=$CONFIG_ABS"; break; fi
  done
  CALLER_MASTER="${V2_MASTER_URI:-${ROS_MASTER_URI:-}}"
  exec env -i HOME="$HOME" USER="${USER:-robot}" LOGNAME="${LOGNAME:-robot}" PATH=/usr/local/bin:/usr/bin:/bin LC_ALL=C.UTF-8 V2_MASTER_URI="${CALLER_MASTER:-$MASTER}" V2_CALLER_MASTER_URI="$CALLER_MASTER" V2_PRP_CONFIG_MODE=1 V2_EXTRA_SETUP="$OVERLAY" V2_PYTHON="$PYTHON" OPENBLAS_NUM_THREADS=1 \
    bash --noprofile --norc -c 'source "$1" >/dev/null; shift; cd "$1"; shift; exec "$V2_PYTHON" -B -m v2.scripts.stage_v2 "$@"' _ "$HERE/env_noetic.sh" "$PKG" "${ARGS[@]}"
fi
for arg in "$@"; do
  if [[ "$arg" == --from-r || "$arg" == --from-r=* || "$arg" == --from-p || "$arg" == --from-p=* ]]; then
    MASTER="${V2_MASTER_URI:-${ROS_MASTER_URI:-$MASTER}}"
    break
  fi
done
exec env -i HOME="$HOME" USER="${USER:-robot}" LOGNAME="${LOGNAME:-robot}" PATH=/usr/local/bin:/usr/bin:/bin LC_ALL=C.UTF-8 V2_MASTER_URI="$MASTER" \
  bash --noprofile --norc -c 'source "$1" >/dev/null; shift; cd "$1"; shift; exec python3 -B -m v2.scripts.stage_v2 "$@"' _ "$HERE/env_noetic.sh" "$PKG" "$@"
