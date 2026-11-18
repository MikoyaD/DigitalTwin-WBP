#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PKG="$(cd -- "$HERE/../.." && pwd -P)"
exec env -i HOME="$HOME" USER="${USER:-robot}" LOGNAME="${LOGNAME:-robot}" PATH=/usr/local/bin:/usr/bin:/bin LC_ALL=C.UTF-8 OPENBLAS_NUM_THREADS=1 V2_MASTER_URI=http://127.0.0.1:11411 \
  bash --noprofile --norc -c 'source "$1" >/dev/null; shift; cd "$1"; shift; exec python3 -B -m v2.scripts.phg1_run "$@"' _ "$HERE/env_noetic.sh" "$PKG" "$@"
