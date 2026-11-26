#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PKG="$(cd -- "$HERE/../.." && pwd -P)"
export V2_MASTER_URI="${V2_MASTER_URI:-http://127.0.0.1:11411}"
source "$HERE/env_noetic.sh" >/dev/null
cd "$PKG"
exec /usr/bin/python3 -B "$@"
