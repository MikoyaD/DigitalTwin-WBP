#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PKG="$(cd -- "$HERE/../.." && pwd -P)"
cd "$PKG"
exec /usr/bin/python3 -B -m v2.scripts.v2_launcher "$@"
