#!/usr/bin/env bash
# Source from a clean, non-interactive bash. No ~/.bashrc and no ROS2 overlay.
set -euo pipefail
_v2_source_setup() {
  local setup_file="$1"
  shift
  source "$setup_file" "$@"
}
V2_ENV_SETUP_RESULTS=""
if [[ "${V2_PRP_CONFIG_MODE:-0}" == 1 ]]; then
  _v2_pkg="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
  _v2_setups=(/opt/ros/noetic/setup.bash)
  if [[ -n "${V2_EXTRA_SETUP:-}" ]]; then _v2_setups+=("$V2_EXTRA_SETUP"); fi
  _v2_setups+=("$(cd -- "$_v2_pkg/../.." && pwd -P)/devel/setup.bash")
else
  # The laboratory uses system MoveIt and its existing catkin workspace.
  _v2_setups=(/opt/ros/noetic/setup.bash /home/robot/digitaltwin/zzs_ws/devel/setup.bash)
fi
for setup in "${_v2_setups[@]}"; do
  if [[ ! -r "$setup" ]]; then printf 'MISSING_SETUP %s\n' "$setup" >&2; return 30 2>/dev/null || exit 30; fi
  # Catkin setup scripts inspect unset variables; temporarily relax nounset.
  set +u
  if [[ "${V2_PRP_CONFIG_MODE:-0}" == 1 && "$setup" != /opt/ros/noetic/setup.bash ]]; then
    _v2_source_setup "$setup" --extend
  else
    _v2_source_setup "$setup"
  fi
  set -u
  printf 'SOURCED %s\n' "$setup"
  V2_ENV_SETUP_RESULTS="${V2_ENV_SETUP_RESULTS}${setup}:OK;"
done
if [[ "${ROS_DISTRO:-}" != noetic ]]; then printf 'WRONG_ROS_DISTRO %s\n' "${ROS_DISTRO:-unset}" >&2; return 31 2>/dev/null || exit 31; fi
export ROS_MASTER_URI="${V2_MASTER_URI:-http://127.0.0.1:11321}"
export PYTHONDONTWRITEBYTECODE=1
export V2_ENV_SETUP_RESULTS
printf 'ROS_DISTRO=%s ROS_MASTER_URI=%s ROS_PACKAGE_PATH=%s\n' "$ROS_DISTRO" "$ROS_MASTER_URI" "${ROS_PACKAGE_PATH:-}"
