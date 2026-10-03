#!/usr/bin/env bash
# ROS-generated setup scripts access optional variables that may be unset, so
# this launcher intentionally does not enable `set -u` (nounset).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${DRONE_PLANNER_WORKSPACE:-/home/drone_planner}"

source /opt/ros/jazzy/setup.bash

if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  echo "ERROR: Drone planner workspace is not built: $WORKSPACE_DIR/install/setup.bash is missing." >&2
  echo "Build it with: cd $WORKSPACE_DIR && colcon build" >&2
  exit 1
fi

# The top-level workspace contains harpia_msgs and drone_planner. Do not source
# src/harpia_msgs/install here: that legacy nested build can contain generated
# interface libraries from different revisions.
source "$WORKSPACE_DIR/install/setup.bash"
exec python3 "$SCRIPT_DIR/drone_planner_cli.py" "$@"
