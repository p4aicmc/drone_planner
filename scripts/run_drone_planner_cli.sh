#!/usr/bin/env bash
# Opens the Harpia operator CLI against a running drone system.
# ROS-generated setup scripts access optional variables that may be unset, so
# this launcher intentionally does not enable `set -u` (nounset).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${DRONE_PLANNER_WORKSPACE:-}" ]]; then
  WORKSPACE_DIR="$DRONE_PLANNER_WORKSPACE"
elif [[ -f "$SCRIPT_DIR/../src/install/setup.bash" ]]; then
  WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../src" && pwd)"   # host checkout
else
  WORKSPACE_DIR="/home/drone_planner"                   # container
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi

if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  echo "ERROR: workspace is not built: $WORKSPACE_DIR/install/setup.bash is missing." >&2
  echo "Build it with: cd $WORKSPACE_DIR && ./build.sh" >&2
  exit 1
fi

source "$WORKSPACE_DIR/install/setup.bash"
# Tab completion for the drone actions
export ACTION_PLANNER_CLI_ACTIONS="${ACTION_PLANNER_CLI_ACTIONS:-go_to,take_image,recharge_battery,take_off,land}"
exec ros2 run action_planner action_planner_cli.py "$@"
