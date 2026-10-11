#!/usr/bin/env bash
# Builds the drone workspace (this directory) together with the action_planner framework
# packages in a single colcon invocation, so one `source install/setup.bash` is
# enough. The framework location is resolved from, in order:
#   $ACTION_PLANNER_DIR, <repo>/action_planner (next to this workspace), /home/action_planner (container).
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
  echo "Using ROS Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  echo "Using ROS Humble"
else
  echo "Error: Neither ROS Jazzy nor ROS Humble found" >&2
  exit 1
fi

for candidate in "${ACTION_PLANNER_DIR:-}" "$(pwd)/../action_planner" "/home/action_planner"; do
  if [[ -n "$candidate" && -d "$candidate/src/action_planner" ]]; then
    FRAMEWORK_DIR="$(cd "$candidate" && pwd)"
    break
  fi
done
if [[ -z "${FRAMEWORK_DIR:-}" ]]; then
  echo "Error: action_planner framework not found. Set ACTION_PLANNER_DIR to the framework workspace." >&2
  exit 1
fi
echo "Using framework at $FRAMEWORK_DIR"

# Leftovers from before the framework split (nested harpia_msgs build) or from the
# intermediate package names; colcon never removes stale installs by itself.
if [[ -d src/harpia_msgs || -d install/harpia_msgs || -d install/harpia_framework ]]; then
  echo "Warning: stale pre-split files detected (src/harpia_msgs or an old install/). Run 'rm -rf build install log src/harpia_msgs' and rebuild." >&2
fi

echo "Building drone workspace + framework:"
if ! colcon build --base-paths . "$FRAMEWORK_DIR/src" "$@"; then
  echo "Error building the workspace" >&2
  exit 1
fi
source install/setup.bash
echo "Built successfully"
