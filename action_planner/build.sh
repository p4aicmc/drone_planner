#!/usr/bin/env bash
# Builds the action_planner framework as a standalone colcon workspace (this directory).
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo "Error: Neither ROS Jazzy nor ROS Humble found" >&2
  exit 1
fi

colcon build "$@"
echo "Framework built. Source install/setup.bash to use it."
