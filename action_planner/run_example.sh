#!/usr/bin/env bash
# Runs the standalone example (no robot needed). Build first with ./build.sh.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
source install/setup.bash

mkdir -p output
exec ros2 launch action_planner example_launch.py "$@"
