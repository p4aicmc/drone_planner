#!/usr/bin/env bash

rm -rf output
mkdir -p output

if [[ -z "$RUN_SCRIPT_DO_NOT_CLEAR" ]]; then
    unset RUN_SCRIPT_DO_NOT_CLEAR
    clear
fi

# Source ROS 2 first (prefer jazzy, fallback to humble)
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: No ROS 2 installation found at /opt/ros/jazzy or /opt/ros/humble." >&2
    exit 1
fi

# Source workspace overlay
source install/setup.bash

echo "Running project:"

if [ -f "./process_output.py" ]; then
    stdbuf -o0 ros2 launch drone_planner harpia_launch.py ${1:+mission_index:=$1} 2>&1 | stdbuf -o0 python3 ./process_output.py
else
    ros2 launch drone_planner harpia_launch.py ${1:+mission_index:=$1}
fi
