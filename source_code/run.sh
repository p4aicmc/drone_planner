rm -rf output
mkdir output

if [[ -z "$RUN_SCRIPT_DO_NOT_CLEAR" ]]; then
    unset RUN_SCRIPT_DO_NOT_CLEAR
    clear
fi

source src/harpia_msgs/install/setup.bash
source install/setup.bash

echo "Running project:"

if [ -f "./process_output.py" ]; then
    # ros2 launch route_executor2 harpia_launch.py ${1:+mission_index:=$1}
    # ros2 launch route_executor2 harpia_launch.py ${1:+mission_index:=$1} | ./process_output.py
    stdbuf -o0 ros2 launch route_executor2 harpia_launch.py ${1:+mission_index:=$1} 2>&1 | stdbuf -o0 ./process_output.py
else
    ros2 launch route_executor2 harpia_launch.py ${1:+mission_index:=$1}
fi