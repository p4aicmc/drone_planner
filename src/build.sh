if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
  echo "Using ROS Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  echo "Using ROS Humble"
else
  echo "Error: Neither ROS Jazzy nor ROS Humble found"
  exit 1
fi

if [[ ! -f "src/harpia_msgs/install/setup.bash" ]]; then
  unset COMPILE_SCRIPT_JUMP_HARPIA_MSGS_COMPILATION_IF_ALREADY_IS
fi

if [[ -z "$COMPILE_SCRIPT_JUMP_HARPIA_MSGS_COMPILATION_IF_ALREADY_IS" ]]; then
  unset COMPILE_SCRIPT_JUMP_HARPIA_MSGS_COMPILATION_IF_ALREADY_IS
  echo "Building harpia_msgs:"
  cd src/harpia_msgs
  if ! colcon build; then
    echo "Error building harpia_msgs"
    exit 1
  fi
  cd ../..
  echo "Builded harpia_msgs successfully"
else
  echo -e "Detected an existing harpia_msgs installation. Skipping compilation.\n"
fi

source src/harpia_msgs/install/setup.bash

echo "Building main project:"
# if ! colcon build --symlink-install; then
if ! colcon build; then
  echo "Error building main project"
  exit 1
fi
source install/setup.bash
echo "Builded main project successfully"

chmod +x install/drone_planner/share/drone_planner/solver/OPTIC/generate_plan.sh install/drone_planner/share/drone_planner/solver/OPTIC/optic-clp
chmod +x install/drone_planner/share/drone_planner/solver/TFD/generate_plan.sh install/drone_planner/share/drone_planner/solver/TFD/downward/preprocess/preprocess install/drone_planner/share/drone_planner/solver/TFD/downward/search/search