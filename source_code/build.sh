
source /opt/ros/humble/setup.bash

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

chmod +x install/route_executor2/share/route_executor2/solver/OPTIC/generate_plan.sh install/route_executor2/share/route_executor2/solver/OPTIC/optic-clp
chmod +x install/route_executor2/share/route_executor2/solver/TFD/generate_plan.sh install/route_executor2/share/route_executor2/solver/TFD/downward/preprocess/preprocess install/route_executor2/share/route_executor2/solver/TFD/downward/search/search