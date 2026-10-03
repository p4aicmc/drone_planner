#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# if rviz GUI doesnt show up run: "xhost +local:root" as root and re-run, also as root
sudo -E docker run --rm -it \
  --network=host \
  --ipc=host \
  --shm-size=512m \
  --privileged \
  --env DISPLAY=$DISPLAY \
  --env XAUTHORITY=/root/.Xauthority \
  --env QT_X11_NO_MITSHM=1 \
  --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:rw" \
  --volume harpia_drone_planner_vscode_server:/root/.vscode-server:rw \
  --volume "$REPO_ROOT/scripts:/home/scripts:rw" \
  --volume "$REPO_ROOT/src:/home/drone_planner:rw" \
  --device /dev/dri:/dev/dri \
  harpia2_drone_planner_dev
