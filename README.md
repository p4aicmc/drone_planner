# Harpia - A Hybrid System for UAV Missions

## About the project

Harpia 2 is the ROS 2 version of the Harpia system for UAV mission and path planning. The project aims to provides a set of tools and libraries for generate UAV autonomous missions using PDDL Planning in a high-level for the user. 


  
## Repository layout

Harpia 2 is split in two parts:

- **`action_planner/`** – the action_planner framework, a standalone colcon workspace. PDDL knowledge
  base, OPTIC/TFD planners, plan execution over ROS 2 actions, mission control, lifecycle
  orchestration and the operator CLI. Robot-agnostic; see [`action_planner/README.md`](action_planner/README.md)
  for how it works and how to use it from another robot.
- **`src/`** – the drone system, a colcon workspace that imports the framework. Flight control over
  MAVROS/PX4, RRT* path planning, mission/map data, the PDDL domain and the action executor nodes
  (`go_to`, `take_off`, `land`, `take_image`, `recharge_battery`).
- `scripts/` – PX4 SITL + MAVROS service runners and the CLI wrapper. `container/` – Docker images.
  `map_generator/` – standalone KML to `map.json` tool.

## Container build

Run from `container/` (see [`container/README.md`](container/README.md) for the dev image):
```bash
cd container
bash docker_build.sh      # image harpia2_drone_planner (ROS Jazzy, Gazebo Harmonic, PX4 SITL)
xhost +local:root
bash docker_run.sh
```

### Running the full system in container

- Open QGroundControl and make sure the connection with the drone is working
- The container starts PX4 and MAVROS in the background and drops you in `/home/drone_planner`:
```bash
cd /home/drone_planner
./run.sh [mission_index]
```
- Operator CLI (optional, open it before `run.sh` to start the mission by hand):
```bash
/home/scripts/run_drone_planner_cli.sh
```

## Local installation

<aside>
💡 Make sure that the system is updated →`sudo apt-get update`
</aside>

### System Versions

- Ubuntu 24.04 LTS with ROS 2 Jazzy (what the container uses). Ubuntu 22.04 / ROS 2 Humble still
  works with the scripts, which fall back to Humble when Jazzy is absent.
- PX4 Autopilot (SITL, `gz_x500` target), QGroundControl, MAVROS

### ROS 2
- See the official docs: [ROS 2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation.html)
  (replace `jazzy` by `humble` below for 22.04).

```bash
sudo apt install software-properties-common

sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
sudo apt install ros-dev-tools
```

### PX4
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash PX4_Autopilot/Tools/setup/ubuntu.sh
cd PX4_Autopilot
make px4_sitl gz_x500
```
(`scripts/run_px4.sh` expects the checkout at `/home/PX4-Autopilot`; edit it for a local path.)

### QGroundControl
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
- Logout and login again
```bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage  # (or double click)
```

### Mavros
```bash
sudo apt install ros-jazzy-mavros
ros2 run mavros install_geographiclib_datasets.sh
```

### Python dependencies and the OPTIC planner
```bash
python3 -m pip install --break-system-packages --ignore-installed -r container/requirements.txt   # --ignore-installed: apt owns numpy/scipy
sudo apt install coinor-libcbc3.1
sudo ln -sf /usr/lib/x86_64-linux-gnu/libCbc.so.3.1 /usr/lib/x86_64-linux-gnu/libCbc.so.3
```

### Build

```bash
cd src
./build.sh                  # builds framework + drone packages into src/install
./run.sh [mission_index]    # ros2 launch drone_planner harpia_launch.py
./f_compile_and_run.sh [N]  # both
```

`build.sh` runs `colcon build --base-paths . ../action_planner/src`, so a single
`source src/install/setup.bash` gives you everything. Manual equivalent:
```bash
cd src
source /opt/ros/jazzy/setup.bash
colcon build --base-paths . ../action_planner/src
source install/setup.bash
ros2 launch drone_planner harpia_launch.py mission_index:=1 solver:=OPTIC
```

The framework alone (no drone, no MAVROS) can be built and demonstrated with
`cd framework && ./build.sh && ./run_example.sh`.

> **Upgrading from the pre-split layout** (where `harpia_msgs` lived in `src/src` and was built
> nested): colcon does not remove stale installs, so run
> `rm -rf src/build src/install src/log src/src/harpia_msgs` once before building.

### Troubleshooting

- Most problems come from not sourcing ROS 2 and the workspace (`source /opt/ros/jazzy/setup.bash`
  and `source src/install/setup.bash`) in every new terminal. Adding the ROS line to `.bashrc` helps.
- Every Python change needs a rebuild: the install is not symlinked, nodes run from `install/`.
- `action_planner` fails to plan: check that `optic-clp` can load `libCbc.so.3` (see above) and look
  at `src/output/domain.pddl` / `problem.pddl`, which are the exact files sent to the planner.

## Simulation Video

[![Video](Video)](https://youtu.be/--hn0I5QUJ8)

## Articles 
- [Service-Oriented Architecture to Integrate Flight Safety and Mission Management Subsystems into UAVs, ICAS, BeloHorizonte, 2018](https://www.icas.org/ICAS_ARCHIVE/ICAS2018/data/papers/ICAS2018_0374_paper.pdf)
- [Harpia: A Hybrid System for Agricultural UAV Missions](https://authors.elsevier.com/tracking/article/details.do?surname=Vannini&aid=100191&jid=ATECH)
