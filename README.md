# This readme file is currently under construction

Please be aware that this readme file is a work in progress and may not be complete. We will be updating it regularly with new information, so please check back later for updates.

# Harpia - A Hybrid System for UAV Missions

## About the project

Harpia 2 is the ROS 2 version of the Harpia system for UAV mission and path planning. The project aims to provides a set of tools and libraries for generate UAV autonomous missions in a high-level for the user. 

## The Architecture

## Instalation
<aside>
💡 Make sure that the system is updated →`sudo apt-get update`
</aside>

### System Versions

- Ubuntu: Ubuntu 22.04 LTS
- ROS 2 → Humble
- QGroundControl →
- Mavros
  
### Instalation

#### ROS 2 Humble
- https://docs.ros.org/en/humble/Installation.html

- sudo apt install software-properties-common
- sudo add-apt-repository universe

- sudo apt update && sudo apt install curl -y
- sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
- echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

- sudo apt update
- sudo apt upgrade

- sudo apt install ros-humble-desktop
- sudo apt install ros-dev-tools

#### PX4
- git clone https://github.com/PX4/PX4-Autopilot.git --recursive
- bash PX4_Autopilot/Tools/setup/ubuntu.sh
##### Use gazebo classic instead of Ignition Gazebo
- sudo apt remove gz-harmonic
- sudo apt install aptitude
- sudo aptitude install gazebo libgazebo11 libgazebo-dev
- cd PX4_Autopilot
- make px4_sitl gazebo-classic

#### QGroundControl
- sudo usermod -a -G dialout $USER
- sudo apt-get remove modemmanager -y
- sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
- sudo apt install libfuse2 -y
- sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
- Logout and login again
- chmod +x ./QGroundControl.AppImage
- ./QGroundControl.AppImage  (or double click)

#### Mavros
- sudo apt install ros-humble-mavros
- ros2 run mavros install_geographiclib_datasets.sh

### Build

- (Recommended) Use the bash scripts "compile.sh", "compile_and_run.sh" and "run.sh"
- Manual compiling:
-- cd src/harpia_msgs (in the project root directory)
-- source /opt/ros/humble/setup.bash
-- colcon build
-- source install/setup.bash
-- cd ../.. (return to the project root directory)
-- colcon build
-- source install/setup.bash
-- Launch the system with ros2 launch launch/launch_file.py

### Troubleshooting

- Common issues when installing or running the system comes from:
- Not sourcing ROS 2, harpia_msgs or the harpia system itself (remember to use source /opt/ros/humble/setup.bash and any other setup.bash files needed in every new terminal session)
- To avoid problems sourcing ROS 2 its recommended to add the source /opt/ros/humble/setup.bash to your .bashrc file

## Simulation Video

[![Video](https://i9.ytimg.com/vi_webp/--hn0I5QUJ8/mq2.webp?sqp=CJzBop4G-oaymwEmCMACELQB8quKqQMa8AEB-AHUBoAC4AOKAgwIABABGGQgZShUMA8=&rs=AOn4CLALXTaHg7IRncNrzhT9RfPaIgf7Pg)](https://youtu.be/--hn0I5QUJ8)

## Articles 
- [Service-Oriented Architecture to Integrate Flight Safety and Mission Management Subsystems into UAVs, ICAS, BeloHorizonte, 2018](https://www.icas.org/ICAS_ARCHIVE/ICAS2018/data/papers/ICAS2018_0374_paper.pdf)
- [Harpia: A Hybrid System for Agricultural UAV Missions](https://authors.elsevier.com/tracking/article/details.do?surname=Vannini&aid=100191&jid=ATECH)
