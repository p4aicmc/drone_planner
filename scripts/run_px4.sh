#!/bin/bash

cd /home/PX4-Autopilot
source /opt/ros/jazzy/setup.bash


export PX4_HOME_LAT=-22.001333
export PX4_HOME_LON=-47.934152
export PX4_HOME_ALT=0.0

HEADLESS=1 make px4_sitl gz_x500
