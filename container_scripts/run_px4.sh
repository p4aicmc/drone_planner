#!/bin/bash

cd /home/PX4-Autopilot

export PX4_HOME_LAT=-22.001333
export PX4_HOME_LON=-47.934152
export PX4_HOME_ALT=847.142652

make px4_sitl gazebo-classic HEADLESS=1