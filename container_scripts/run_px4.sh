#!/bin/bash

cd /home/PX4-Autopilot

export PX4_HOME_LAT=-8.368936009367731;
export PX4_HOME_LON=-35.014375441995696;
make px4_sitl gazebo-classic HEADLESS=1