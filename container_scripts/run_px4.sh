#!/bin/bash

cd /home/PX4-Autopilot

export PX4_HOME_LAT=-8.368936009367731;
export PX4_HOME_LON=-35.014375441995696;
export PX4_HOME_ALT=0.0;
make px4_sitl gz_x500 HEADLESS=1