#!/bin/bash

cd /home/mavros_navigator
./run.sh

exec "${@:-bash}"
