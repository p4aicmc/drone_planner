#!/usr/bin/env bash
# Build, then run. Any argument is passed to run.sh as the mission index.
clear
cd "$(dirname "${BASH_SOURCE[0]}")"

./build.sh

export RUN_SCRIPT_DO_NOT_CLEAR=1
./run.sh ${1:+$1}
