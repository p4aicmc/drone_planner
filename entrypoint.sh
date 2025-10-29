#!/bin/bash

cd /home/mavros_navigator
# ./run.sh

set -euo pipefail

cmd1 &> cmd1.log &   p1=$!
cmd2 &> cmd2.log &   p2=$!
cmd3

exec "${@:-bash}"
