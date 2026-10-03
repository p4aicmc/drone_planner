#!/bin/bash
set -euo pipefail

/home/scripts/run_all.sh

cd /home/drone_planner

# Drop into provided command (or bash)
exec "${@:-bash}"
exit "$fg_status"
