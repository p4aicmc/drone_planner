#!/bin/bash
set -euo pipefail

cd /home/mavros_navigator

# --- Start PX4 in background, log to file ---
/home/scripts/run_px4.sh >> /home/mavros_navigator/px4.log 2>&1 & p1=$!

# --- Start MAVROS in background, log to file ---
/home/scripts/run_mavros.sh >> /home/mavros_navigator/mavros.log 2>&1 & p2=$!

# Forward signals to the background processes
cleanup() {
  echo "Cleaning up..."
  kill -TERM "$p1" "$p2" 2>/dev/null || true
  wait "$p1" "$p2" 2>/dev/null || true
}
trap cleanup INT TERM

# --- Run your app in the foreground (prints to terminal) ---
# ./run.sh
exec "${@:-bash}"
fg_status=$?

# After your app exits, wait for PX4 and MAVROS to finish (or Ctrl+C)
wait "$p1" "$p2" || true

# Drop into provided command (or bash)
exec "${@:-bash}"
exit "$fg_status"
