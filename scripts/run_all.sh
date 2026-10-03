#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PID_DIR="${PID_DIR:-/tmp/drone_planner_exxon}"
LOG_DIR="${LOG_DIR:-/tmp/drone_planner_exxon/logs}"

mkdir -p "$PID_DIR" "$LOG_DIR"

SERVICES=(
  "px4:$SCRIPT_DIR/run_px4.sh:$LOG_DIR/px4.log"
  "mavros:$SCRIPT_DIR/run_mavros.sh:$LOG_DIR/mavros.log"
)

is_running() {
  local pid_file="$1"
  [[ -s "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null
}

any_running=false
for service in "${SERVICES[@]}"; do
  name="${service%%:*}"
  if is_running "$PID_DIR/$name.pid"; then
    any_running=true
    break
  fi
done

if [[ "$any_running" == true ]]; then
  echo "Existing services found; stopping them first..."
  bash "$SCRIPT_DIR/stop_all.sh"
fi

start_service() {
  local name="$1"
  local script="$2"
  local log_file="$3"

  if [[ ! -x "$script" ]]; then
    echo "Missing or non-executable script: $script" >&2
    exit 1
  fi

  : > "$log_file"
  setsid "$script" >> "$log_file" 2>&1 &
  local pid=$!
  echo "$pid" > "$PID_DIR/$name.pid"
  echo "Started $name (pid $pid, log $log_file)"
}

for service in "${SERVICES[@]}"; do
  IFS=: read -r name script log_file <<< "$service"
  start_service "$name" "$script" "$log_file"
done

echo "All services started in the background."
