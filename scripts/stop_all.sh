#!/usr/bin/env bash
set -euo pipefail

PID_DIR="${PID_DIR:-/tmp/drone_planner_exxon}"
SERVICES=(px4 mavros)

stop_service() {
  local name="$1"
  local pid_file="$PID_DIR/$name.pid"

  if [[ ! -s "$pid_file" ]]; then
    echo "$name is not running"
    return
  fi

  local pid
  pid="$(cat "$pid_file")"

  if ! kill -0 "$pid" 2>/dev/null; then
    echo "$name is not running (removing stale pid $pid)"
    rm -f "$pid_file"
    return
  fi

  echo "Stopping $name (pid $pid)..."
  kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true

  for _ in {1..50}; do
    if ! kill -0 "$pid" 2>/dev/null; then
      rm -f "$pid_file"
      echo "Stopped $name"
      return
    fi
    sleep 0.1
  done

  echo "$name did not stop after SIGTERM; sending SIGKILL..."
  kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  rm -f "$pid_file"
  echo "Stopped $name"
}

for service in "${SERVICES[@]}"; do
  stop_service "$service"
done
