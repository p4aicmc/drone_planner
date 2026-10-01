#!/bin/bash
set -euo pipefail

DOCKER_CMD=(docker)
if ! docker ps >/dev/null 2>&1; then
  DOCKER_CMD=(sudo docker)
fi

find_container() {
  local image="$1"
  "${DOCKER_CMD[@]}" ps \
    --filter "ancestor=${image}" \
    --format '{{.ID}} {{.Image}} {{.Names}}' \
    | head -n 1 \
    | awk '{print $1}'
}

CONTAINER_ID="$(find_container harpia2_drone_planner_dev)"
CONTAINER_KIND="dev"

if [[ -z "$CONTAINER_ID" ]]; then
  CONTAINER_ID="$(find_container harpia2_drone_planner)"
  CONTAINER_KIND="regular"
fi

if [[ -z "$CONTAINER_ID" ]]; then
  echo "ERROR: No running harpia2_drone_planner_dev or harpia2_drone_planner container found." >&2
  echo "Start one first with container/docker_run_dev.sh or container/docker_run.sh." >&2
  exit 1
fi

echo "Opening ${CONTAINER_KIND} container: ${CONTAINER_ID}"
"${DOCKER_CMD[@]}" exec -it "$CONTAINER_ID" bash
