# Container setup

Two images are defined here:

- **`harpia2_drone_planner`** (production) — built from `Dockerfile`. Copies `action_planner/` to
  `/home/action_planner` and `src/` to `/home/drone_planner`, then runs `/home/drone_planner/build.sh`
  at image-build time (one colcon build of the framework and drone packages into
  `/home/drone_planner/install`). Self-contained; no bind mounts needed to run.
- **`harpia2_drone_planner_dev`** (dev) — built from `dev_Dockerfile`. Installs the same
  system/PX4/Python dependencies but does **not** copy or build the project. `action_planner/`, `src/` and
  `scripts/` are bind-mounted from the host at `docker run` time instead, so you can edit on the host
  and rebuild inside the container without rebuilding the image.

Both images build PX4 SITL (`gz_x500` target) from source, which takes a while the first time.
`.dockerignore` at the repo root keeps host `build/`, `install/`, `log/` and virtualenvs out of the
build context.

All commands below assume your working directory is `container/` (the scripts use `cd ..` / relative
paths internally).

## Production image (`harpia2_drone_planner`)

1. Build the image:
   ```bash
   cd container
   bash docker_build.sh
   ```
   Equivalent to running, from the repo root: `docker build -f container/Dockerfile -t harpia2_drone_planner .`

2. Allow the container to reach your X server (only needed once per host session, or whenever the
   RViz GUI window fails to appear):
   ```bash
   xhost +local:root
   ```

3. Run the container:
   ```bash
   cd container
   bash docker_run.sh
   ```
   This starts PX4 and MAVROS in the background (`entrypoint.sh` runs `/home/scripts/run_all.sh`,
   logging to `/tmp/drone_planner_exxon/logs/` inside the container) and drops you into an
   interactive `bash` shell in `/home/drone_planner`. From there, run the planner with:
   ```bash
   ./run.sh [mission_index]
   ```
   and, optionally, the operator CLI from a second terminal
   (`bash open_container_terminal.sh` on the host attaches one):
   ```bash
   /home/scripts/run_drone_planner_cli.sh
   ```

## Dev image (`harpia2_drone_planner_dev`)

1. Build the image:
   ```bash
   cd container
   bash docker_build_dev.sh
   ```
   Equivalent to running, from the repo root: `docker build -f container/dev_Dockerfile -t harpia2_drone_planner_dev .`

2. Allow the container to reach your X server (same caveat as above):
   ```bash
   xhost +local:root
   ```

3. Run the container:
   ```bash
   cd container
   bash docker_run_dev.sh
   ```
   This drops you into an interactive `bash` shell and bind-mounts, read-write, from the repo root:
   - `scripts/` → `/home/scripts`
   - `action_planner/` → `/home/action_planner`
   - `src/` → `/home/drone_planner`

   It also mounts the named Docker volume `harpia_drone_planner_vscode_server` at `/root/.vscode-server`,
   so VS Code's container server install is reused when the dev container is recreated.

   Because the project itself isn't built into the image, build it inside the container before
   running anything. PX4 and MAVROS are not started automatically in the dev image; use the scripts:
   ```bash
   cd /home/drone_planner
   ./build.sh                      # framework + drone packages (finds the framework at /home/action_planner)
   /home/scripts/run_all.sh        # PX4 SITL + MAVROS in the background (stop_all.sh to stop)
   ./run.sh [mission_index]
   ```
   To work on the framework alone: `cd /home/action_planner && ./build.sh && ./run_example.sh`.

   Since the source directories are bind-mounted, edits made on the host are picked up immediately —
   just re-run `./build.sh` inside the container after changing code (the install is not symlinked).

## Notes

- Both `docker_run*.sh` scripts run the container with `--privileged --network=host` and mount
  `/tmp/.X11-unix` and `~/.Xauthority` for GUI (RViz) passthrough, plus `/dev/dri` for GPU access.
- If RViz's window still doesn't appear after `xhost +local:root`, re-run it as root as well.
- Both scripts invoke `docker` via `sudo -E` — expect a sudo password prompt if your shell isn't
  already elevated.
- `container/requirements.txt` pins the Python dependencies installed into both images (it is a
  superset of `action_planner/src/action_planner/requirements.txt`); update it if the planner's or the
  drone system's Python dependencies change.
