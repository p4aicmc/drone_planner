# Container setup

Two images are defined here:

- **`harpia2`** (production) — built from `Dockerfile`. Copies `src/drone_planner` and
  `src/lightweight_plume` into the image and `colcon build`s them at image-build time. Self-contained;
  no bind mounts needed to run.
- **`harpia2_dev`** (dev) — built from `dev_Dockerfile`. Installs the same system/PX4/Python
  dependencies but does **not** copy or build the project. `scripts/`, `src/drone_planner`, and
  `src/lightweight_plume` are bind-mounted from the host at `docker run` time instead, so you can edit
  on the host and rebuild inside the container without rebuilding the image.

Both images build PX4 SITL (`gz_x500` target) from source, which takes a while the first time.

All commands below assume your working directory is `container/` (the scripts use `cd ..` / relative
paths internally).

## Production image (`harpia2`)

1. Build the image:
   ```bash
   cd container
   bash docker_build.sh
   ```
   Equivalent to running, from the repo root: `docker build -f container/Dockerfile -t harpia2 .`

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
   This starts PX4, MAVROS, and RViz in the background (`entrypoint.sh`, logging to `/home/{px4,mavros,rviz}.log`
   inside the container) and drops you into an interactive `bash` shell. From there, run the planner
   with:
   ```bash
   cd /home/drone_planner
   ./run.sh [mission_index]
   ```
   `docker_run.sh` bind-mounts `../sensoring_data` (relative to `container/`, i.e. `<repo>/sensoring_data`)
   into `/home/sensoring_data` — everything else runs from what was baked into the image.

## Dev image (`harpia2_dev`)

1. Build the image:
   ```bash
   cd container
   bash docker_build_dev.sh
   ```
   Equivalent to running, from the repo root: `docker build -f container/dev_Dockerfile -t harpia2_dev .`

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
   - `src/drone_planner` → `/home/drone_planner`
   - `src/lightweight_plume` → `/home/lightweight_plume`

   It also mounts the named Docker volume `harpia2_dev_vscode_server` at `/root/.vscode-server`,
   so VS Code's container server install is reused when the dev container is recreated.

   Because the project itself isn't built into the image, build it inside the container before
   running anything:
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd /home/drone_planner
   ./build.sh          # or ./f_compile_and_run.sh to force a full harpia_msgs rebuild
   ```
   ```bash
   cd /home/lightweight_plume
   colcon build
   source install/setup.bash
   ```
   Since the source directories are bind-mounted, edits made on the host are picked up immediately —
   just re-run `colcon build` inside the container after changing code.

## Notes

- Both `docker_run*.sh` scripts run the container with `--privileged --network=host` and mount
  `/tmp/.X11-unix` and `~/.Xauthority` for GUI (RViz) passthrough, plus `/dev/dri` for GPU access.
- If RViz's window still doesn't appear after `xhost +local:root`, re-run it as root as well.
- Both scripts invoke `docker` via `sudo -E` — expect a sudo password prompt if your shell isn't
  already elevated.
- `container/requirements.txt` pins the Python dependencies installed into both images; update it if
  the planner's Python dependencies change.
