# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Harpia 2 is a ROS 2 system for autonomous UAV missions, split in two parts:

- **`action_planner/`**: the action_planner framework, a standalone colcon workspace. A PDDL knowledge
  base built on `unified_planning`, external temporal planners (OPTIC, TFD), plan execution over ROS 2
  actions, mission control, lifecycle orchestration and an operator CLI. It knows nothing about drones.
- **`src/`**: the drone system, a colcon workspace that imports the framework. Flight control over
  MAVROS/PX4, RRT* path planning, mission/map data, the PDDL domain and the action executor nodes.

Everything is Python (`rclpy`) except the vendored planner binaries. The two workspaces are built
together into `src/install` by `src/build.sh` (`colcon build --base-paths . ../action_planner/src`), so one
`source src/install/setup.bash` covers both. The framework can also be built and run on its own.

`action_planner/README.md` documents the framework's contracts and node parameters. Read it before
touching planning or action-execution code. The package, its python module and its main node share
the name `action_planner`; the node's executable is `action_planner_node.py` on purpose, since a
script called `action_planner.py` next to the other scripts would shadow the module.

## Layout

```
action_planner/src/action_planner_msgs/          ActionCaller.action, ExecutePlan.action, StrInOut.srv
action_planner/src/action_planner/
  action_planner/                         python module: ActionExecutorBase, TimedActionNode,
                                            ActionPlannerMemory, ActionPlannerExecutor
  scripts/                                  action_planner, mission_controller, command_interface,
                                            lifecycle_manager, action_planner_cli, example nodes
  solver/{OPTIC,TFD}/generate_plan.sh       planner wrappers (binaries are tracked in git)
  examples/, launch/example_launch.py       standalone demo
src/src/drone_planner_msgs/                 DroneSetpoint, DroneCommand, GeneratePath, RequestControl,
                                            ReleaseControl, MoveTo
src/src/drone_planner/
  drone_planner/drone_command_action_base.py  python module shared by take_off/land
  scripts/                                  drone nodes; launch/harpia_launch.py; pddl/; data/; libs/
scripts/                                    PX4 + MAVROS service runners, CLI wrapper, MAVROS debug GUI
container/                                  Dockerfiles (ROS Jazzy, Gazebo Harmonic, PX4 SITL)
map_generator/                              standalone PyQt5 KML -> map.json tool, not part of the ROS build
```

## Build and run

```bash
cd src
./build.sh                  # ROS Jazzy (Humble fallback); colcon build of framework + drone packages
./run.sh [mission_index]    # sources src/install/setup.bash, ros2 launch drone_planner harpia_launch.py
./f_compile_and_run.sh [N]  # both

cd framework
./build.sh && ./run_example.sh   # framework alone: plan + execute a toy domain with stub actions
```

Launch arguments: `mission_index:=N` (entry of `data/all_missions.json`), `solver:=OPTIC|TFD`.
`run.sh` pipes output through `src/process_output.py` (log colorizer) and recreates `src/output/`,
where `action_planner` dumps the generated `domain.pddl` / `problem.pddl`.

**No `--symlink-install`.** Nodes run from `install/`; every Python edit in either workspace needs a
rebuild (`./build.sh`) before it takes effect.

**After pulling the framework split** (older checkouts had `harpia_msgs` under `src/src` with a nested
install): `rm -rf src/build src/install src/log src/src/harpia_msgs`, then rebuild. `build.sh` warns
about this but does not clean.

**Framework location.** `src/build.sh` looks for the framework at `$ACTION_PLANNER_DIR`, then
`../action_planner`, then `/home/action_planner` (container).

There are no automated tests. `pddl/harpia_domain_test.pddl` is the live domain used by the launch
file, not a test fixture. The framework example (`action_planner/run_example.sh`) is the closest thing to
an end-to-end check and needs no robot.

### Simulation stack (PX4 SITL + MAVROS)

```bash
scripts/run_all.sh    # run_px4.sh (HEADLESS gz_x500 from /home/PX4-Autopilot) + run_mavros.sh as
                      # detached services; PIDs/logs under /tmp/drone_planner_exxon
scripts/stop_all.sh
```

These hardcode Jazzy and container paths. The production container entrypoint runs `run_all.sh`;
the dev container does not.

### Container

Run from `container/`. Images: `harpia2_drone_planner` (copies `action_planner/` and `src/`, runs
`build.sh` at image build) and `harpia2_drone_planner_dev` (deps only; mounts `action_planner/` ->
`/home/action_planner`, `src/` -> `/home/drone_planner`, `scripts/` -> `/home/scripts`).
`.dockerignore` keeps host build dirs out of the context.

### Operator CLI

```bash
scripts/run_drone_planner_cli.sh [command]   # sources the workspace, exec ros2 run action_planner action_planner_cli.py
scripts/debug_mavros.py                      # Tk GUI for MAVROS modes/arming
```

Commands: `start_mission`, `cancel_mission`, `run <action> [args...]`, `cancel_action`. While the
interactive CLI is open it serves `action_planner_cli/presence`, so `mission_controller` loads the problem
and waits for `start_mission` instead of auto-starting.

## Architecture

### Startup is gated by `lifecycle_manager`

The launch file starts every node at once. Most are `LifecycleNode`s and begin unconfigured. The
framework's `lifecycle_manager` configures then activates each node only once all its dependencies
are `active`, following the `managed_nodes` list in `src/src/drone_planner/launch/harpia_launch.py`
(entries `"node"` or `"node:dep1,dep2"`). Rough order: `drone_control` -> `data_server`, `take_off`,
`land` -> `problem_generator`, `path_planner` -> `go_to` -> `action_planner` -> `mission_controller`.
`drone_control` activation blocks on MAVROS services, so nothing past step 1 comes up without MAVROS.
Details and failure modes: `src/docs/node_initialization.md`.

### Planning pipeline (framework nodes in italics)

1. `data_server` loads `data/map.json`, `data/hardware.json` and the `mission_index` entry of
   `data/all_missions.json`; serves them as JSON over `Trigger` services and publishes the map.
   `data_server/update_map` (`StrInOut`) accepts map patches (`map_updater.py` is a demo client).
2. `problem_generator` is the drone's **problem provider**: bases and ROIs become `base` / `region`
   instances, pairwise haversine distances become `(distance a b)` functions, and mission commands
   map `take_picture` -> `picture_goal` predicate + `taken_image` goal, `end` -> `at <base>` goal.
   Served on `problem_generator/get_problem`; republished on `problem_generator/problem_update` when
   the mission or map changes (which makes *mission_controller* cancel and replan).
3. *mission_controller* pushes the problem to *action_planner* through
   `action_planner/update_parameters` (`StrInOut`, JSON command list), then calls the
   `action_planner/execute_plan` action.
4. *action_planner* keeps state in `ActionPlannerMemory` (singleton over `unified_planning`), writes
   `domain.pddl` + `problem.pddl` under `<framework share>/solver/`, runs
   `solver/<solver>/generate_plan.sh`, and parses `t: (action args) [dur]` lines. `solver` is a
   parameter (default OPTIC).
5. `ActionPlannerExecutor` walks the plan, checks each step's conditions against memory, sends it as
   an `ActionCaller` goal to `/action/<action_name>` with the PDDL arguments as `parameters`, and
   applies effects on success.

### Action naming contract

A PDDL durative-action name, the ROS node name and the action server path `/action/<name>` are the
same string. Every executor subclasses `action_planner.ActionExecutorBase`, which owns the
`ActionCaller` server (on the runtime node name, so launch-file renaming works) and polls
`execute_goal()` once per second until it returns `(True, status)`. `take_off` / `land` are
`DroneCommandActionNode`s that call `/drone_control/command`; `take_image` / `recharge_battery` are
`TimedActionNode` stubs.

Adding a drone action means touching: the domain PDDL, a node subclassing `ActionExecutorBase`,
`install(PROGRAMS ...)` in `src/src/drone_planner/CMakeLists.txt`, a `Node(...)` entry plus a
`managed_nodes` entry (and `action_planner`'s dependency list) in `harpia_launch.py`, a
`problem_generator` mapping if a mission command should produce it, and `ACTION_PLANNER_CLI_ACTIONS` in
`scripts/run_drone_planner_cli.sh` for tab completion.

The launch file passes `pddl/harpia_domain_test.pddl` (has `take_off` / `land`);
`harpia_domain.pddl` is the older variant without them.

### Flight stack (drone side)

`drone_control` is the only node that talks to MAVROS. It owns a single control lease
(`request_control` / `release_control`, `drone_planner_msgs/srv`); while leased, the holder publishes
`DroneSetpoint` on `/drone_control/setpoint/local` and `drone_control` forwards them at
`setpoint_rate_hz`. It also executes `take_off` / `land` via `/drone_control/command`.

`route_executor` is the lease holder for navigation: it serves the `MoveTo` action
`/drone/move_to_waypoint`, streams setpoints until within 1 m, then releases. `go_to` calls
`path_planner/generate_path` (`GeneratePath`: region names -> waypoints, RRT* from `libs/RRT`) and
sends each waypoint as a `MoveTo` goal.

### Command flow

CLI -> `command_interface/command` (*command_interface*) -> `mission_controller/command` for
`start_mission` / `cancel_mission`, or `action_planner/command` for `run <action> [args]` /
`cancel_action`. *command_interface* first asks *action_planner* `can_start_plan` so a manual action
and a mission never overlap.

## Dependencies worth knowing

Python deps are pinned in `container/requirements.txt` (superset of
`action_planner/src/action_planner/requirements.txt`: `unified_planning==1.2.0` is the framework's,
`shapely` / `scipy` / `termcolor` are the drone's). OPTIC needs `coinor-libcbc` with a `libCbc.so.3`
symlink (the Dockerfile does this). Action executor nodes import only `action_planner`'s base
classes, which do not need `unified_planning`; only `action_planner` does.
