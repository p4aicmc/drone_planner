# action_planner framework

Robot-agnostic ROS 2 planning framework: a PDDL knowledge base, external temporal
planners (OPTIC, TFD), plan execution over ROS 2 actions, mission control,
lifecycle orchestration and an operator CLI. It knows nothing about drones. A
robot system (for example the `drone_planner` workspace in `../src`) plugs in two
things: a **problem provider** that describes the world as PDDL, and one
**action executor node per PDDL action**.

This directory is a self-contained colcon workspace. The ROS package, its python
module and its main node are all called `action_planner`; the node's executable is
`action_planner_node.py` (a script named `action_planner.py` would shadow the module
on `sys.path`).

```
action_planner/
  build.sh, run_example.sh
  src/
    action_planner_msgs/          ActionCaller.action, ExecutePlan.action, StrInOut.srv
    action_planner/
      action_planner/   importable python module (ActionExecutorBase, TimedActionNode, planner internals)
      scripts/            ROS executables (nodes + CLI)
      solver/OPTIC, TFD   planners and their generate_plan.sh wrappers
      examples/, launch/  standalone example
```

## Build and run standalone

Requirements: ROS 2 Jazzy (Humble also works), the pip packages in
`src/action_planner/requirements.txt`, and for OPTIC the CBC solver library:

```bash
sudo apt install coinor-libcbc3.1
sudo ln -sf /usr/lib/x86_64-linux-gnu/libCbc.so.3.1 /usr/lib/x86_64-linux-gnu/libCbc.so.3
python3 -m pip install --break-system-packages --ignore-installed -r src/action_planner/requirements.txt
```

```bash
./build.sh          # sources ROS, colcon build
./run_example.sh    # ros2 launch action_planner example_launch.py [solver:=OPTIC|TFD]
```

The example brings up the framework nodes, a static problem provider
(`examples/example_problem.json`) and two stub action executors named `move` and
`inspect` after the actions of `examples/example_domain.pddl`. With no CLI
attached the mission auto-starts; the log should show the nodes activating in
order, `Generated plan with 5 actions.`, each `move`/`inspect` step executing,
then `Mission completed successfully`. The generated `domain.pddl` and
`problem.pddl` are copied to `action_planner/output/`.

To drive it by hand, start the CLI in another terminal before launching (its
presence makes `mission_controller` wait for `start_mission`):

```bash
source install/setup.bash
ros2 run action_planner action_planner_cli.py            # interactive
ros2 run action_planner action_planner_cli.py run move hall lab   # one-shot
```

## Using the framework from your own workspace

Either build everything in one go from your workspace root:

```bash
colcon build --base-paths . /path/to/harpia2/action_planner/src
```

or build this workspace once and overlay it (`source action_planner/install/setup.bash`
before building yours). The drone workspace in `../src` uses the first form
(`src/build.sh`). Your package then declares `<depend>action_planner</depend>`
and `<depend>action_planner_msgs</depend>` and imports:

```python
from action_planner import ActionExecutorBase, TimedActionNode
from action_planner_msgs.action import ActionCaller, ExecutePlan
from action_planner_msgs.srv import StrInOut
```

## The two contracts a robot implements

### 1. Problem provider

A node (any name; `mission_controller` is pointed at it through parameters) that
serves the current planning problem.

| Interface | Type | Default name | Content |
|---|---|---|---|
| service | `std_srvs/Trigger` | `problem_generator/get_problem` | `message` is JSON `{"instances": [...], "predicates": [...], "functions": [...], "goals": [...]}` |
| topic (optional) | `std_msgs/String` | `problem_generator/problem_update` | JSON list of update commands (below). Triggers cancel + replan. |

Every list entry is a space-separated PDDL string: instances `"r1 region"`
(name then type), predicates and goals `"at r1"`, functions `"(distance r1 r2) 10.0"`.
`scripts/static_problem_provider.py` is the minimal implementation; the drone
system's `problem_generator` derives the same JSON from its map, hardware and
mission files.

### 2. Action executors

For every durative action in the domain there is a lifecycle node **whose node
name is the action name**. It serves the `action_planner_msgs/ActionCaller` action on
`/action/<name>`; the planner sends each plan step there with the PDDL
arguments, in order, in `parameters` (strings). Subclass
`action_planner.ActionExecutorBase` and implement:

| Method | Called | Return |
|---|---|---|
| `new_goal(goal_request)` | on goal arrival | `True` to accept |
| `execute_goal(goal_handle)` | once per second until done | `(finished: bool, status: float 0..1)` |
| `cancel_goal_request(goal_handle)` | on cancel request | `True` to accept |
| `cancel_goal(goal_handle)` | when a cancel is honoured | nothing |

Set `self._goal_success = False` before returning `finished=True` to report
failure. The optional `on_configure_extension` / `on_activate_extension` /
`on_deactivate_extension` / `on_cleanup_extension` hooks run inside the
corresponding lifecycle transitions. `TimedActionNode` is a ready-made stub
that succeeds after `n_iterations` ticks; launching
`timed_action_node.py` with `name='<action>'` gives you a placeholder executor.

Because the base class uses the runtime node name, one executable can serve
several actions through launch-file renaming.

## Nodes and parameters

All framework nodes except `command_interface` are lifecycle nodes and are
expected to be brought up by `lifecycle_manager`.

| Node | Role | Parameters |
|---|---|---|
| `lifecycle_manager.py` | Configures then activates managed nodes once their dependencies are `active`. | `managed_nodes` (string array, required): `"node"` or `"node:dep1,dep2"` |
| `action_planner_node.py` | Holds the PDDL state (`ActionPlannerMemory`, built on `unified_planning`), writes domain/problem, calls `solver/<solver>/generate_plan.sh`, and steps the plan through `/action/<name>` servers (`ActionPlannerExecutor`). Serves `action_planner/execute_plan` (`ExecutePlan`), `action_planner/update_parameters` and `action_planner/command` (`StrInOut`). | `pddl_domain` (path, required), `solver` (`OPTIC` default, or `TFD`), `output_dir` (default `<workspace>/output`) |
| `mission_controller.py` | Fetches the problem, loads it into `action_planner`, runs the plan, retries on failure, replans on provider updates. Publishes `mission_status` (`MISSION_STARTED` / `MISSION_FINISHED`). Serves `mission_controller/command`. | `problem_service`, `problem_update_topic` (defaults above) |
| `command_interface.py` | Operator entry point, service `command_interface/command` (`StrInOut`). Routes `start_mission` / `cancel_mission` to `mission_controller` and `run <action> [args]` / `cancel_action` to `action_planner`; refuses to start a mission while a manual action runs. | none |
| `action_planner_cli.py` | Interactive or one-shot client of `command_interface/command`. While interactive it serves `action_planner_cli/presence`, which switches `mission_controller` from auto-start to wait-for-`start_mission`. | env `ACTION_PLANNER_CLI_ACTIONS="a,b"` adds `run a` / `run b` tab completions |
| `static_problem_provider.py` | Example problem provider. | `problem_file` |
| `timed_action_node.py` | Example / placeholder action executor. | `n_iterations` |

### `action_planner/update_parameters` commands

The request is a JSON list of `{"type": ..., "values": [...]}`:
`add_instances`, `add_predicates`, `remove_predicates`, `add_functions` /
`update_functions`, `set_goals` (replaces all goals), `clear_goal`,
`clear_knowledge`, `log_pddl`. `mission_controller` sends the provider's problem
as `add_instances`, `add_predicates`, `add_functions`, `set_goals`.

### Concurrency notes

Goal callbacks of `action_planner` and of every `ActionExecutorBase` node run in a
reentrant callback group on a multithreaded executor, so two goal requests can be
evaluated at the same time. Both claim their single slot under a lock before doing
any work and reject the other request, so a plan is never generated twice and an
executor never runs two goals at once. `lifecycle_manager` re-sends its initial
`get_state` request when a node has not answered within 2 s, because a reply lost
while DDS discovery is still settling would otherwise stall start-up silently.

### Solvers

`solver/<NAME>/generate_plan.sh <domain> <problem>` must print the plan to stdout
one step per line in OPTIC's format, `t: (action arg1 arg2) [duration]`, and
exit non-zero on failure. `OPTIC` (`optic-clp`, needs libCbc) is the default;
`TFD` is Temporal Fast Downward (prebuilt binaries + python translator). Add a
planner by adding a directory with such a script and passing its name as the
`solver` parameter.
