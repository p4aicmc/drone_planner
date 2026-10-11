# Node initialization sequence

`harpia_launch.py` starts every process at once. The order in which the
lifecycle nodes actually come up is decided at runtime by the framework's
`lifecycle_manager`: it configures and activates a node only once every node it
depends on is already `active`. The dependency table is the `managed_nodes`
parameter passed by `launch/harpia_launch.py` (one entry per node,
`"node"` or `"node:dep1,dep2"`). The chart reads left to right: each numbered
step lists the nodes that come up together once the previous step is fully
active.

```mermaid
flowchart LR
    mavros[("MAVROS<br/>arming, set_mode, land<br/>services")]

    subgraph s1["1"]
        drone_control["drone_control"] ~~~ take_image["take_image"] ~~~ recharge_battery["recharge_battery"]
    end

    subgraph s2["2"]
        data_server["data_server"] ~~~ take_off["take_off"] ~~~ land["land"]
    end

    subgraph s3["3"]
        problem_generator["problem_generator"] ~~~ path_planner["path_planner"]
    end

    subgraph s4["4"]
        go_to["go_to"]
    end

    subgraph s5["5"]
        action_planner["action_planner"]
    end

    subgraph s6["6"]
        mission_controller["mission_controller"]
    end

    ready(["mission loaded<br/>and started"])

    mavros -.-> s1
    s1 --> s2 --> s3 --> s4 --> s5 --> s6 --> ready

    classDef drone fill:#e8f0fe,stroke:#2a78d6,color:#0b0b0b
    classDef framework fill:#e6f5ea,stroke:#2e8b57,color:#0b0b0b
    classDef external fill:#fdeee6,stroke:#eb6834,color:#0b0b0b
    classDef plain fill:#f4f4f2,stroke:#52514e,stroke-dasharray:4 2,color:#0b0b0b
    class drone_control,take_image,recharge_battery,data_server,take_off,land,problem_generator,path_planner,go_to drone
    class action_planner,mission_controller framework
    class mavros external
    class ready plain
```

Blue nodes belong to `drone_planner`, green nodes to `action_planner`.
Each numbered step waits for every node of the previous step to be `active`
before its own nodes are configured and activated. Nodes inside the same step
come up concurrently. MAVROS must be running before step 1 completes, because
`drone_control` waits for its services during activation. `lifecycle_manager`,
`command_interface` (framework) and `route_executor` (drone) are plain nodes
that start immediately with the launch file and are not part of the sequence.

Why each step depends on the previous one, from `managed_nodes` in
`harpia_launch.py`:

| Step | Nodes | Package | Wait for |
|---|---|---|---|
| 1 | drone_control, take_image, recharge_battery | drone_planner | nothing |
| 2 | data_server, take_off, land | drone_planner | drone_control |
| 3 | problem_generator, path_planner | drone_planner | data_server |
| 4 | go_to | drone_planner | path_planner (and the `/drone/move_to_waypoint` action server of route_executor) |
| 5 | action_planner | action_planner | go_to, take_image, recharge_battery, take_off, land |
| 6 | mission_controller | action_planner | action_planner, problem_generator |


## How the manager decides

- **Discovery.** For every entry in its table the manager polls the node's
  `get_state` service every 0.2 s. A node that does not answer within 100 tries
  (about 20 s) is logged as not found and dropped from the table. Any node that
  lists it as a dependency then makes the manager raise a `KeyError` on its next
  tick, so startup aborts rather than stalls.
- **Start.** Once every managed node has reported a state, the manager starts a
  0.5 s timer. On each tick it walks the table and, for every node whose
  dependencies are all `active`, sends `configure` if the node is
  `unconfigured` or `activate` if it is `inactive`. Only one transition is in
  flight per node at a time.
- **Steps.** Because a node needs two ticks (configure, then activate) plus its
  own `on_configure` and `on_activate` work, each step adds at least a second
  before the next one can start. Nodes in the same step transition
  concurrently.
- **Done.** When every managed node is `active` the manager logs
  "Finished activating all lifecycle nodes" and stops its timer.

## What blocks a hung startup

- **MAVROS.** `drone_control` is in step 1 and every later step hangs off it.
  Its `on_activate` waits up to 10 s for `/mavros/cmd/arming`,
  `/mavros/set_mode` and `/mavros/cmd/land` and returns `ERROR` if any is
  missing. If MAVROS is not up, nothing beyond step 1 is even configured.
- **route_executor.** `go_to` blocks in `on_activate` until the
  `/drone/move_to_waypoint` action server exists. That server is
  `route_executor`, a plain node started by the launch file, so it is normally
  already there; if it crashed, step 4 never completes.
- **data_server services.** `path_planner` blocks in `on_configure` until
  `data_server/home_position` and `data_server/drone_position` are available,
  and `mission_controller` fails activation if `problem_generator/get_problem`
  or `action_planner/update_parameters` is not up within 5 s. The dependency
  table already orders these correctly, so they only matter if an earlier node
  activated without actually creating its services.

## Nodes outside the manager

- `lifecycle_manager` itself, `command_interface` and `route_executor` are
  plain `rclpy` nodes. They start as soon as the launch file runs them and are
  not gated on anything, although `route_executor` only requests control of
  the drone from `drone_control` when its first goal arrives.
- `command_interface` forwards commands from the CLI (`command_interface/command`) to
  `mission_controller` and `action_planner`, so it is only useful once step 6
  is done.

## After activation

When `mission_controller` activates it checks whether the action_planner CLI is
present (`action_planner_cli/presence`). It then asks `problem_generator` for the PDDL
problem. Without the CLI it starts the mission right away; with the CLI it
loads the problem and waits for a `start_mission` command.
