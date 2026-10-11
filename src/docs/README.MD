# drone_planner docs

- `architecture_diagram.drawio`: system architecture. To view and edit the diagram file, use the website: https://app.diagrams.net
  Note: the diagram predates the split into `action_planner/` (planning) and `src/` (drone system); the
  node graph is still accurate, the package boundaries are not.
- `node_initialization.md`: order in which `lifecycle_manager` brings the nodes up, as a Mermaid chart,
  with the dependencies and external prerequisites that gate each step.
- The framework's own documentation (contracts, node parameters, solvers) is in
  [`../../action_planner/README.md`](../../action_planner/README.md).
