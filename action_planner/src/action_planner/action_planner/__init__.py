"""action_planner framework.

Importable pieces for robot systems built on top of the framework:

- :class:`ActionExecutorBase` -- base class for action executor nodes. The node
  name is the PDDL action name and the node serves ``/action/<name>``.
- :class:`TimedActionNode` -- stub executor that reports success after a fixed
  number of ticks; useful as a placeholder while a real action is developed.

The planner internals (``action_planner.action_planner_memory`` and
``action_planner.action_planner_executor``) are deliberately not imported
here: they need the ``unified_planning`` pip package, which action executor
nodes should not have to carry.
"""

from action_planner.action_executor_base import ActionExecutorBase
from action_planner.timed_action import TimedActionNode

__all__ = [
    'ActionExecutorBase',
    'TimedActionNode',
]
