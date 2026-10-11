from rclpy.lifecycle import TransitionCallbackReturn

from action_planner.action_executor_base import ActionExecutorBase


class TimedActionNode(ActionExecutorBase):
    """Action executor that succeeds after ``n_iterations`` ticks (one tick per second).

    It does nothing else, which makes it a convenient stand-in for actions whose
    real implementation does not exist yet, and the executor used by the
    framework's standalone example. The PDDL action it implements is whatever
    the node is named (``name:=`` in a launch file or ``--ros-args -r __node:=``).
    """

    def __init__(self, node_name='timed_action', n_iterations=10):
        super().__init__(node_name)
        self.declare_parameter('n_iterations', n_iterations)
        self.n_iterations = max(1, int(self.get_parameter('n_iterations').value))
        self.index = 0

    def on_configure_extension(self):
        self.n_iterations = max(1, int(self.get_parameter('n_iterations').value))
        return TransitionCallbackReturn.SUCCESS

    def new_goal(self, goal_request):
        self.index = 0
        return True

    def execute_goal(self, goal_handle):
        self.index += 1
        if self.index >= self.n_iterations:
            self.get_logger().info('Goal completed')
            return True, 1.0
        self.get_logger().info(f'Goal status {self.index}/{self.n_iterations}')
        return False, self.index / self.n_iterations

    def cancel_goal(self, goal_handle):
        self.get_logger().info('Canceling goal')

    def cancel_goal_request(self, goal_handle):
        return True
