from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import TransitionCallbackReturn

from action_planner.action_executor_base import ActionExecutorBase
from drone_planner_msgs.srv import DroneCommand


class DroneCommandActionNode(ActionExecutorBase):
    """Action executor that forwards a one-shot flight command (take_off, land)
    to drone_control's `/drone_control/command` service and reports its outcome."""
    def __init__(self, command_name, default_altitude=0.0):
        super().__init__(command_name)
        self.command_name = command_name
        self.default_altitude = default_altitude
        self.command_future = None
        self.last_status = 0.0

    def on_configure_extension(self):
        self.command_client = self.create_client(
            DroneCommand,
            '/drone_control/command',
            callback_group=ReentrantCallbackGroup()
        )

        while not self.command_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('/drone_control/command service not available, waiting again...')

        return TransitionCallbackReturn.SUCCESS

    def new_goal(self, goal_request):
        self.command_future = None
        self.last_status = 0.0
        self._goal_success = True
        return True

    def execute_goal(self, goal_handle):
        if self.command_future is None:
            request = DroneCommand.Request()
            request.command = self.command_name
            request.altitude = self.default_altitude
            self.command_future = self.command_client.call_async(request)
            return False, self.last_status

        if not self.command_future.done():
            return False, self.last_status

        try:
            response = self.command_future.result()
        except Exception as e:
            self.get_logger().error(f'{self.command_name} command failed: {e}')
            self._goal_success = False
            return True, self.last_status

        self.command_future = None

        if response is None:
            self.get_logger().error(f'{self.command_name} command returned no response')
            self._goal_success = False
            return True, self.last_status

        if not response.accepted:
            self.get_logger().error(f'{self.command_name} rejected: {response.message}')
            self._goal_success = False
            return True, self.last_status

        if response.finished:
            self._goal_success = response.success
            if response.success:
                self.last_status = 1.0
                self.get_logger().info(f'{self.command_name} completed: {response.message}')
            else:
                self.get_logger().error(f'{self.command_name} failed: {response.message}')
            return True, self.last_status

        self.last_status = min(0.95, self.last_status + 0.1)
        self.get_logger().info(f'{self.command_name} running: {response.message}')
        return False, self.last_status

    def cancel_goal(self, goal_handle):
        self.command_future = None

    def cancel_goal_request(self, goal_handle):
        return True
