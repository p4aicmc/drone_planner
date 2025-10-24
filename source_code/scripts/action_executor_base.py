from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from rclpy.task import Future
import time

from harpia_msgs.action import ActionCaller

class ActionExecutorBase(LifecycleNode):

    def __init__(self, node_name):
        super().__init__(node_name)
        self._node_name = node_name
        self.get_logger().info("ActionExecutorBase initialized")

    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring... (base class)')
        return self.on_configure_extension()
    def on_configure_extension(self):
        """For child classes to override. Return SUCCESS by default."""
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating... (base class)')
        self.action_cb = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ActionCaller,
            '/action/'+self._node_name,
            execute_callback   = self.execute_cb,
            goal_callback      = self.goal_cb,
            cancel_callback    = self.cancel_cb,
            callback_group     = self.action_cb
        )

        return self.on_activate_extension()
    def on_activate_extension(self):
        """For child classes to override. Return SUCCESS by default."""
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating... (base class)')
        return self.on_deactivate_extension()
    def on_deactivate_extension(self):
        """For child classes to override. Return SUCCESS by default."""
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up... (base class)')
        return self.on_cleanup_extension()
    def on_cleanup_extension(self):
        """For child classes to override. Return SUCCESS by default."""
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down... (base class)')
        return self.on_shutdown_extension()
    def on_shutdown_extension(self):
        """For child classes to override. Return SUCCESS by default."""
        return TransitionCallbackReturn.SUCCESS
    


    # ---- callbacks -------------------------------------------------
    def goal_cb(self, goal_request):
        self.get_logger().info(f'Received goal: {str(goal_request)}')
        
        if self.new_goal(goal_request):
            return GoalResponse.ACCEPT 
        else:
            return GoalResponse.REJECT

    def cancel_cb(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT if self.cancel_goal_request(goal_handle) else CancelResponse.REJECT

    def execute_cb(self, goal_handle):
        self.get_logger().info(
            f"Executing action: {self.get_name()} "
            f"params: {goal_handle.request.parameters}")
        
        

        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                self.cancel_goal(goal_handle)
                goal_handle.canceled()
                return ActionCaller.Result()

            # self.get_logger().info(f"Executing step")
            finish, status = self.execute_goal(goal_handle)
            feedback = ActionCaller.Feedback()
            feedback.status = status
            goal_handle.publish_feedback(feedback)

            if finish:
                break
            time.sleep(1.0)

        self.get_logger().info('Goal succeeded')
        result = ActionCaller.Result()
        result.success = True
        goal_handle.succeed()
        return result

    # -------------------------------------------------------------------

    
    def new_goal(self, goal_request):
        raise NotImplementedError("Subclasses should implement this method.")
    
    def execute_goal(self, goal_handle):
        raise NotImplementedError("Subclasses should implement this method.")
    
    def cancel_goal(self, goal_handle):
        raise NotImplementedError("Subclasses should implement this method.")
    
    def cancel_goal_request(self, goal_handle):
        raise NotImplementedError("Subclasses should implement this method.")
