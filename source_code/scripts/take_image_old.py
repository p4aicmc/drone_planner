#!/usr/bin/env python3

import os, sys, rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
package_share_path = get_package_share_directory("route_executor2")
scripts_path = os.path.join(package_share_path, 'scripts')
sys.path.append(scripts_path)

from action_executor_base import ActionExecutorBase
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState

class ActionNodeExample(ActionExecutorBase):

    def __init__(self):
        super().__init__("take_image")
        self.get_logger().info("ActionNodeExample initialized")

        self.index = 0
        self.n_iterations = 10

    def on_configure_extension(self):
        self.get_logger().info("Configuring... (example)")
        return TransitionCallbackReturn.SUCCESS
    

    def new_goal(self, goal_request):
        self.get_logger().info("New goal received")
        self.index = 0
        return True

    def execute_goal(self, goal_handle):

        # self.get_logger().info("action name: " + str(goal_handle.request.action_name))
        # self.get_logger().info("parameters: " + str(goal_handle.request.parameters))
        # self.get_logger().info("Executing goal")
        self.index += 1

        if self.index >= self.n_iterations:
            self.get_logger().info("Goal completed")
            return True, 1.0
        
        self.get_logger().info(f"Goal status {self.index}/{self.n_iterations}")
        return False, self.index/self.n_iterations   

    def cancel_goal(self, goal_handle):
        self.get_logger().info("Canceling goal")

    def cancel_goal_request(self, goal_handle):
        self.get_logger().info("Cancel goal request received")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ActionNodeExample()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()