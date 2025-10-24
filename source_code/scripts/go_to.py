#!/usr/bin/env python3

import os, sys, rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
package_share_path = get_package_share_directory("route_executor2")
scripts_path = os.path.join(package_share_path, 'scripts')
sys.path.append(scripts_path)

from action_executor_base import ActionExecutorBase
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState


import rclpy
from rclpy.parameter import Parameter, ParameterType
from harpia_msgs.srv import GeneratePath
from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ActionNodeExample(ActionExecutorBase):

    def __init__(self):
        super().__init__("go_to")
        self.get_logger().info("ActionNodeExample initialized")

    def on_configure_extension(self):
        self.is_action_running = False
        self.waypoints = []  # Initialize waypoints as an empty list
        self.current_waypoint_index = 0  # Initialize the index
        self.path_planner_group = MutuallyExclusiveCallbackGroup()
        self.action_client_group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(
            GeneratePath, 
            'path_planner/generate_path', 
            callback_group=self.path_planner_group
        )
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('path planner service not available, waiting again...')
        self.action_client = ActionClient(
            self, 
            MoveTo, 
            '/drone/move_to_waypoint', 
            callback_group=self.action_client_group
        )

        self._can_receive_new_goal = True

        return TransitionCallbackReturn.SUCCESS
    

    def new_goal(self, goal_request) -> bool:
        origin = goal_request.parameters[0]
        destination = goal_request.parameters[1]
        self.get_logger().info(f"Origin: {origin}, Destination: {destination}")

        if not self._can_receive_new_goal:
            self.get_logger().info("Cannot receive new goal, action is already running")
            return False
        
        self.get_logger().info(f"New goal received {goal_request.parameters}")
        self.progress_ = 0.0
        pass # self.get_logger().info(f"Current action arguments: {goal_request}") # NOT_ESSENTIAL_PRINT
        self._can_receive_new_goal = False
        self.is_action_running = True
        self.send_path_planner(origin, destination)  # Now asynchronous

        self.current_waypoint_index = 0
        self.waypoints = []
        
        return True

    def execute_goal(self, goal_handle):

        if not self.waypoints:
            # Skip processing if waypoints are not yet available
            # self.get_logger().info('Waiting for waypoints from the path planner...') # NOT_ESSENTIAL_PRINT
            return False, self.progress_

        self.progress_ = self.current_waypoint_index / len(self.waypoints)
        # self.get_logger().info(f"progress: {self.current_waypoint_index}/{len(self.waypoints)} = {self.progress_}")

        if self.progress_ < 1.0:
            return False, self.progress_
        else:
            self._can_receive_new_goal = True
            self.is_action_running = False
            return True, 1.0

    def cancel_goal(self, goal_handle):
        self.get_logger().info("Canceling goal")
        self.is_action_running = False
        self._can_receive_new_goal = True

    def cancel_goal_request(self, goal_handle):
        self.get_logger().info("Cancel goal request received")
        return True
    

    ###################################################

    def send_path_planner(self, origin, destination):
        """
        Sends a request to the path planner asynchronously.

        Parameters
        ----------
        origin : str
            The starting point of the path.
        destination : str
            The destination point of the path.
        """

        if not self.is_action_running:
            self.get_logger().info('Mission stoped, dont call path_planner') # NOT_ESSENTIAL_PRINT
            return

        self.req = GeneratePath.Request()
        self.req.origin = origin
        self.req.destination = destination

        # Send the request asynchronously
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.path_planner_response_callback)

    def path_planner_response_callback(self, future):
        """
        Handles the response from the path planner service.

        Parameters
        ----------
        future : Future
            Future containing the service response.
        """
        try:
            response = future.result()
            self.get_logger().info('response number of waypoints: ' + str(len(response.waypoints))) # NOT_ESSENTIAL_PRINT
            # if response.success: # this dont work
            if len(response.waypoints) <= 0:
                self.get_logger().error('Failed to generate path')
                return
            
            self.get_logger().info('Received waypoints from path planner:') # NOT_ESSENTIAL_PRINT
            for waypoint in response.waypoints:
                self.get_logger().info(f"x: {waypoint.pose.position.x:20.15f} y: {waypoint.pose.position.y:20.15f}") # NOT_ESSENTIAL_PRINT

            self.waypoints = response.waypoints  # Assuming `waypoints` is part of the response
            self.current_waypoint_index = 0  # Reset index when waypoints are received
            self.send_goal(self.waypoints[0])
                # self.finish(False, 0.0, 'Failed to generate path')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            # self.finish(False, 0.0, 'Service call exception')
        
    def send_goal(self, waypoint):
        """
        Sends a waypoint as a goal to the action server.

        Parameters
        ----------
        waypoint : PoseStamped
            Target waypoints messages.
        """

        if not self.is_action_running:
            self.get_logger().info('Mission stoped, dont send waypoint') # NOT_ESSENTIAL_PRINT
            return


        # self.get_logger().info("sending waypoint") # NOT_ESSENTIAL_PRINT
        goal_msg = MoveTo.Goal()
        goal_msg.destination = waypoint

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("waiting...") # NOT_ESSENTIAL_PRINT

        # self.get_logger().info(f"Sending waypoint goal: x={waypoint.pose.position.x}, y={waypoint.pose.position.y}, z={waypoint.pose.position.z}") # NOT_ESSENTIAL_PRINT

        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """
        Receives feedback from the action server.

        Parameters
        ----------
        feedback_msg : MoveTo.FeedbackMessage
            Feedback message containing the distance to the waypoint.
        """
        feedback = feedback_msg.feedback

    def goal_response_callback(self, future):
        """
        Handles the response from the action server after sending a goal.

        Parameters
        ----------
        future : Future
            Future containing the goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        # self.get_logger().info('Goal accepted') # NOT_ESSENTIAL_PRINT
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    
    def get_result_callback(self, future):
        """
        Processes the result of a goal after completion.

        Parameters
        ----------
        future : Future
            Future containing the result of the goal.
        """
        result = future.result().result
        if result:
            # self.get_logger().info('Waypoint reached successfully') # NOT_ESSENTIAL_PRINT
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                next_waypoint = self.waypoints[self.current_waypoint_index]
                self.send_goal(next_waypoint)
            else:
                pass # self.get_logger().info('All waypoints reached') # NOT_ESSENTIAL_PRINT
                self._can_receive_new_goal = True
                self.is_action_running = False
        else:
            self.get_logger().error('Failed to reach waypoint')


    # Path planner
    def send_path_planner(self, origin, destination):
        """
        Sends a request to the path planner asynchronously.

        Parameters
        ----------
        origin : str
            The starting point of the path.
        destination : str
            The destination point of the path.
        """

        if not self.is_action_running:
            self.get_logger().info('Mission stoped, dont call path_planner') # NOT_ESSENTIAL_PRINT
            return

        req = GeneratePath.Request()
        req.origin = origin
        req.destination = destination

        # Send the request asynchronously
        future = self.cli.call_async(req)
        future.add_done_callback(self.path_planner_response_callback)

    def path_planner_response_callback(self, future):
        """
        Handles the response from the path planner service.

        Parameters
        ----------
        future : Future
            Future containing the service response.
        """
        try:
            response = future.result()
            self.get_logger().info('response number of waypoints: ' + str(len(response.waypoints))) # NOT_ESSENTIAL_PRINT
            # if response.success: # this dont work
            if len(response.waypoints) > 0:
                pass # self.get_logger().info('Received waypoints from path planner:') # NOT_ESSENTIAL_PRINT
                for waypoint in response.waypoints:
                    pass # self.get_logger().info(f"x: {waypoint.pose.position.x:20.15f} y: {waypoint.pose.position.y:20.15f}") # NOT_ESSENTIAL_PRINT

                self.waypoints = response.waypoints  # Assuming `waypoints` is part of the response
                self.current_waypoint_index = 0  # Reset index when waypoints are received
                self.send_goal(self.waypoints[0])
            else:
                self.get_logger().error('Failed to generate path')
                # self.finish(False, 0.0, 'Failed to generate path')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            # self.finish(False, 0.0, 'Service call exception')


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