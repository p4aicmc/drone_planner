#!/usr/bin/env python3

import os, sys, rclpy, json
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
package_share_path = get_package_share_directory("route_executor2")
scripts_path = os.path.join(package_share_path, 'scripts')
sys.path.append(scripts_path)

from action_executor_base import ActionExecutorBase
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState


import rclpy
from rclpy.parameter import Parameter, ParameterType
from harpia_msgs.srv import GenerateSurveyPath
from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String


class ActionNodeExample(ActionExecutorBase):

    def __init__(self):
        super().__init__("survey")
        self.get_logger().info("survey initialized")

    def on_configure_extension(self):
        self.is_action_running = False
        self.waypoints = []  # Initialize waypoints as an empty list
        self.current_waypoint_index = 0  # Initialize the index
        self.mission_waypoint_reference_index = 0  # Persisted across replans
        self.latest_plume_parameters = {
            "focus": [0.0, 0.0],
            "plume_lenth": 150.0,
            "plume_angle": 30.0,
            "plume_direction": [1.0, 0.0],
        }
        self._active_move_goal_handle = None
        self._active_move_goal_token = 0
        self._next_move_goal_token = 1
        self._latest_path_request_seq = 0
        self._next_path_request_seq = 1
        self._replan_in_progress = False
        self._replan_queued = False
        self._canceling_for_replan = False
        self._replan_skip_count = 0
        self.survey_path_gen_group = MutuallyExclusiveCallbackGroup()
        self.action_client_group = MutuallyExclusiveCallbackGroup()
        self.cli_survey_path_gen = self.create_client(
            GenerateSurveyPath, 
            'survey_path_gen/generate_path', 
            callback_group=self.survey_path_gen_group
        )
        self.plume_update_sub = self.create_subscription(
            String,
            'plume_update',
            self.plume_update_callback,
            10
        )
        while not self.cli_survey_path_gen.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('survey path gen service not available, waiting again...')
        self.action_client = ActionClient(
            self, 
            MoveTo, 
            '/drone/move_to_waypoint', 
            callback_group=self.action_client_group
        )

        self._can_receive_new_goal = True

        return TransitionCallbackReturn.SUCCESS
    

    def new_goal(self, goal_request) -> bool:
        region = goal_request.parameters[0]
        self.get_logger().info(f"Region: {region}")

        if not self._can_receive_new_goal:
            self.get_logger().info("Cannot receive new goal, action is already running")
            return False
        
        self.get_logger().info(f"New goal received {goal_request.parameters}")
        self.progress_ = 0.0
        pass # self.get_logger().info(f"Current action arguments: {goal_request}") # NOT_ESSENTIAL_PRINT
        self._can_receive_new_goal = False
        self.is_action_running = True
        self.mission_waypoint_reference_index = 0

        self.send_survey_path_gen()

        self.current_waypoint_index = 0
        self.waypoints = []
        
        return True

    def execute_goal(self, goal_handle):

        if not self.waypoints:
            # Skip processing if waypoints are not yet available
            # self.get_logger().info('Waiting for waypoints from the survey path gen...') # NOT_ESSENTIAL_PRINT
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
        if self._active_move_goal_handle is not None:
            self._active_move_goal_handle.cancel_goal_async()
            self._active_move_goal_handle = None
        self.is_action_running = False
        self._can_receive_new_goal = True
        self.current_waypoint_index = 0
        self.mission_waypoint_reference_index = 0

    def cancel_goal_request(self, goal_handle):
        self.get_logger().info("Cancel goal request received")
        return True
    

    ###################################################

    def send_survey_path_gen(self, reason="mission"):
        """
        Sends a request to the survey path gen asynchronously.

        Parameters
        ----------
        origin : str
            The starting point of the path.
        """

        self.get_logger().info("Sending request to survey path gen...") # NOT_ESSENTIAL_PRINT

        if not self.is_action_running:
            self.get_logger().info('Mission stoped, dont call survey_path_get') # NOT_ESSENTIAL_PRINT
            return

        self.req = GenerateSurveyPath.Request()
        self.req.focus = self.latest_plume_parameters["focus"]
        self.req.plume_lenth = self.latest_plume_parameters["plume_lenth"]
        self.req.plume_angle = self.latest_plume_parameters["plume_angle"]
        self.req.plume_direction = self.latest_plume_parameters["plume_direction"]

        self.get_logger().info(f"Sending request to survey path gen with parameters: {str(self.req)}") # NOT_ESSENTIAL_PRINT

        # Send the request asynchronously
        request_seq = self._next_path_request_seq
        self._next_path_request_seq += 1
        self._latest_path_request_seq = request_seq
        self.future = self.cli_survey_path_gen.call_async(self.req)
        self.future.add_done_callback(
            lambda future, request_seq=request_seq, reason=reason:
            self.survey_path_gen_response_callback(future, request_seq, reason)
        )

    def plume_update_callback(self, msg: String):
        """
        Receives plume updates from plume_updater and stores the latest values.
        """
        try:
            plume_parameters = json.loads(msg.data)
            self.latest_plume_parameters = {
                "focus": plume_parameters["focus"],
                "plume_lenth": plume_parameters["plume_lenth"],
                "plume_angle": plume_parameters["plume_angle"],
                "plume_direction": plume_parameters["plume_direction"],
            }
            self.get_logger().info(
                f"Updated plume parameters from plume_update topic: {self.latest_plume_parameters}"
            ) # NOT_ESSENTIAL_PRINT

            # Replan only while the mission is running and there is a path in execution.
            if self.is_action_running and (self._active_move_goal_handle is not None or len(self.waypoints) > 0):
                self.get_logger().info(
                    "Active survey path detected. Starting replanning using new plume parameters."
                ) # NOT_ESSENTIAL_PRINT
                self.replan_active_path()
        except Exception as e:
            self.get_logger().error(f"Invalid plume_update message: {e}")

    def replan_active_path(self):
        if not self.is_action_running:
            return

        if self._replan_in_progress:
            self._replan_queued = True
            return

        self._replan_in_progress = True
        self._canceling_for_replan = True
        self._replan_queued = False
        # Preserve mission progress in replans: skip as many new waypoints
        # as were already completed in the current path.
        self._replan_skip_count = self.mission_waypoint_reference_index

        if self._active_move_goal_handle is not None:
            cancel_future = self._active_move_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.replan_cancel_done_callback)
        else:
            self._canceling_for_replan = False
            self.send_survey_path_gen(reason="replan")

    def replan_cancel_done_callback(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to cancel active waypoint during replan: {e}")
        finally:
            self._active_move_goal_handle = None
            self._canceling_for_replan = False
            self.send_survey_path_gen(reason="replan")

    def survey_path_gen_response_callback(self, future, request_seq=None, reason="mission"):
        """
        Handles the response from the survey path gen service.

        Parameters
        ----------
        future : Future
            Future containing the service response.
        """
        try:
            if request_seq is not None and request_seq != self._latest_path_request_seq:
                return

            response = future.result()
            self.get_logger().info('response number of waypoints: ' + str(len(response.waypoints))) # NOT_ESSENTIAL_PRINT
            # if response.success: # this dont work
            if len(response.waypoints) <= 0:
                self.get_logger().error('Failed to generate path')
                if reason == "replan":
                    self._replan_in_progress = False
                    if self._replan_queued:
                        self._replan_queued = False
                        self.replan_active_path()
                return
            
            self.get_logger().info('Received waypoints from path planner:') # NOT_ESSENTIAL_PRINT
            for waypoint in response.waypoints:
                self.get_logger().info(f"x: {waypoint.pose.position.x:20.15f} y: {waypoint.pose.position.y:20.15f}") # NOT_ESSENTIAL_PRINT

            waypoints = response.waypoints
            if reason == "replan":
                skip_count = max(0, self._replan_skip_count)
                waypoints = waypoints[skip_count:]
                if len(waypoints) <= 0:
                    self.get_logger().error(
                        f'Replanned path has no waypoints after skipping the first {skip_count}.'
                    )
                    self._replan_in_progress = False
                    return

            self.waypoints = waypoints  # Assuming `waypoints` is part of the response
            self.current_waypoint_index = 0  # Reset index when waypoints are received
            if reason != "replan":
                self.mission_waypoint_reference_index = 0
            if reason == "replan":
                self._replan_in_progress = False
                if self._replan_queued:
                    self._replan_queued = False
                    self.replan_active_path()
                    return
            self.send_goal(self.waypoints[0])
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            if reason == "replan":
                self._replan_in_progress = False
                if self._replan_queued:
                    self._replan_queued = False
                    self.replan_active_path()
        
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

        goal_token = self._next_move_goal_token
        self._next_move_goal_token += 1

        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future, goal_token=goal_token: self.goal_response_callback(future, goal_token)
        )

    def feedback_callback(self, feedback_msg):
        """
        Receives feedback from the action server.

        Parameters
        ----------
        feedback_msg : MoveTo.FeedbackMessage
            Feedback message containing the distance to the waypoint.
        """
        feedback = feedback_msg.feedback

    def goal_response_callback(self, future, goal_token):
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

        self._active_move_goal_handle = goal_handle
        self._active_move_goal_token = goal_token

        # self.get_logger().info('Goal accepted') # NOT_ESSENTIAL_PRINT
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, goal_token=goal_token: self.get_result_callback(future, goal_token)
        )

    
    def get_result_callback(self, future, goal_token):
        """
        Processes the result of a goal after completion.

        Parameters
        ----------
        future : Future
            Future containing the result of the goal.
        """
        if goal_token != self._active_move_goal_token:
            return

        self._active_move_goal_handle = None
        result = future.result().result
        succeeded = bool(getattr(result, 'success', result))
        if succeeded:
            # self.get_logger().info('Waypoint reached successfully') # NOT_ESSENTIAL_PRINT
            self.current_waypoint_index += 1
            self.mission_waypoint_reference_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                next_waypoint = self.waypoints[self.current_waypoint_index]
                self.send_goal(next_waypoint)
            else:
                pass # self.get_logger().info('All waypoints reached') # NOT_ESSENTIAL_PRINT
                self._active_move_goal_handle = None
                self._can_receive_new_goal = True
                self.is_action_running = False
        else:
            if self._canceling_for_replan or self._replan_in_progress:
                return
            self.get_logger().error('Failed to reach waypoint')


    # # survey path gen
    # def send_survey_path_gen(self, origin, destination):
    #     """
    #     Sends a request to the survey path gen asynchronously.

    #     Parameters
    #     ----------
    #     origin : str
    #         The starting point of the path.
    #     destination : str
    #         The destination point of the path.
    #     """

    #     if not self.is_action_running:
    #         self.get_logger().info('Mission stoped, dont call survey_path_get') # NOT_ESSENTIAL_PRINT
    #         return

    #     req = GeneratePath.Request()
    #     req.origin = origin
    #     req.destination = destination

    #     # Send the request asynchronously
    #     future = self.cli_survey_path_gen.call_async(req)
    #     future.add_done_callback(self.survey_path_gen_response_callback)

    def find_location_by_name(self, name):
        """
        Searches for a location by name in bases, ROIs, or NFZs.

        Parameters
        ----------
        name : str
            Name of the location to find.

        Returns
        -------
        tuple or None
            Coordinates (x, y) of the location, or None if not found.
        """
        for base in self.map.bases:
            if base['name'] == name:
                return base['center']

        for roi in self.map.rois:
            if roi['name'] == name:
                return roi['center']

        for nfz in self.map.nfz:
            if nfz['name'] == name:
                return nfz['center']

        return None

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
