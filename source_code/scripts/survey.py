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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
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
        self.plumes = {}  # plume_name -> plume params dict, loaded from map
        self.target_plume_name = None  # name of the plume being surveyed
        self._active_move_goal_handle = None
        self._active_move_goal_token = 0
        self._next_move_goal_token = 1
        self._latest_path_request_seq = 0
        self._next_path_request_seq = 1
        self._replan_in_progress = False
        self._replan_queued = False
        self._canceling_for_replan = False
        self.survey_path_gen_group = MutuallyExclusiveCallbackGroup()
        self.action_client_group = MutuallyExclusiveCallbackGroup()
        self.cli_survey_path_gen = self.create_client(
            GenerateSurveyPath, 
            'survey_path_gen/generate_path', 
            callback_group=self.survey_path_gen_group
        )

        # Subscribe to the latched map topic to get plume definitions
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(
            String,
            'data_server/map',
            self.map_update_callback,
            map_qos
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
        plume_name = goal_request.parameters[0]
        self.get_logger().info(f"Plume: {plume_name}")

        if not self._can_receive_new_goal:
            self.get_logger().info("Cannot receive new goal, action is already running")
            return False
        
        self.get_logger().info(f"New goal received {goal_request.parameters}")
        self.progress_ = 0.0
        self._can_receive_new_goal = False
        self.is_action_running = True

        if plume_name not in self.plumes:
            self.get_logger().error(f"Plume '{plume_name}' not found in map, cannot start survey")
            self._can_receive_new_goal = True
            self.is_action_running = False
            return False

        self.target_plume_name = plume_name
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

        plume = self.plumes.get(self.target_plume_name)
        if plume is None:
            self.get_logger().error(f"Target plume '{self.target_plume_name}' not in map, cannot generate path")
            return

        self.req = GenerateSurveyPath.Request()
        self.req.focus_lat = plume["focus_lat"]
        self.req.focus_lon = plume["focus_lon"]
        self.req.plume_lenth = plume["plume_lenth"]
        self.req.plume_angle = plume["plume_angle"]
        self.req.plume_direction = plume["plume_direction"]

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

    @staticmethod
    def _plume_to_params(plume):
        """Convert a plume dict from the map into the flat param dict used internally."""
        focus = plume["focus"]  # [lon, lat, alt] — same convention as map.json
        return {
            "focus_lat": focus[1],
            "focus_lon": focus[0],
            "plume_lenth": plume["plume_length"],
            "plume_angle": plume["plume_angle"],
            "plume_direction": plume["plume_direction"],
        }

    def map_update_callback(self, msg: String):
        """
        Receives map updates from the data_server latched topic and extracts plume parameters.
        """
        try:
            map_data = json.loads(msg.data)
            plume_list = map_data.get("plumes", [])

            # Build new plumes dict keyed by name
            old_plumes = self.plumes
            self.plumes = {}
            for p in plume_list:
                name = p.get("name")
                if name:
                    self.plumes[name] = self._plume_to_params(p)

            self.get_logger().info(
                f"Plumes loaded from map: {list(self.plumes.keys())}"
            ) # NOT_ESSENTIAL_PRINT

            # Replan if the active target plume changed during a survey
            if self.target_plume_name and self.target_plume_name in self.plumes:
                old_params = old_plumes.get(self.target_plume_name)
                new_params = self.plumes[self.target_plume_name]
                if old_params is not None and old_params != new_params:
                    if self.is_action_running and (self._active_move_goal_handle is not None or len(self.waypoints) > 0):
                        self.get_logger().info(
                            f"Plume '{self.target_plume_name}' changed in map update. Starting replanning."
                        ) # NOT_ESSENTIAL_PRINT
                        self.replan_active_path()
        except Exception as e:
            self.get_logger().error(f"Failed to parse map for plume data: {e}")

    def replan_active_path(self):
        if not self.is_action_running:
            return

        if self._replan_in_progress:
            self._replan_queued = True
            return

        self._replan_in_progress = True
        self._canceling_for_replan = True
        self._replan_queued = False

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
                waypoints = waypoints[16:]
                if len(waypoints) <= 0:
                    self.get_logger().error(
                        'Replanned path has no waypoints after skipping the first 10.'
                    )
                    self._replan_in_progress = False
                    return

            self.waypoints = waypoints  # Assuming `waypoints` is part of the response
            self.current_waypoint_index = 0  # Reset index when waypoints are received
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
