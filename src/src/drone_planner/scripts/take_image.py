#!/usr/bin/env python3

"""Fly a closed square centered on the requested image region."""

import json
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.append(os.path.join(get_package_share_directory("drone_planner"), "scripts"))
from action_executor_base import ActionExecutorBase
from photo_square import square_waypoints


class TakeImage(ActionExecutorBase):
    def __init__(self):
        super().__init__("take_image")
        self.declare_parameter("photo_altitude", 10.0)
        self.declare_parameter("square_margin", 0.8)

    def on_configure_extension(self):
        self.map_data = None
        self.home = None
        self.active = False
        self.started = False
        self.failed = False
        self.waypoints = []
        self.waypoint_index = 0
        self.move_goal_handle = None
        self.goal_generation = 0

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_subscription = self.create_subscription(
            String, "data_server/map", self.map_callback, map_qos)
        self.home_group = MutuallyExclusiveCallbackGroup()
        self.move_group = MutuallyExclusiveCallbackGroup()
        self.home_client = self.create_client(
            Trigger, "data_server/home_position",
            callback_group=self.home_group)
        self.move_client = ActionClient(
            self, MoveTo, "/drone/move_to_waypoint",
            callback_group=self.move_group)
        self.request_home()
        return TransitionCallbackReturn.SUCCESS

    def request_home(self):
        if self.home_client.wait_for_service(timeout_sec=2.0):
            self.home_client.call_async(Trigger.Request()).add_done_callback(
                self.home_callback)
        else:
            self.get_logger().warning("home position service unavailable")

    def home_callback(self, future):
        try:
            response = future.result()
            lat, lon = response.message.split()[:2]
            self.home = (float(lat), float(lon))
        except Exception as exc:
            self.get_logger().error(f"Could not read home position: {exc}")

    def map_callback(self, message):
        try:
            self.map_data = json.loads(message.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"Invalid map data: {exc}")

    def new_goal(self, goal_request):
        if self.active or len(goal_request.parameters) != 1:
            return False
        if self.home is None:
            self.request_home()
        if self.home is None or self.map_data is None:
            self.get_logger().error("Map or home position is not ready")
            return False

        name = goal_request.parameters[0]
        region = next((roi for roi in self.map_data.get("roi", [])
                       if roi.get("name") == name), None)
        if region is None:
            self.get_logger().error(f"Unknown image region: {name}")
            return False

        try:
            points = square_waypoints(
                region, self.map_data.get("nfz", []), *self.home,
                margin=float(self.get_parameter("square_margin").value))
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Cannot fly square in {name}: {exc}")
            return False

        altitude = float(self.get_parameter("photo_altitude").value)
        self.waypoints = []
        for x, y in points:
            waypoint = PoseStamped()
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.position.z = altitude
            waypoint.pose.orientation.w = 1.0
            self.waypoints.append(waypoint)

        self.goal_generation += 1
        self.waypoint_index = 0
        self.move_goal_handle = None
        self.started = False
        self.failed = False
        self.active = True
        self._goal_success = True
        self.get_logger().info(f"Flying square over {name} with {len(points)} waypoints")
        return True

    def execute_goal(self, goal_handle):
        if self.failed:
            self.active = False
            self._goal_success = False
            return True, self.waypoint_index / len(self.waypoints)
        if self.waypoint_index == len(self.waypoints):
            self.active = False
            return True, 1.0
        if not self.started:
            self.started = True
            self.send_waypoint(self.goal_generation)
        return False, self.waypoint_index / len(self.waypoints)

    def send_waypoint(self, generation):
        if generation != self.goal_generation or not self.active:
            return
        if not self.move_client.wait_for_server(timeout_sec=1.0):
            self.fail("Waypoint action server unavailable")
            return
        request = MoveTo.Goal()
        request.destination = self.waypoints[self.waypoint_index]
        try:
            future = self.move_client.send_goal_async(request)
            future.add_done_callback(
                lambda result: self.waypoint_response(result, generation))
        except Exception as exc:
            self.fail(f"Could not send waypoint: {exc}")

    def waypoint_response(self, future, generation):
        try:
            handle = future.result()
            if generation != self.goal_generation or not self.active:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.fail("Waypoint goal rejected")
                return
            self.move_goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda result: self.waypoint_result(result, generation))
        except Exception as exc:
            if generation == self.goal_generation and self.active:
                self.fail(f"Waypoint goal failed: {exc}")

    def waypoint_result(self, future, generation):
        if generation != self.goal_generation or not self.active:
            return
        try:
            result = future.result().result
            if not result.success:
                self.fail("Waypoint was not reached")
                return
            self.move_goal_handle = None
            self.waypoint_index += 1
            if self.waypoint_index < len(self.waypoints):
                self.send_waypoint(generation)
        except Exception as exc:
            self.fail(f"Waypoint result failed: {exc}")

    def fail(self, message):
        self.get_logger().error(message)
        self.failed = True

    def cancel_goal(self, goal_handle):
        self.goal_generation += 1
        self.active = False
        if self.move_goal_handle is not None:
            self.move_goal_handle.cancel_goal_async()
        self.move_goal_handle = None

    def cancel_goal_request(self, goal_handle):
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TakeImage()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down take_image")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
