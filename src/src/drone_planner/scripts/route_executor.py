#!/usr/bin/env python3

import time
from math import sqrt

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from harpia_msgs.msg import DroneSetpoint
from harpia_msgs.srv import ReleaseControl, RequestControl


class RouteExecutor(Node):
    def __init__(self):
        super().__init__('route_executor')
        self.get_logger().info('RouteExecutor node initializing')

        self.owner = 'route_executor'
        self.waypoint = PoseStamped()
        self.current_position = PoseStamped()
        self.current_position_received = False
        self._cancel_requested = False
        self._active_goal = False
        self._timer = None
        self._release_control_future = None

        self.setpoint_pub = self.create_publisher(
            DroneSetpoint,
            '/drone_control/setpoint/local',
            10
        )
        self.request_control_client = self.create_client(
            RequestControl,
            '/drone_control/request_control'
        )
        self.release_control_client = self.create_client(
            ReleaseControl,
            '/drone_control/release_control'
        )

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/drone_control/local_pose',
            self.position_callback,
            qos_profile
        )

        self._action_server = ActionServer(
            self,
            MoveTo,
            '/drone/move_to_waypoint',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        if self._active_goal:
            self.get_logger().warn('Rejecting waypoint goal because one is already active')
            return GoalResponse.REJECT

        self.waypoint = goal_request.destination
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelling waypoint goal')
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        feedback_msg = MoveTo.Feedback()
        result = MoveTo.Result()
        self._cancel_requested = False
        self._active_goal = True
        goal_finished = False
        goal_future = rclpy.task.Future()

        lease_success, lease_message = await self.request_control()
        if not lease_success:
            self.get_logger().error(f'Control lease denied: {lease_message}')
            result.success = False
            goal_handle.abort()
            self._active_goal = False
            return result

        def timer_callback():
            nonlocal goal_finished

            if goal_finished or goal_future.done():
                return

            feedback_msg.distance = float(self.get_distance(self.waypoint))
            goal_handle.publish_feedback(feedback_msg)

            if self.has_reached_waypoint(self.waypoint):
                goal_finished = True
                self.finish_timer()
                result.success = True
                self.release_control()
                if goal_handle.is_active:
                    goal_handle.succeed()
                if not goal_future.done():
                    goal_future.set_result(result)

            elif self._cancel_requested:
                goal_finished = True
                self.finish_timer()
                result.success = False
                self.release_control()
                if goal_handle.is_active:
                    goal_handle.canceled()
                if not goal_future.done():
                    goal_future.set_result(result)

            else:
                self.publish_current_setpoint()

        self._timer = self.create_timer(0.05, timer_callback)
        await goal_future

        self._active_goal = False
        return result

    async def request_control(self):
        if not self.request_control_client.service_is_ready():
            self.get_logger().info('Waiting for /drone_control/request_control...')
            if not self.request_control_client.wait_for_service(timeout_sec=5.0):
                return False, 'request_control service not available'

        request = RequestControl.Request()
        request.owner = self.owner
        request.reason = 'move_to_waypoint'
        future = self.request_control_client.call_async(request)
        response = await future

        if response is None:
            return False, 'request_control returned no response'
        return bool(response.success), response.message

    def release_control(self):
        if not self.release_control_client.service_is_ready():
            return

        request = ReleaseControl.Request()
        request.owner = self.owner
        self._release_control_future = self.release_control_client.call_async(request)
        self._release_control_future.add_done_callback(self.release_control_callback)

    def release_control_callback(self, future):
        try:
            response = future.result()
            if response is not None and not response.success:
                self.get_logger().warn(f'Control release failed: {response.message}')
        except Exception as e:
            self.get_logger().warn(f'Control release request failed: {e}')
        finally:
            self._release_control_future = None

    def position_callback(self, msg):
        self.current_position = msg
        self.current_position_received = True

    def publish_current_setpoint(self):
        msg = DroneSetpoint()
        msg.owner = self.owner
        msg.setpoint = self.waypoint
        msg.setpoint.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def has_reached_waypoint(self, waypoint, threshold=1.0):
        if not self.current_position_received:
            return False
        return self.get_distance(waypoint) < threshold

    def get_distance(self, waypoint):
        current_position = self.current_position.pose.position
        return sqrt(
            (waypoint.pose.position.x - current_position.x) ** 2 +
            (waypoint.pose.position.y - current_position.y) ** 2 +
            (waypoint.pose.position.z - current_position.z) ** 2
        )

    def finish_timer(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None


def main(args=None):
    rclpy.init(args=args)
    node = RouteExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
