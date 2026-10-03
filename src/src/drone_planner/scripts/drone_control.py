#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from mavros_msgs.msg import ExtendedState, State as MavrosState
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from harpia_msgs.msg import DroneSetpoint
from harpia_msgs.srv import DroneCommand, ReleaseControl, RequestControl


class DroneControl(LifecycleNode):
    def __init__(self):
        super().__init__('drone_control')

        self.declare_parameter('takeoff_altitude', 3.0)
        self.declare_parameter('takeoff_altitude_tolerance', 0.35)
        self.declare_parameter('lease_timeout_sec', 0.5)
        self.declare_parameter('setpoint_rate_hz', 20.0)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring drone_control node...')

        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.takeoff_altitude_tolerance = self.get_parameter('takeoff_altitude_tolerance').value
        self.lease_timeout_sec = self.get_parameter('lease_timeout_sec').value
        self.setpoint_period = 1.0 / self.get_parameter('setpoint_rate_hz').value

        self.active_lease_owner = None
        self.active_operation = 'idle'
        self.last_external_setpoint_time = None
        self.latest_external_setpoint = None
        self.current_pose = None
        self.current_state = MavrosState()
        self.current_extended_state = None
        self.hold_setpoint = PoseStamped()
        self.land_command_sent = False
        self.command_group = ReentrantCallbackGroup()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.mavros_setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        self.local_pose_pub = self.create_publisher(PoseStamped, '/drone_control/local_pose', 10)
        self.global_position_pub = self.create_publisher(NavSatFix, '/drone_control/global_position', 10)
        self.state_pub = self.create_publisher(String, '/drone_control/state', 10)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        self.state_sub = self.create_subscription(
            MavrosState,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )
        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self.extended_state_callback,
            qos_profile
        )
        self.global_position_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.global_position_callback,
            qos_profile
        )
        self.external_setpoint_sub = self.create_subscription(
            DroneSetpoint,
            '/drone_control/setpoint/local',
            self.external_setpoint_callback,
            10
        )

        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming',
            callback_group=self.command_group
        )
        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode',
            callback_group=self.command_group
        )
        self.land_client = self.create_client(
            CommandTOL,
            '/mavros/cmd/land',
            callback_group=self.command_group
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating drone_control node...')

        if not self.arming_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/mavros/cmd/arming service not available')
            return TransitionCallbackReturn.ERROR
        if not self.set_mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/mavros/set_mode service not available')
            return TransitionCallbackReturn.ERROR
        if not self.land_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/mavros/cmd/land service not available')
            return TransitionCallbackReturn.ERROR

        self.request_control_srv = self.create_service(
            RequestControl,
            '/drone_control/request_control',
            self.request_control_callback,
            callback_group=self.command_group
        )
        self.release_control_srv = self.create_service(
            ReleaseControl,
            '/drone_control/release_control',
            self.release_control_callback,
            callback_group=self.command_group
        )
        self.command_srv = self.create_service(
            DroneCommand,
            '/drone_control/command',
            self.command_callback,
            callback_group=self.command_group
        )
        self.setpoint_timer = self.create_timer(self.setpoint_period, self.setpoint_timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def pose_callback(self, msg):
        self.current_pose = msg
        self.local_pose_pub.publish(msg)

        if self.active_operation == 'idle' and self.active_lease_owner is None:
            self.hold_setpoint = self.copy_pose(msg)

    def state_callback(self, msg):
        self.current_state = msg

    def extended_state_callback(self, msg):
        self.current_extended_state = msg

    def global_position_callback(self, msg):
        self.global_position_pub.publish(msg)

    def external_setpoint_callback(self, msg):
        if msg.owner != self.active_lease_owner:
            self.get_logger().warn(
                f"Ignoring setpoint from '{msg.owner}', active lease is '{self.active_lease_owner}'"
            )
            return

        self.latest_external_setpoint = msg.setpoint
        self.last_external_setpoint_time = time.monotonic()

    def request_control_callback(self, request, response):
        if self.active_lease_owner is not None and self.active_lease_owner != request.owner:
            response.success = False
            response.message = f"control lease already held by {self.active_lease_owner}"
            return response

        if self.active_operation not in ['idle', 'hold']:
            response.success = False
            response.message = f"drone_control operation '{self.active_operation}' is active"
            return response

        if not self.is_flying():
            response.success = False
            response.message = 'drone is not flying'
            return response

        if not self.prepare_for_external_control(response):
            return response

        self.active_lease_owner = request.owner
        self.active_operation = 'external'
        self.latest_external_setpoint = None
        self.last_external_setpoint_time = time.monotonic()

        response.success = True
        response.message = f"control lease granted to {request.owner}"
        return response

    def release_control_callback(self, request, response):
        if self.active_lease_owner != request.owner:
            response.success = False
            response.message = f"cannot release lease owned by {self.active_lease_owner}"
            return response

        self.active_lease_owner = None
        self.active_operation = 'hold'
        self.latest_external_setpoint = None
        self.last_external_setpoint_time = None
        self.update_hold_setpoint()

        response.success = True
        response.message = 'control lease released'
        return response

    def command_callback(self, request, response):
        command = request.command

        if command == 'take_off':
            altitude = request.altitude if request.altitude > 0 else self.takeoff_altitude
            return self.handle_takeoff(altitude, response)
        if command == 'land':
            return self.handle_land(response)

        response.accepted = False
        response.finished = True
        response.success = False
        response.message = f"unknown command '{command}'"
        return response

    def handle_takeoff(self, altitude, response):
        if self.active_lease_owner is not None:
            return self.command_response(response, False, True, False, 'cannot take off while control lease is active')

        if self.current_pose is None:
            return self.command_response(response, False, True, False, 'current pose is unknown')

        if self.current_pose.pose.position.z >= altitude - self.takeoff_altitude_tolerance:
            self.active_operation = 'hold'
            self.update_hold_setpoint()
            return self.command_response(response, True, True, True, 'takeoff altitude already reached')

        self.hold_setpoint = self.copy_pose(self.current_pose)
        self.hold_setpoint.pose.position.z = altitude
        self.active_operation = 'takeoff'
        self.land_command_sent = False

        if not self.ensure_offboard():
            self.active_operation = 'idle'
            return self.command_response(response, False, True, False, 'failed to enter OFFBOARD mode')

        if not self.current_state.armed and not self.set_armed(True):
            self.active_operation = 'idle'
            return self.command_response(response, False, True, False, 'arm failed before takeoff')

        return self.command_response(response, True, False, False, 'takeoff in progress')

    def handle_land(self, response):
        if self.active_lease_owner is not None:
            self.get_logger().warn(f"Revoking lease from {self.active_lease_owner} for landing")
            self.active_lease_owner = None

        if self.current_extended_state is None:
            return self.command_response(response, False, True, False, 'cannot land until MAVROS extended state is known')

        if self.is_landed():
            if self.current_state.armed:
                if not self.set_armed(False):
                    return self.command_response(response, False, True, False, 'landed but disarm failed')
            self.clear_control_state()
            return self.command_response(response, True, True, True, 'landed and disarmed')

        if not self.land_command_sent:
            if not self.send_land_command():
                self.active_operation = 'idle'
                self.land_command_sent = False
                return self.command_response(response, False, True, False, 'land command rejected')
            self.land_command_sent = True

        self.active_operation = 'land'
        return self.command_response(response, True, False, False, 'landing in progress')

    def command_response(self, response, accepted, finished, success, message):
        response.accepted = accepted
        response.finished = finished
        response.success = success
        response.message = message
        return response

    def prepare_for_external_control(self, response):
        if self.current_pose is None:
            response.success = False
            response.message = 'current pose is unknown'
            return False

        self.update_hold_setpoint()

        if not self.ensure_offboard():
            response.success = False
            response.message = 'failed to enter OFFBOARD mode'
            return False

        if not self.current_state.armed and not self.set_armed(True):
            response.success = False
            response.message = 'arm failed before granting control lease'
            return False

        return True

    def setpoint_timer_callback(self):
        self.publish_state()

        # if the drone is being controlled by another node (lease owner)
        if self.active_lease_owner is not None:
            if self.last_external_setpoint_time is None:
                return

            # check if the lease owner stoped publishing new waypoints
            if time.monotonic() - self.last_external_setpoint_time > self.lease_timeout_sec:
                self.get_logger().error(f"Lease holder '{self.active_lease_owner}' timed out, switching to hold")
                self.active_lease_owner = None
                self.active_operation = 'hold'
                self.latest_external_setpoint = None
                self.last_external_setpoint_time = None
                self.update_hold_setpoint()
                self.publish_hold_setpoint()
                return

            # publish the waypoint
            if self.latest_external_setpoint is not None:
                self.mavros_setpoint_pub.publish(self.latest_external_setpoint)
            return

        # if in taking off
        if self.active_operation == 'takeoff':
            # publish the hold waypoint
            self.publish_hold_setpoint()
            # check if hit the target height
            if self.current_pose and self.current_pose.pose.position.z >= self.hold_setpoint.pose.position.z - self.takeoff_altitude_tolerance:
                self.active_operation = 'hold'
                self.update_hold_setpoint()
            return

        # if is landing
        if self.active_operation == 'land':
            # and landed completed
            if self.is_landed():
                # disarm if armed
                if self.current_state.armed:
                    if not self.set_armed(False):
                        self.get_logger().error('Landed but disarm failed')
                        return
                # set as idle
                self.clear_control_state()
            return

        # if is holding position
        if self.active_operation == 'hold':
            # send the waypoint
            self.publish_hold_setpoint()

    def publish_state(self):
        msg = String()
        msg.data = json.dumps({
            'connected': bool(self.current_state.connected),
            'armed': bool(self.current_state.armed),
            'mode': self.current_state.mode,
            'flying': self.is_flying(),
            'landed': self.is_landed(),
            'landing': self.is_landing(),
            'landed_state': self.landed_state_name(),
            'active_lease_owner': self.active_lease_owner,
            'active_operation': self.active_operation,
        })
        self.state_pub.publish(msg)

    def ensure_offboard(self):
        if self.current_state.mode == 'OFFBOARD':
            return True

        start = time.monotonic()
        while time.monotonic() - start < 1.0:
            self.publish_hold_setpoint()
            time.sleep(self.setpoint_period)

        request = SetMode.Request()
        request.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(request)
        return self.wait_for_future_result(future, lambda result: result is not None and result.mode_sent)
    

    def ensure_auto_hold(self):
        if self.current_state.mode == 'AUTO.LOITER':
            return True

        request = SetMode.Request()
        request.custom_mode = 'AUTO.LOITER'
        future = self.set_mode_client.call_async(request)
        return self.wait_for_future_result(future, lambda result: result is not None and result.mode_sent)

    def set_armed(self, armed):
        request = CommandBool.Request()
        request.value = armed
        future = self.arming_client.call_async(request)
        return self.wait_for_future_result(future, lambda result: result is not None and result.success)

    def send_land_command(self):

        if not self.ensure_auto_hold():
            self.get_logger().error("Failed to change to auto.loiter mode")
            return False

        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = 0.0
        future = self.land_client.call_async(request)
        return self.wait_for_future_result(future, lambda result: result is not None and result.success)

    def clear_control_state(self):
        self.active_lease_owner = None
        self.active_operation = 'idle'
        self.latest_external_setpoint = None
        self.last_external_setpoint_time = None
        self.land_command_sent = False

    def wait_for_future_result(self, future, is_success, timeout_sec=5.0):
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > timeout_sec:
                return False
            time.sleep(0.05)

        if not future.done():
            return False

        try:
            return bool(is_success(future.result()))
        except Exception as e:
            self.get_logger().error(f'MAVROS service call failed: {e}')
            return False

    def publish_hold_setpoint(self):
        self.mavros_setpoint_pub.publish(self.hold_setpoint)

    def update_hold_setpoint(self):
        if self.current_pose is not None:
            self.hold_setpoint = self.copy_pose(self.current_pose)
            self.hold_setpoint.pose.orientation.w = 1.0

    def is_flying(self):
        if self.current_extended_state is None:
            return False
        return self.current_extended_state.landed_state in [
            ExtendedState.LANDED_STATE_IN_AIR,
            ExtendedState.LANDED_STATE_TAKEOFF,
            ExtendedState.LANDED_STATE_LANDING,
        ]

    def is_landed(self):
        return (
            self.current_extended_state is not None
            and self.current_extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        )

    def is_landing(self):
        return (
            self.current_extended_state is not None
            and self.current_extended_state.landed_state == ExtendedState.LANDED_STATE_LANDING
        )

    def landed_state_name(self):
        if self.current_extended_state is None:
            return 'UNKNOWN'

        names = {
            ExtendedState.LANDED_STATE_UNDEFINED: 'UNDEFINED',
            ExtendedState.LANDED_STATE_ON_GROUND: 'ON_GROUND',
            ExtendedState.LANDED_STATE_IN_AIR: 'IN_AIR',
            ExtendedState.LANDED_STATE_TAKEOFF: 'TAKEOFF',
            ExtendedState.LANDED_STATE_LANDING: 'LANDING',
        }
        return names.get(self.current_extended_state.landed_state, 'UNKNOWN')

    def copy_pose(self, pose):
        new_pose = PoseStamped()
        new_pose.header = pose.header
        new_pose.pose.position.x = pose.pose.position.x
        new_pose.pose.position.y = pose.pose.position.y
        new_pose.pose.position.z = pose.pose.position.z
        new_pose.pose.orientation.x = pose.pose.orientation.x
        new_pose.pose.orientation.y = pose.pose.orientation.y
        new_pose.pose.orientation.z = pose.pose.orientation.z
        new_pose.pose.orientation.w = pose.pose.orientation.w
        return new_pose


def main(args=None):
    rclpy.init(args=args)
    node = DroneControl()
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
