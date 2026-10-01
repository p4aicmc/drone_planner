#!/usr/bin/env python3

import os

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class DroneMarker(Node):
    def __init__(self):
        super().__init__('drone_marker')

        default_mesh = self._resolve_default_mesh()
        self.declare_parameter('pose_topic', '/drone_control/local_pose')
        self.declare_parameter('marker_topic', '/drone/marker')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('mesh_resource', default_mesh)
        self.declare_parameter('scale', 1.0)

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value

        self.default_frame = self.get_parameter('frame_id').get_parameter_value().string_value
        self.mesh_resource = self.get_parameter('mesh_resource').get_parameter_value().string_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value

        if not self.mesh_resource.startswith('file://'):
            self.get_logger().warn('mesh_resource should start with file:// for RViz mesh loading')
        else:
            mesh_path = self.mesh_resource.replace('file://', '', 1)
            if not os.path.isfile(mesh_path):
                self.get_logger().warn(f'mesh_resource file not found: {mesh_path}')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, qos)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_pose = None
        self.last_pose_warn_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.publish_marker)

    def _resolve_default_mesh(self) -> str:
        candidates = [
            '/home/PX4-Autopilot/Tools/simulation/gz/models/x500_base/meshes/5010Base.dae',
            '/home/PX4-Autopilot/Tools/simulation/gz/models/x500_base/meshes/NXP-HGD-CF.dae',
            '/home/PX4-Autopilot/Tools/simulation/gz/models/quadtailsitter/meshes/body.dae',
        ]

        for candidate in candidates:
            if os.path.isfile(candidate):
                return f'file://{candidate}'

        return 'file:///home/PX4-Autopilot/Tools/simulation/gz/models/x500_base/meshes/5010Base.dae'

    def pose_callback(self, msg: PoseStamped):
        self.last_pose = msg
        self.publish_tf()

    def publish_marker(self):
        if self.last_pose is None:
            now = self.get_clock().now()
            if (now - self.last_pose_warn_time).nanoseconds > 5_000_000_000:
                self.get_logger().warn('Waiting for pose topic to publish...')
                self.last_pose_warn_time = now
            return

        stamp = self.get_clock().now().to_msg()

        mesh_marker = Marker()
        mesh_marker.header.stamp = stamp
        mesh_marker.header.frame_id = self.default_frame
        mesh_marker.ns = 'px4'
        mesh_marker.id = 1
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.action = Marker.ADD
        mesh_marker.pose = self.last_pose.pose
        mesh_marker.scale.x = self.scale
        mesh_marker.scale.y = self.scale
        mesh_marker.scale.z = self.scale
        mesh_marker.color.r = 1.0
        mesh_marker.color.g = 1.0
        mesh_marker.color.b = 1.0
        mesh_marker.color.a = 1.0
        mesh_marker.mesh_resource = self.mesh_resource
        mesh_marker.mesh_use_embedded_materials = True
        self.marker_pub.publish(mesh_marker)

        body_marker = Marker()
        body_marker.header.stamp = stamp
        body_marker.header.frame_id = self.default_frame
        body_marker.ns = 'px4_fallback'
        body_marker.id = 2
        body_marker.type = Marker.CUBE
        body_marker.action = Marker.ADD
        body_marker.pose = self.last_pose.pose
        body_marker.scale.x = 0.6
        body_marker.scale.y = 0.6
        body_marker.scale.z = 0.15
        body_marker.color.r = 0.1
        body_marker.color.g = 0.8
        body_marker.color.b = 0.1
        body_marker.color.a = 0.35
        self.marker_pub.publish(body_marker)

    def publish_tf(self):
        if self.last_pose is None:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.default_frame
        t.child_frame_id = 'drone_base_link'
        t.transform.translation.x = self.last_pose.pose.position.x
        t.transform.translation.y = self.last_pose.pose.position.y
        t.transform.translation.z = self.last_pose.pose.position.z
        t.transform.rotation = self.last_pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = DroneMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
