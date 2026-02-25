#!/usr/bin/env python3

import rclpy
import json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String


class PlumeUpdater(Node):

    def __init__(self):
        super().__init__('plume_updater')

        self.plume_publisher = self.create_publisher(
            String, 'plume_update', 10
        )

        self.new_plume = {
            "focus": [0.0, 0.0],             # (x, y) origin point of the plume
            "plume_lenth": 0.0,               # length of the plume in meters
            "plume_angle": 0.0,               # spread angle of the plume in degrees
            "plume_direction": [0.0, 0.0],    # (x, y) direction vector of the plume
        }

        self.startup_delay = 60.0             # wait 1 minute before publishing

        # One-shot timer to wait before starting to publish
        self.startup_timer = self.create_timer(self.startup_delay, self.start_publishing)
        self.publish_timer = None

    def start_publishing(self):
        self.destroy_timer(self.startup_timer)
        self.get_logger().info('Startup delay elapsed, beginning plume publishing.')

    def send_plume_update(self):
        msg = String()
        msg.data = json.dumps(self.new_plume)

        self.plume_publisher.publish(msg)
        self.get_logger().info(f'Plume published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = PlumeUpdater()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
