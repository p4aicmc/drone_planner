#!/usr/bin/env python3

import rclpy
import json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String


class PlumeUpdater(Node):
    UPDATE_SPACING_SECONDS = 50.0
    PLUME_UPDATES = [
        {
            "focus": [0.0, 10.0],          # (x, y) origin point of the plume
            "plume_lenth": 150.0,         # length of the plume in meters
            "plume_angle": 30.0,          # spread angle of the plume in degrees
            "plume_direction": [1.0, 1.0] # (x, y) direction vector of the plume
        },
        {
            "focus": [0.0, 20.0],
            "plume_lenth": 150.0,
            "plume_angle": 30.0,
            "plume_direction": [0.0, 1.0]
        }
    ]

    def __init__(self):
        super().__init__('plume_updater')

        self.plume_publisher = self.create_publisher(
            String, 'plume_update', 10
        )

        self.startup_delay = 50.0

        # One-shot timer to wait before starting to publish
        self.startup_timer = self.create_timer(self.startup_delay, self.start_publishing)
        self.update_timer = None
        self.current_update_index = 0

    def start_publishing(self):
        self.destroy_timer(self.startup_timer)
        self.get_logger().info('Startup delay elapsed, beginning plume publishing.')
        self.send_next_plume_update()
        self.update_timer = self.create_timer(
            self.UPDATE_SPACING_SECONDS,
            self.send_next_plume_update
        )

    def send_next_plume_update(self):
        if self.current_update_index >= len(self.PLUME_UPDATES):
            if self.update_timer is not None:
                self.destroy_timer(self.update_timer)
                self.update_timer = None
            self.get_logger().info('All plume updates published.')
            return

        plume_update = self.PLUME_UPDATES[self.current_update_index]
        msg = String()
        msg.data = json.dumps(plume_update)

        self.plume_publisher.publish(msg)
        self.get_logger().info(
            f'Plume update {self.current_update_index + 1}/{len(self.PLUME_UPDATES)} published: {msg.data}'
        )
        self.current_update_index += 1


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
