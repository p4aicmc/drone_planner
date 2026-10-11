#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

from drone_planner.drone_command_action_base import DroneCommandActionNode


def main(args=None):
    rclpy.init(args=args)
    node = DroneCommandActionNode('take_off')
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
