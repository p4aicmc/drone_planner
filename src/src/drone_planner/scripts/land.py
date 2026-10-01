#!/usr/bin/env python3

import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

package_share_path = get_package_share_directory("drone_planner")
scripts_path = os.path.join(package_share_path, 'scripts')
sys.path.append(scripts_path)

from drone_command_action_base import DroneCommandActionNode


def main(args=None):
    rclpy.init(args=args)
    node = DroneCommandActionNode('land')
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
