#!/usr/bin/env python3
"""Generic stub action executor. Launch it once per PDDL action, renaming the node:

    Node(package='action_planner', executable='timed_action_node.py', name='take_image',
         parameters=[{'n_iterations': 5}])
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor

from action_planner.timed_action import TimedActionNode


def main(args=None):
    rclpy.init(args=args)
    node = TimedActionNode('timed_action')
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
