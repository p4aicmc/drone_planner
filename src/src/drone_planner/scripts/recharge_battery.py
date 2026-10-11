#!/usr/bin/env python3
"""Placeholder for the `recharge_battery` PDDL action: succeeds after a few ticks.
Replace TimedActionNode with a subclass of action_planner.ActionExecutorBase
that drives the real recharge procedure."""
import rclpy
from rclpy.executors import MultiThreadedExecutor

from action_planner.timed_action import TimedActionNode


def main(args=None):
    rclpy.init(args=args)
    node = TimedActionNode('recharge_battery', n_iterations=10)
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
