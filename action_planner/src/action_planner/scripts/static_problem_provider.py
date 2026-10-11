#!/usr/bin/env python3
"""Minimal problem provider: serves a PDDL problem description read from a JSON file.

This is the smallest implementation of the problem-provider contract that
``mission_controller`` consumes. A real robot replaces it with a node that
derives the problem from its own world model (see ``problem_generator`` in the
drone system).

Contract:
  - service ``problem_generator/get_problem`` (std_srvs/Trigger): ``message`` is a
    JSON object ``{"instances": [...], "predicates": [...], "functions": [...], "goals": [...]}``
    where every entry is a space-separated PDDL string, e.g. ``"r1 region"``,
    ``"at r1"``, ``"(distance r1 r2) 10.0"``.
  - topic ``problem_generator/problem_update`` (std_msgs/String, optional): a JSON
    list of ``{"type": ..., "values": [...]}`` update commands understood by
    ``action_planner/update_parameters``.
"""
import json

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_srvs.srv import Trigger


class StaticProblemProvider(LifecycleNode):
    def __init__(self):
        super().__init__('problem_generator')
        self.declare_parameter('problem_file', '')
        self.problem = None
        self.srv = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        problem_file = self.get_parameter('problem_file').value
        try:
            with open(problem_file) as f:
                self.problem = json.load(f)
        except Exception as error:
            self.get_logger().error(f"Could not read problem file '{problem_file}': {error}")
            return TransitionCallbackReturn.ERROR

        for key in ('instances', 'predicates', 'functions', 'goals'):
            self.problem.setdefault(key, [])
        self.get_logger().info(f"Loaded problem from '{problem_file}'")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.srv = self.create_service(Trigger, 'problem_generator/get_problem', self.get_problem_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self.srv is not None:
            self.destroy_service(self.srv)
            self.srv = None
        return super().on_deactivate(state)

    def get_problem_callback(self, request, response):
        response.success = True
        response.message = json.dumps(self.problem)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StaticProblemProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
