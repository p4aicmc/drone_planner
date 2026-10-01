#!/usr/bin/env python3
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from harpia_msgs.srv import StrInOut


class CommandInterface(Node):
    def __init__(self):
        super().__init__('command_interface')
        self.callback_group = ReentrantCallbackGroup()
        self.mission_command_client = self.create_client(
            StrInOut,
            'mission_controller/command',
            callback_group=self.callback_group,
        )
        self.action_command_client = self.create_client(
            StrInOut,
            'action_planner/command',
            callback_group=self.callback_group,
        )
        self.command_service = self.create_service(
            StrInOut,
            'drone_planner/command',
            self.command_callback,
            callback_group=self.callback_group,
        )

    def command_callback(self, request, response):
        parts = request.message.strip().split()
        if not parts:
            response.success = False
            response.message = 'Empty command'
            return response

        command = parts[0].lower().replace('-', '_')

        if command in ['run', 'cancel_action']:
            return self.forward_to_action_planner(request, response)

        if command in ['start_mission', 'cancel_mission']:
            if command == 'start_mission':
                can_start = self.call_service(
                    self.action_command_client,
                    'action planner command service',
                    'can_start_plan',
                    timeout_sec=5.0,
                )
                if can_start is None:
                    response.success = False
                    response.message = 'Action planner command service is not available'
                    return response

                if not can_start.success:
                    response.success = False
                    response.message = can_start.message
                    return response

            return self.forward_to_mission_controller(request, response)

        response.success = False
        response.message = f"Unknown command '{request.message}'"
        return response

    def forward_to_mission_controller(self, request, response):
        mission_response = self.call_service(
            self.mission_command_client,
            'mission controller command service',
            request.message,
            timeout_sec=30.0,
        )

        if mission_response is None:
            response.success = False
            response.message = 'Mission controller command service is not available'
            return response

        response.success = mission_response.success
        response.message = mission_response.message
        return response

    def forward_to_action_planner(self, request, response):
        action_response = self.call_service(
            self.action_command_client,
            'action planner command service',
            request.message,
            timeout_sec=30.0,
        )

        if action_response is None:
            response.success = False
            response.message = 'Action planner command service is not available'
            return response

        response.success = action_response.success
        response.message = action_response.message
        return response

    def call_service(self, client, service_description, message, timeout_sec):
        if not client.wait_for_service(timeout_sec=2.0):
            return None

        service_request = StrInOut.Request()
        service_request.message = message

        done = threading.Event()
        future = client.call_async(service_request)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=timeout_sec):
            result = StrInOut.Response()
            result.success = False
            result.message = f'{service_description.capitalize()} timed out'
            return result

        try:
            service_response = future.result()
        except Exception as error:
            result = StrInOut.Response()
            result.success = False
            result.message = f'{service_description.capitalize()} failed: {error}'
            return result

        if service_response is None:
            result = StrInOut.Response()
            result.success = False
            result.message = f'{service_description.capitalize()} returned no response'
            return result

        return service_response


def main(args=None):
    rclpy.init(args=args)
    node = CommandInterface()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
