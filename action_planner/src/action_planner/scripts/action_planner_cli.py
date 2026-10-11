#!/usr/bin/env python3
"""Interactive operator CLI for the action_planner framework.

Run it with the workspace sourced:  ros2 run action_planner action_planner_cli.py [command...]

Commands are forwarded to command_interface ('command_interface/command'):
  start_mission | cancel_mission | run <action> [args...] | cancel_action
While the interactive prompt is open the CLI serves 'action_planner_cli/presence', which
tells mission_controller to wait for start_mission instead of auto-starting.
Extra tab-completion entries for your actions: ACTION_PLANNER_CLI_ACTIONS="go_to,take_off".
"""
import os
import sys
import threading

COMMANDS = [
    'start_mission',
    'cancel_mission',
    'cancel_action',
    'run',
    'exit',
    'quit',
]
COMMANDS += [
    f'run {action.strip()}'
    for action in os.environ.get('ACTION_PLANNER_CLI_ACTIONS', '').split(',')
    if action.strip()
]


try:
    import readline
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from action_planner_msgs.srv import StrInOut
    from std_srvs.srv import Trigger
except ImportError as error:
    print(f'Failed to import ROS dependencies: {error}', file=sys.stderr)
    print('Source a workspace where action_planner is built, then run this script again.', file=sys.stderr)
    sys.exit(1)


class CommandClient(Node):
    def __init__(self):
        super().__init__('action_planner_cli')
        self.client = self.create_client(StrInOut, 'command_interface/command')
        self.presence_service = None
        self.background_executor = None
        self.background_executor_thread = None
        self.use_background_executor = False

    def start_interactive_services(self):
        self.presence_service = self.create_service(
            Trigger,
            'action_planner_cli/presence',
            self.presence_callback,
        )
        self.background_executor = MultiThreadedExecutor(num_threads=2)
        self.background_executor.add_node(self)
        self.background_executor_thread = threading.Thread(
            target=self.background_executor.spin,
            daemon=True,
        )
        self.background_executor_thread.start()
        self.use_background_executor = True

    def stop_interactive_services(self):
        self.use_background_executor = False
        if self.background_executor is not None:
            self.background_executor.shutdown()
        if self.background_executor_thread is not None:
            self.background_executor_thread.join(timeout=2.0)

    def presence_callback(self, request, response):
        response.success = True
        response.message = 'action_planner CLI is running'
        return response

    def send_command(self, command):
        if not self.client.wait_for_service(timeout_sec=2.0):
            return False, 'Command interface service is not available'

        request = StrInOut.Request()
        request.message = command
        future = self.client.call_async(request)

        if self.use_background_executor:
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(timeout=30.0):
                return False, 'Command timed out'
        else:
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done():
            return False, 'Command timed out'

        try:
            response = future.result()
        except Exception as error:
            return False, f'Command failed: {error}'

        if response is None:
            return False, 'Command returned no response'

        return response.success, response.message


def configure_readline():
    matches = []

    def complete(text, state):
        nonlocal matches
        if state == 0:
            matches = [command for command in COMMANDS if command.startswith(text)]
            matches = [match + ' ' if match.startswith('run ') else match for match in matches]
        try:
            return matches[state]
        except IndexError:
            return None

    readline.set_completer(complete)
    readline.set_completer_delims('')
    readline.parse_and_bind('tab: complete')


def print_response(success, message):
    status = 'OK' if success else 'ERROR'
    print(f'{status}: {message}')


def remember_command(command):
    history_length = readline.get_current_history_length()
    if history_length > 0 and readline.get_history_item(history_length) == command:
        return

    readline.add_history(command)


def interactive_loop(command_client):
    command_client.start_interactive_services()
    configure_readline()
    print('action_planner CLI. Type commands like start_mission, cancel_mission or run <action> [args].')
    print('Type exit or quit to close.')

    try:
        while True:
            try:
                command = input('action_planner> ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return

            if not command:
                continue

            if command.lower() in ['exit', 'quit']:
                return

            remember_command(command)
            print_response(*command_client.send_command(command))
    finally:
        command_client.stop_interactive_services()


def main():
    rclpy.init()
    command_client = CommandClient()

    try:
        if len(sys.argv) > 1:
            command = ' '.join(sys.argv[1:])
            print_response(*command_client.send_command(command))
        else:
            interactive_loop(command_client)
    finally:
        try:
            command_client.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
