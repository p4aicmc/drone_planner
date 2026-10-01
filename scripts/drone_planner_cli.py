#!/usr/bin/env python3
import os
import shlex
import sys
import threading

COMMANDS = [
    'start_mission',
    'cancel_mission',
    'cancel_action',
    'run',
    'run go_to',
    'run pulverize_region',
    'run recharge_battery',
    'run recharge_input',
    'run take_off',
    'run land',
    'exit',
    'quit',
]


def bootstrap_ros_environment():
    if os.environ.get('DRONE_PLANNER_CLI_BOOTSTRAPPED') == '1':
        return

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_dir = os.path.dirname(script_dir)
    default_workspace = '/home/drone_planner'
    if not os.path.isdir(default_workspace):
        source_workspace = os.path.join(repo_dir, 'src')
        default_workspace = (
            source_workspace if os.path.isdir(source_workspace) else repo_dir
        )
    workspace_dir = os.environ.get('DRONE_PLANNER_WORKSPACE', default_workspace)
    setup_files = []

    if os.path.exists('/opt/ros/jazzy/setup.bash'):
        setup_files.append('/opt/ros/jazzy/setup.bash')
    elif os.path.exists('/opt/ros/humble/setup.bash'):
        setup_files.append('/opt/ros/humble/setup.bash')

    # harpia_msgs is part of this colcon workspace. Loading a separate nested
    # harpia_msgs install can mix generated libraries from different builds.
    setup_files.append(os.path.join(workspace_dir, 'install/setup.bash'))

    existing_setup_files = [path for path in setup_files if os.path.exists(path)]
    if not existing_setup_files:
        return

    source_commands = ' && '.join(
        f'source {shlex.quote(path)}' for path in existing_setup_files
    )
    argv = ' '.join(shlex.quote(arg) for arg in sys.argv)
    command = (
        f'{source_commands} && '
        'export DRONE_PLANNER_CLI_BOOTSTRAPPED=1 && '
        f'exec {shlex.quote(sys.executable)} {argv}'
    )

    os.execv('/bin/bash', ['/bin/bash', '-lc', command])


bootstrap_ros_environment()

try:
    import readline
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from harpia_msgs.srv import StrInOut
    from std_srvs.srv import Trigger
except ImportError as error:
    print(f'Failed to import ROS dependencies: {error}', file=sys.stderr)
    print('Build the drone_planner workspace first, then run this script again.', file=sys.stderr)
    sys.exit(1)


class CommandClient(Node):
    def __init__(self):
        super().__init__('drone_planner_cli')
        self.client = self.create_client(StrInOut, 'drone_planner/command')
        self.presence_service = None
        self.background_executor = None
        self.background_executor_thread = None
        self.use_background_executor = False

    def start_interactive_services(self):
        self.presence_service = self.create_service(
            Trigger,
            'drone_planner_cli/presence',
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
        response.message = 'Drone Planner CLI is running'
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
    print('Drone Planner CLI. Type commands like start_mission or cancel_mission.')
    print('Type exit or quit to close.')

    try:
        while True:
            try:
                command = input('drone_planner> ').strip()
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
