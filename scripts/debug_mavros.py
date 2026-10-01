#!/usr/bin/env python3

import json
import os
import queue
import shlex
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk


MODES = [
    'OFFBOARD',
    'POSCTL',
    'ALTCTL',
    'MANUAL',
    'AUTO.LOITER',
    'AUTO.MISSION',
    'AUTO.LAND',
    'AUTO.RTL',
    'STABILIZED',
]


def bootstrap_ros_environment():
    if os.environ.get('DEBUG_MAVROS_BOOTSTRAPPED') == '1':
        return

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_dir = os.path.dirname(script_dir)
    setup_files = []

    if os.path.exists('/opt/ros/jazzy/setup.bash'):
        setup_files.append('/opt/ros/jazzy/setup.bash')
    elif os.path.exists('/opt/ros/humble/setup.bash'):
        setup_files.append('/opt/ros/humble/setup.bash')

    setup_files.extend([
        os.path.join(repo_dir, 'src/drone_planner/src/harpia_msgs/install/setup.bash'),
        os.path.join(repo_dir, 'src/drone_planner/install/setup.bash'),
    ])

    existing_setup_files = [path for path in setup_files if os.path.exists(path)]
    if not existing_setup_files:
        return

    source_commands = ' && '.join(
        f'source {shlex.quote(path)}' for path in existing_setup_files
    )
    argv = ' '.join(shlex.quote(arg) for arg in sys.argv)
    command = (
        f'{source_commands} && '
        'export DEBUG_MAVROS_BOOTSTRAPPED=1 && '
        f'exec {shlex.quote(sys.executable)} {argv}'
    )

    os.execv('/bin/bash', ['/bin/bash', '-lc', command])


bootstrap_ros_environment()

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State as MavrosState
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class MavrosDebugNode(Node):
    def __init__(self, gui_queue, command_queue):
        super().__init__('debug_mavros_gui')
        self.gui_queue = gui_queue
        self.command_queue = command_queue
        self.publish_setpoint = False
        self.setpoint = (0.0, 0.0, 3.0)
        self.last_setpoint_publish = 0.0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_sub = self.create_subscription(
            MavrosState,
            '/mavros/state',
            self.state_callback,
            qos_profile,
        )
        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self.extended_state_callback,
            qos_profile,
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10,
        )
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.command_timer = self.create_timer(0.05, self.process_commands)
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint_callback)

    def state_callback(self, msg):
        data = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'connected': bool(msg.connected),
            'armed': bool(msg.armed),
            'guided': bool(msg.guided),
            'manual_input': bool(msg.manual_input),
            'mode': msg.mode,
            'system_status': int(msg.system_status),
        }
        self.gui_queue.put(('state', json.dumps(data, indent=2)))

    def extended_state_callback(self, msg):
        data = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'vtol_state': int(msg.vtol_state),
            'landed_state': int(msg.landed_state),
            'landed_state_name': self.landed_state_name(msg.landed_state),
        }
        self.gui_queue.put(('extended_state', json.dumps(data, indent=2)))

    def process_commands(self):
        while True:
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                return

            action = command[0]
            if action == 'set_mode':
                self.send_mode(command[1])
            elif action == 'arm':
                self.send_arm(True)
            elif action == 'disarm':
                self.send_arm(False)
            elif action == 'land':
                self.send_land()
            elif action == 'start_setpoint':
                self.setpoint = command[1]
                self.publish_setpoint = True
                self.gui_queue.put(('command_result', f'Started publishing setpoint {self.setpoint}'))
            elif action == 'stop_setpoint':
                self.publish_setpoint = False
                self.gui_queue.put(('command_result', 'Stopped publishing setpoint'))

    def send_mode(self, mode):
        if not self.set_mode_client.service_is_ready():
            self.gui_queue.put(('command_result', 'Waiting for /mavros/set_mode service...'))
            if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
                self.gui_queue.put(('command_result', '/mavros/set_mode service not available'))
                return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(lambda done: self.set_mode_done(mode, done))

    def set_mode_done(self, mode, future):
        try:
            result = future.result()
        except Exception as error:
            self.gui_queue.put(('command_result', f"Set mode '{mode}' failed: {error}"))
            return

        if result is None:
            self.gui_queue.put(('command_result', f"Set mode '{mode}' returned no response"))
            return

        self.gui_queue.put((
            'command_result',
            f"Set mode '{mode}': mode_sent={bool(result.mode_sent)}",
        ))

    def send_arm(self, armed):
        if not self.arming_client.service_is_ready():
            self.gui_queue.put(('command_result', 'Waiting for /mavros/cmd/arming service...'))
            if not self.arming_client.wait_for_service(timeout_sec=2.0):
                self.gui_queue.put(('command_result', '/mavros/cmd/arming service not available'))
                return

        request = CommandBool.Request()
        request.value = armed
        future = self.arming_client.call_async(request)
        future.add_done_callback(lambda done: self.arm_done(armed, done))

    def arm_done(self, armed, future):
        action = 'Arm' if armed else 'Disarm'
        try:
            result = future.result()
        except Exception as error:
            self.gui_queue.put(('command_result', f'{action} failed: {error}'))
            return

        if result is None:
            self.gui_queue.put(('command_result', f'{action} returned no response'))
            return

        self.gui_queue.put((
            'command_result',
            f'{action}: success={bool(result.success)} result={int(result.result)}',
        ))

    def send_land(self):
        if not self.land_client.service_is_ready():
            self.gui_queue.put(('command_result', 'Waiting for /mavros/cmd/land service...'))
            if not self.land_client.wait_for_service(timeout_sec=2.0):
                self.gui_queue.put(('command_result', '/mavros/cmd/land service not available'))
                return

        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = 0.0
        future = self.land_client.call_async(request)
        future.add_done_callback(self.land_done)

    def land_done(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.gui_queue.put(('command_result', f'Land failed: {error}'))
            return

        if result is None:
            self.gui_queue.put(('command_result', 'Land returned no response'))
            return

        self.gui_queue.put((
            'command_result',
            f'Land: success={bool(result.success)} result={int(result.result)}',
        ))

    def publish_setpoint_callback(self):
        if not self.publish_setpoint:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.setpoint[0]
        msg.pose.position.y = self.setpoint[1]
        msg.pose.position.z = self.setpoint[2]
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)
        self.last_setpoint_publish = time.monotonic()

    def landed_state_name(self, landed_state):
        names = {
            ExtendedState.LANDED_STATE_UNDEFINED: 'UNDEFINED',
            ExtendedState.LANDED_STATE_ON_GROUND: 'ON_GROUND',
            ExtendedState.LANDED_STATE_IN_AIR: 'IN_AIR',
            ExtendedState.LANDED_STATE_TAKEOFF: 'TAKEOFF',
            ExtendedState.LANDED_STATE_LANDING: 'LANDING',
        }
        return names.get(landed_state, 'UNKNOWN')


class CollapsibleText(ttk.Frame):
    def __init__(self, parent, title):
        super().__init__(parent)
        self.visible = tk.BooleanVar(value=True)
        self.toggle = ttk.Checkbutton(
            self,
            text=title,
            variable=self.visible,
            command=self.update_visibility,
            style='Toolbutton',
        )
        self.toggle.pack(fill='x', pady=(0, 4))

        self.body = ttk.Frame(self)
        self.body.pack(fill='both', expand=True)
        self.text = tk.Text(self.body, height=10, width=54, wrap='none')
        self.text.pack(side='left', fill='both', expand=True)
        scroll = ttk.Scrollbar(self.body, orient='vertical', command=self.text.yview)
        scroll.pack(side='right', fill='y')
        self.text.configure(yscrollcommand=scroll.set)

    def update_visibility(self):
        if self.visible.get():
            self.body.pack(fill='both', expand=True)
        else:
            self.body.forget()

    def set_text(self, value):
        self.text.configure(state='normal')
        self.text.delete('1.0', tk.END)
        self.text.insert(tk.END, value)
        self.text.configure(state='disabled')

    def append_line(self, value):
        self.text.configure(state='normal')
        self.text.insert(tk.END, value + '\n')
        self.text.see(tk.END)
        self.text.configure(state='disabled')


class DebugMavrosApp:
    def __init__(self, root, command_queue, gui_queue):
        self.root = root
        self.command_queue = command_queue
        self.gui_queue = gui_queue

        self.root.title('MAVROS Debug')
        self.root.geometry('980x620')

        self.mode_var = tk.StringVar(value='OFFBOARD')
        self.x_var = tk.StringVar(value='0.0')
        self.y_var = tk.StringVar(value='0.0')
        self.z_var = tk.StringVar(value='3.0')
        self.setpoint_running = False

        self.build_ui()
        self.poll_gui_queue()

    def build_ui(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill='both', expand=True)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        controls = ttk.Frame(main, padding=(0, 0, 10, 0))
        controls.grid(row=0, column=0, sticky='ns')

        ttk.Label(controls, text='Mode').pack(anchor='w')
        mode_select = ttk.Combobox(
            controls,
            textvariable=self.mode_var,
            values=MODES,
            state='normal',
            width=18,
        )
        mode_select.pack(fill='x', pady=(2, 8))
        ttk.Button(controls, text='Send Mode', command=self.send_mode).pack(fill='x', pady=(0, 14))

        ttk.Label(controls, text='Setpoint').pack(anchor='w')
        self.add_field(controls, 'X', self.x_var)
        self.add_field(controls, 'Y', self.y_var)
        self.add_field(controls, 'Z', self.z_var)
        self.setpoint_button = ttk.Button(
            controls,
            text='Start Publishing Setpoint',
            command=self.toggle_setpoint,
        )
        self.setpoint_button.pack(fill='x', pady=(4, 14))

        ttk.Button(controls, text='Arm', command=lambda: self.command_queue.put(('arm',))).pack(
            fill='x',
            pady=(0, 6),
        )
        ttk.Button(controls, text='Disarm', command=lambda: self.command_queue.put(('disarm',))).pack(
            fill='x',
            pady=(0, 14),
        )
        ttk.Button(controls, text='Land', command=lambda: self.command_queue.put(('land',))).pack(
            fill='x',
            pady=(0, 14),
        )

        self.command_box = CollapsibleText(controls, 'Command Results')
        self.command_box.pack(fill='both', expand=True)

        right = ttk.Frame(main)
        right.grid(row=0, column=1, sticky='nsew')
        right.rowconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self.state_box = CollapsibleText(right, '/mavros/state')
        self.state_box.grid(row=0, column=0, sticky='nsew', pady=(0, 8))
        self.extended_state_box = CollapsibleText(right, '/mavros/extended_state')
        self.extended_state_box.grid(row=1, column=0, sticky='nsew')

    def add_field(self, parent, label, variable):
        row = ttk.Frame(parent)
        row.pack(fill='x', pady=2)
        ttk.Label(row, text=label, width=2).pack(side='left')
        ttk.Entry(row, textvariable=variable, width=12).pack(side='left', fill='x', expand=True)

    def send_mode(self):
        mode = self.mode_var.get().strip()
        if not mode:
            self.command_box.append_line('Mode is empty')
            return
        self.command_queue.put(('set_mode', mode))

    def toggle_setpoint(self):
        if self.setpoint_running:
            self.command_queue.put(('stop_setpoint',))
            self.setpoint_running = False
            self.setpoint_button.configure(text='Start Publishing Setpoint')
            return

        try:
            setpoint = (
                float(self.x_var.get()),
                float(self.y_var.get()),
                float(self.z_var.get()),
            )
        except ValueError:
            self.command_box.append_line('Setpoint fields must be numbers')
            return

        self.command_queue.put(('start_setpoint', setpoint))
        self.setpoint_running = True
        self.setpoint_button.configure(text='Stop Publishing Setpoint')

    def poll_gui_queue(self):
        while True:
            try:
                target, value = self.gui_queue.get_nowait()
            except queue.Empty:
                break

            if target == 'state':
                self.state_box.set_text(value)
            elif target == 'extended_state':
                self.extended_state_box.set_text(value)
            elif target == 'command_result':
                timestamp = time.strftime('%H:%M:%S')
                self.command_box.append_line(f'[{timestamp}] {value}')

        self.root.after(100, self.poll_gui_queue)


def run_ros(gui_queue, command_queue, stop_event):
    rclpy.init()
    node = MavrosDebugNode(gui_queue, command_queue)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main():
    gui_queue = queue.Queue()
    command_queue = queue.Queue()
    stop_event = threading.Event()

    ros_thread = threading.Thread(
        target=run_ros,
        args=(gui_queue, command_queue, stop_event),
        daemon=True,
    )
    ros_thread.start()

    root = tk.Tk()
    DebugMavrosApp(root, command_queue, gui_queue)

    def on_close():
        stop_event.set()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()
    ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
