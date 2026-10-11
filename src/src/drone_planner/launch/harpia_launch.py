# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Harpia drone system bring-up.

Framework nodes (package action_planner) do the planning and plan execution;
drone_planner nodes provide the world model (data_server, problem_generator),
flight control and the action executors named after the PDDL actions.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Declare launch arguments
    mission_index_arg = DeclareLaunchArgument('mission_index', default_value='1', description='The mission index to execute from the mission file.')
    solver_arg = DeclareLaunchArgument('solver', default_value='OPTIC', description='Planner used by action_planner: OPTIC or TFD.')

    # Use LaunchConfiguration to get the values of arguments
    mission_index_value = LaunchConfiguration('mission_index')

    # Get the launch directory
    share_dir = get_package_share_directory('drone_planner')
    domain_file = os.path.join(share_dir, 'pddl', 'harpia_domain_test.pddl')

    # Start-up order enforced by lifecycle_manager: "node" or "node:dep1,dep2".
    # A node is configured/activated only when all its dependencies are active.
    managed_nodes = [
        'drone_control',
        'data_server:drone_control',
        'take_off:drone_control',
        'land:drone_control',
        'problem_generator:data_server',
        'path_planner:data_server',
        'go_to:path_planner',
        'take_image',
        'recharge_battery',
        'action_planner:go_to,take_image,recharge_battery,take_off,land',
        'mission_controller:action_planner,problem_generator',
    ]

    ld = LaunchDescription()

    nodes_to_add = [
        # ---- framework ----
        Node(
            package='action_planner',
            executable='lifecycle_manager.py',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'managed_nodes': managed_nodes}]
        ),
        Node(
            package='action_planner',
            executable='mission_controller.py',
            name='mission_controller',
            output='screen',
            parameters=[]
        ),
        Node(
            package='action_planner',
            executable='command_interface.py',
            name='command_interface',
            output='screen',
            parameters=[]
        ),
        Node(
            package='action_planner',
            executable='action_planner_node.py',
            name='action_planner',
            output='screen',
            parameters=[{'pddl_domain': domain_file, 'solver': LaunchConfiguration('solver')}]
        ),
        # ---- drone system ----
        Node(
            package='drone_planner',
            executable='route_executor.py',
            name='route_executor',
            output='screen',
            parameters=[]
        ),
        Node(
            package='drone_planner',
            executable='drone_control.py',
            name='drone_control',
            output='screen',
            parameters=[]
        ),
        Node(
            package='drone_planner',
            executable='path_planner.py',
            name='path_planner',
            output='screen',
            parameters=[]
        ),
        Node(
            package='drone_planner',
            executable='data_server.py',
            name='data_server',
            output='screen',
            parameters=[{'mission_index': mission_index_value}]
        ),
        Node(
            package='drone_planner',
            executable='problem_generator.py',
            name='problem_generator',
            output='screen',
            parameters=[]
        ),
        Node(
            package='drone_planner',
            executable='go_to.py',
            name='go_to',
            output='screen',
            parameters=[]
        ),
        Node(
            package='drone_planner',
            executable='take_image.py',
            name='take_image',
            output='screen',
            parameters=[]
        ),
        Node(
            package='drone_planner',
            executable='recharge_battery.py',
            name='recharge_battery',
            output='screen',
            parameters=[]
        ),
        Node(
            package='drone_planner',
            executable='take_off.py',
            name='take_off',
            output='screen',
            parameters=[]
        ),
        Node(
            package='drone_planner',
            executable='land.py',
            name='land',
            output='screen',
            parameters=[]
        ),
    ]

    # Create the launch description and populate
    ld.add_action(mission_index_arg)
    ld.add_action(solver_arg)

    for node in nodes_to_add:
        ld.add_action(node)

    return ld
