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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Declare launch arguments
    mission_index_arg = DeclareLaunchArgument('mission_index', default_value='1', description='The mission index to execute from the mission file.')

    # Use LaunchConfiguration to get the values of arguments
    mission_index_value = LaunchConfiguration('mission_index')
    
    # Get the launch directory
    share_dir = get_package_share_directory('route_executor2')
    domain_file = share_dir + '/pddl/harpia_domain_test.pddl'

    ld = LaunchDescription()

    nodes_to_add = [
        # IncludeLaunchDescription( # plansys
        #     PythonLaunchDescriptionSource(os.path.join(
        #         get_package_share_directory('plansys2_bringup'),
        #         'launch',
        #         'plansys2_bringup_launch_monolithic.py')),
        #     launch_arguments={'model_file': domain_file}.items()
        # ),
        Node(
            package='route_executor2',
            executable='lifecycle_manager.py',
            name='lifecycle_manager',
            output='screen',
            parameters=[]
        ),
        Node(
            package='route_executor2',
            executable='mission_controller.py',
            name='mission_controller',
            output='screen',
            parameters=[]
        ),
        Node(
            package='route_executor2',
            executable='route_executor.py',
            name='route_executor',
            output='screen',
            parameters=[]
        ),
        Node(
            package='route_executor2',
            executable='path_planner.py',
            name='path_planner',
            output='screen',
            parameters=[]
        ),
        Node(
            package='route_executor2',
            executable='data_server.py',
            name='data_server',
            output='screen',
            parameters=[{'mission_index': mission_index_value}]
        ),
        Node(
            package='route_executor2',
            executable='problem_generator.py',
            name='problem_generator',
            output='screen',
            parameters=[]
        ),
        Node(
            package='route_executor2',
            executable='action_planner.py',
            name='action_planner',
            output='screen',
            # parameters=[{'pddl_domain': share_dir + '/pddl/harpia_domain.pddl'}]
            parameters=[{"pddl_domain": share_dir+'/pddl/harpia_domain_test.pddl'}]
        ),
        Node(
            package='route_executor2',
            executable='go_to.py',
            name='new_go_to',
            output='screen',
            parameters=[]
        ),
        Node(
            package='route_executor2',
            executable='take_image.py',
            name='new_take_image',
            output='screen',
            parameters=[]
        ),
        Node(
            package='route_executor2',
            executable='recharge_battery.py',
            name='new_recharge_battery',
            output='screen',
            parameters=[]
        ),
    ]

    # Create the launch description and populate
    ld.add_action(mission_index_arg)

    for node in nodes_to_add:
        ld.add_action(node)

    return ld