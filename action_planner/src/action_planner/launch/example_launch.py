"""Standalone demonstration of the action_planner framework with no robot attached.

Brings up the framework nodes, a static problem provider and two stub action
executors named after the actions of examples/example_domain.pddl. Once
everything is active, mission_controller fetches the problem, action_planner
calls the solver and the plan (move/inspect steps) runs against the stubs.
Start action_planner_cli first if you want to trigger the mission by hand.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('action_planner')

    solver_arg = DeclareLaunchArgument('solver', default_value='OPTIC', description='OPTIC or TFD')

    # Dependency table consumed by lifecycle_manager: "node" or "node:dep1,dep2".
    managed_nodes = [
        'problem_generator',
        'move',
        'inspect',
        'action_planner:move,inspect',
        'mission_controller:action_planner,problem_generator',
    ]

    nodes = [
        Node(package='action_planner', executable='lifecycle_manager.py', name='lifecycle_manager',
             output='screen', parameters=[{'managed_nodes': managed_nodes}]),
        Node(package='action_planner', executable='command_interface.py', name='command_interface',
             output='screen'),
        Node(package='action_planner', executable='mission_controller.py', name='mission_controller',
             output='screen'),
        Node(package='action_planner', executable='action_planner_node.py', name='action_planner',
             output='screen',
             parameters=[{
                 'pddl_domain': os.path.join(share_dir, 'examples', 'example_domain.pddl'),
                 'solver': LaunchConfiguration('solver'),
             }]),
        Node(package='action_planner', executable='static_problem_provider.py', name='problem_generator',
             output='screen',
             parameters=[{'problem_file': os.path.join(share_dir, 'examples', 'example_problem.json')}]),
        Node(package='action_planner', executable='timed_action_node.py', name='move',
             output='screen', parameters=[{'n_iterations': 3}]),
        Node(package='action_planner', executable='timed_action_node.py', name='inspect',
             output='screen', parameters=[{'n_iterations': 2}]),
    ]

    ld = LaunchDescription()
    ld.add_action(solver_arg)
    for node in nodes:
        ld.add_action(node)
    return ld
