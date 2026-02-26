#!/usr/bin/env python3

import rclpy
import math
import json
from threading import Thread
from rclpy.node import Node
from harpia_msgs.srv import GenerateSurveyPath
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import time, os, sys
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

# Adiciona o diretório base ao sys.path
libs_path = os.path.join(os.path.dirname(__file__), './libs')
sys.path.append(os.path.abspath(libs_path))

class v2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __add__(self, other):
        return v2(self.x + other.x, self.y + other.y)
    def __sub__(self, other):
        return v2(self.x - other.x, self.y - other.y)
    def __mul__(self, scalar):
        return v2(self.x * scalar, self.y * scalar)
    def __truediv__(self, scalar):
        return v2(self.x / scalar, self.y / scalar)
    def length(self):
        return math.sqrt(self.x**2 + self.y**2)
    def to_list(self):
        return [self.x, self.y]
    
class SurveyPathGen(LifecycleNode):
    """Path planning node for generating and executing paths."""

    def __init__(self):
        """Initializes the path planner node, map, and service."""
        super().__init__('survey_path_gen')   

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        pass # self.get_logger().info("Configuring node...") # NOT_ESSENTIAL_PRINT
        
        self.home_lat = None
        self.home_lon = None

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        pass # self.get_logger().info("Activating node...") # NOT_ESSENTIAL_PRINT

        self.srv = self.create_service(GenerateSurveyPath, 'survey_path_gen/generate_path', self.generate_path_callback)
        pass # self.get_logger().info("Path planner service is ready to generate paths.") # NOT_ESSENTIAL_PRINT

        return TransitionCallbackReturn.SUCCESS
    
    def generate_path_callback(self, request, response):
        """
        Gera um caminho em zigue zague na pluma.
        """

        plume_focus = request.focus
        plume_lenth = request.plume_lenth
        plume_angle = request.plume_angle
        plume_direction = request.plume_direction

        self.get_logger().info(f"Plume parameters received: {plume_focus}, {plume_lenth}, {plume_angle}, {plume_direction}") # NOT_ESSENTIAL_PRINT

        # Validar se os parametros são válidos
        if plume_focus is None:
            self.get_logger().error("Invalid plume focus")
            response.waypoints = []  # Resposta vazia em caso de erro
            return response

        # Gera o caminho com a posição ATUALIZADA do drone
        self.get_logger().info(f"Generating path from current position to goal") # NOT_ESSENTIAL_PRINT
        waypointsList = self.generate_path(plume_focus, plume_lenth, plume_angle, plume_direction) or []
        if not waypointsList:
            self.get_logger().error("Waypoints from the actual position are empty.")

        response.waypoints = waypointsList

        self.get_logger().info("Path generation complete.") # NOT_ESSENTIAL_PRINT
        return response


    def generate_path(self, plume_focus, plume_lenth, plume_angle, plume_direction) -> list:

        focus = plume_focus

        waypoints = []

        direction  = v2(1, 1)
        length = 50
        angle = 45
        step = 5

        direction = direction / direction.length()
        direction2 = v2(-direction.y, direction.x)
        angle_step = step*math.tan(math.radians(angle/2))
        sig = 1
        s = v2(focus[0], focus[1])
        for i in range(int(length/step)):

            sig = -sig

            w = s + direction * (i * step) + direction2 * (sig * angle_step * i)
            waypoints.append(self.create_waypoint_message(w.to_list()))
        
        return waypoints

    def create_waypoint_message(self, waypoint):
        """
        Creates a PoseStamped message for a given waypoint.

        Parameters
        ----------
        waypoint : tuple
            (x, y) coordinates of the waypoint.

        Returns
        -------
        PoseStamped
            Message representing the waypoint.
        """
        wp_msg = PoseStamped()
        wp_msg.pose.position.x = waypoint[0]
        wp_msg.pose.position.y = waypoint[1]
        wp_msg.pose.position.z = 10.0
        wp_msg.pose.orientation.w = 1.0  # Garantindo uma orientação válida
        return wp_msg

def main(args=None):
    """
    Main function to initialize the path planner node and handle spinning.
    """
    rclpy.init(args=args)
    survey_path_gen = SurveyPathGen()
    executor = MultiThreadedExecutor()
    executor.add_node(survey_path_gen)
    try:
        executor.spin()
    except KeyboardInterrupt:
        survey_path_gen.get_logger().info('KeyboardInterrupt, shutting down.\n')
    survey_path_gen.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
