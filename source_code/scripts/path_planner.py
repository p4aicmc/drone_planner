#!/usr/bin/env python3

import rclpy
import math
import json
from threading import Thread
from rclpy.node import Node
from harpia_msgs.srv import GeneratePath
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

from libs.RRT.rrt import RRT
from libs.RayCasting.raycasting import Vector
class CoordinateConverter:
    """Coordinate conversion utilities."""

    @staticmethod
    def geo_to_cart(geo_point, geo_home):
        """
        Converts geographic coordinates to Cartesian coordinates.

        Parameters
        ----------
        geo_point : tuple
            A tuple representing the latitude and longitude of the point to convert.
        geo_home : tuple
            A tuple representing the latitude and longitude of the home reference point.

        Returns
        -------
        tuple
            Cartesian coordinates (x, y) relative to the home reference point.
        """
        def calc_y(lat, home_lat):
            return (lat - home_lat) * 111320.0

        def calc_x(longi, home_long, home_lat):
            return (longi - home_long) * (111320.0 * math.cos(home_lat * math.pi / 180))

        x = calc_x(geo_point[1], geo_home[1], geo_home[0])  # Longitude
        y = calc_y(geo_point[0], geo_home[0])  # Latitude

        return x, y


class Map:
    """Map data handling with conversion and parsing capabilities."""

    def __init__(self, home_lat, home_lon):
        """
        Initializes the map with the given home coordinates.

        Parameters
        ----------
        home_lat : float
            Latitude of the home position.
        home_lon : float
            Longitude of the home position.
        """
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.converter = CoordinateConverter()
        self.bases = []
        self.rois = []
        self.nfz = []

    def read_route_from_json(self, map_file):
        """
        Reads route data from a JSON file and populates bases, ROIs, and NFZs.

        Parameters
        ----------
        map_file : str
            Path to the JSON file containing route data.
        """
        
        data = json.loads(map_file)

        for base in data.get('bases', []):
            center = base.get('center')
            geo_points = base.get('geo_points', [])
            if center and geo_points:
                enu_center_x, enu_center_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                enu_geo_points = [
                    self.converter.geo_to_cart((gp[1], gp[0]), (self.home_lat, self.home_lon)) for gp in geo_points
                ]
                self.bases.append({
                    'id': base.get('id'),
                    'name': base.get('name'),
                    'center': (enu_center_x, enu_center_y),
                    'geo_points': enu_geo_points
                })

        for roi in data.get('roi', []):
            center = roi.get('center')
            geo_points = roi.get('geo_points', [])
            if center and geo_points:
                enu_center_x, enu_center_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                enu_geo_points = [
                    self.converter.geo_to_cart((gp[1], gp[0]), (self.home_lat, self.home_lon)) for gp in geo_points
                ]
                self.rois.append({
                    'id': roi.get('id'),
                    'name': roi.get('name'),
                    'center': (enu_center_x, enu_center_y),
                    'geo_points': enu_geo_points
                })

        for nfz in data.get('nfz', []):
            center = nfz.get('center')
            geo_points = nfz.get('geo_points', [])
            if center and geo_points:
                enu_center_x, enu_center_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                enu_geo_points = [
                    self.converter.geo_to_cart((gp[1], gp[0]), (self.home_lat, self.home_lon)) for gp in geo_points
                ]
                self.nfz.append({
                    'id': nfz.get('id'),
                    'name': nfz.get('name'),
                    'center': (enu_center_x, enu_center_y),
                    'geo_points': enu_geo_points
                })
        
        self.obstacleList = list()
        # Iterando pelos NFZs no mapa
        for nfz in self.nfz:
            # Lista temporária para armazenar os vetores de um único geo_point
            obstacle = []

            for geo_point in nfz.get('geo_points', []):  # Certifica-se de que 'geo_points' existe
                vector = Vector(geo_point[0], geo_point[1])  # Cria um Vector com x e y
                obstacle.append(vector)  # Adiciona o Vector à lista temporária

            self.obstacleList.append(obstacle)  # Adiciona o obstáculo completo à lista principal

class PathPlanner(LifecycleNode):
    """Path planning node for generating and executing paths."""

    def __init__(self):
        """Initializes the path planner node, map, and service."""
        super().__init__('path_planner')   

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        pass # self.get_logger().info("Configuring node...") # NOT_ESSENTIAL_PRINT
        
        self.home_lat = None
        self.home_lon = None
        self.map = None

        # Declaring callback groups
        map_cb = MutuallyExclusiveCallbackGroup()

        self.home_cli = self.create_client(Trigger, 'data_server/home_position',callback_group=map_cb)
        while not self.home_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('data_server/home_position service not available, waiting again...')
        
        self.position_cli = self.create_client(Trigger, 'data_server/drone_position',callback_group=ReentrantCallbackGroup())
        while not self.position_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('data_server/drone_position service not available, waiting again...')

        self.map_cli = self.create_client(Trigger, 'data_server/map', callback_group=map_cb)
        while not self.map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('data_server/map service not available, waiting again...')

        self.home_req = Trigger.Request()
        self.home_future = self.home_cli.call_async(self.home_req)
        self.home_future.add_done_callback(self.home_response_callback)

        self.drone_position = tuple()
        
        self.send_position_req()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        pass # self.get_logger().info("Activating node...") # NOT_ESSENTIAL_PRINT

        self.srv = self.create_service(GeneratePath, 'path_planner/generate_path', self.generate_path_callback)
        pass # self.get_logger().info("Path planner service is ready to generate paths.") # NOT_ESSENTIAL_PRINT

        return TransitionCallbackReturn.SUCCESS
        
    def home_response_callback(self, future):
        """Callback para processar a resposta do serviço de posição gps da home"""
        try:
            response = future.result()
            position = response.message.split()

            # Garante que a posição recebida tem pelo menos dois valores válidos
            if len(position) < 2:
                self.get_logger().error("Received an invalid position response!")
                return

            self.home_lat = float(position[0])
            self.home_lon = float(position[1])

            self.map = Map(self.home_lat, self.home_lon)
            pass # self.get_logger().info(f"Drone home coordinates: {self.home_lat} {self.home_lon}") # NOT_ESSENTIAL_PRINT

            self.send_map_request()
            
        except Exception as e:
            self.get_logger().error(f'Failed to process position response: {e}')

    def send_map_request(self):
        self.map_req = Trigger.Request()
        self.map_future = self.map_cli.call_async(self.map_req)
        self.map_future.add_done_callback(self.map_response_callback)
    
    def position_response_callback(self, future):
        """Callback para processar a resposta do serviço de posição"""
        try:
            response = future.result()
            position = response.message.split()

            # Garante que a posição recebida tem pelo menos dois valores válidos
            if len(position) < 2:
                self.get_logger().error("Received an invalid position response!")
                return

            self.drone_position = (float(position[0]), float(position[1]))
            pass # self.get_logger().info(f"Drone position updated: {self.drone_position}") # NOT_ESSENTIAL_PRINT
        
        except Exception as e:
            self.get_logger().error(f'Failed to process position response: {e}')
        
    def map_response_callback(self, future):
        # Callback for the data_server/map service
        try:
            response = future.result()
            pass # self.get_logger().info('Map response from data_server') # NOT_ESSENTIAL_PRINT
            self.map.read_route_from_json(response.message)
            # error that sometime happens:
            # AttributeError: 'NoneType' object has no attribute 'read_route_from_json'
        except Exception as e:
            self.get_logger().error(f'Map data_server service call failed: {e}')
            # self.finish(False, 0.0, 'Service call exception') # the finish function doesnt exist

    def find_location_by_name(self, name):
        """
        Searches for a location by name in bases, ROIs, or NFZs.

        Parameters
        ----------
        name : str
            Name of the location to find.

        Returns
        -------
        tuple or None
            Coordinates (x, y) of the location, or None if not found.
        """
        for base in self.map.bases:
            if base['name'] == name:
                return base['center']

        for roi in self.map.rois:
            if roi['name'] == name:
                return roi['center']

        for nfz in self.map.nfz:
            if nfz['name'] == name:
                return nfz['center']

        return None

    def send_position_req(self):
        """Envia requisição ao serviço e aguarda resposta"""
        self.position_req = Trigger.Request()
        self.position_future = self.position_cli.call_async(self.position_req)
        self.position_future.add_done_callback(self.position_response_callback)

    def generate_path_callback(self, request, response):
        """
        Gera um caminho baseado na posição atual do drone e no destino.
        """
        pass # self.get_logger().info(f"Received request to generate path from {request.origin} to {request.destination}") # NOT_ESSENTIAL_PRINT

        pass # self.get_logger().info("Requesting updated drone position...") # NOT_ESSENTIAL_PRINT

        # Armazena a posição antiga antes da atualização
        old_position = self.drone_position
        self.send_position_req()

        # Aguarda até que a posição do drone seja atualizada no callback
        timeout = 5.0  # Tempo máximo de espera em segundos
        start_time = time.time()

        while old_position == self.drone_position:
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting for updated drone position.")
                response.waypoints = []
                return response

            time.sleep(0.1)

        # Encontrar as coordenadas de início e destino
        start = self.find_location_by_name(request.origin)
        goal = self.find_location_by_name(request.destination)

        # Validar se as localizações existem
        if start is None or goal is None:
            self.get_logger().error("Invalid origin or destination name.")
            response.waypoints = []  # Resposta vazia em caso de erro
            return response

        # Gera o caminho com a posição ATUALIZADA do drone
        waypointsList = self.generate_path(self.drone_position, start) or []
        if not waypointsList:
            self.get_logger().error("Waypoints from the actual position are empty.")

        waypointsList.extend(self.generate_path(start, goal) or [])
        response.waypoints = waypointsList

        self.get_logger().info("Path generation complete.") # NOT_ESSENTIAL_PRINT
        return response

    def generate_path(self, start: tuple, goal: tuple ) -> list:
        # Define the rand_area, based on origin and destination coordinates
        ps = [start[0], start[1], goal[0], goal[1]]
        rand_area_x = math.floor(min(ps) * 1.2)
        rand_area_y = math.ceil(max(ps) * 1.2)
        rand_area = [rand_area_x, rand_area_y]
    
        rrt = RRT(
            start=start,
            goal=goal,
            rand_area=rand_area,
            obstacle_list=self.map.obstacleList,
            expand_dis=25,  # minumum precision, to consider inside the goal (meters) 100
            path_resolution=1,
            goal_sample_rate=50,
            max_iter=5000,
            check_collision_mode="ray_casting",
        )

        path = rrt.planning(animation=False)
        if path is not None:
            path = list(reversed(path))
        else:
            self.get_logger().error("Failed to generate a valid path.")
            return list()
        
        # Criar mensagens de waypoints para o caminho gerado
        waypoints = [self.create_waypoint_message(waypoint) for waypoint in path]
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

def spin_srv(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

def main(args=None):
    """
    Main function to initialize the path planner node and handle spinning.
    """
    rclpy.init(args=args)
    path_planner = PathPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(path_planner)
    try:
        executor.spin()
    except KeyboardInterrupt:
        path_planner.get_logger().info('KeyboardInterrupt, shutting down.\n')
    path_planner.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()