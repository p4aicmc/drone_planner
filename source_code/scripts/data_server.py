#!/usr/bin/env python3

import rclpy
import math
import json
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import time, os, sys
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import NavSatFix
from shapely.geometry import Point, Polygon
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import String

class DataServer(LifecycleNode):

    def __init__(self):

        super().__init__('data_server')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring node...")

        package_share_dir = get_package_share_directory('route_executor2')
        self.map_file = f"{package_share_dir}/data/map.json"
        # self.map_file = f"{package_share_dir}/data/map_harpia.json"
        self.hardware_file = f"{package_share_dir}/data/hardware.json"
        self.all_missions_file = f"{package_share_dir}/data/all_missions.json"
        self.mission_index = self.declare_parameter('mission_index', 1).value
        self.home_lat = None
        self.home_long = None
        # Declaring callbackgroups for async callbacks
        self.files_cb = ReentrantCallbackGroup()
        self.position_cb = MutuallyExclusiveCallbackGroup()
        self.gps_cb = MutuallyExclusiveCallbackGroup()
        # Define QoS profile for the position subscription
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )


        try:
            with open(self.map_file, 'r') as file:
                self.map = json.loads(file.read())

            with open(self.all_missions_file, 'r') as file:
                missions = json.load(file)
                self.mission = missions[self.mission_index]

            with open(self.hardware_file, 'r') as file:
                self.hardware = json.load(file)

        except FileNotFoundError as e:
            self.get_logger().error(f"File {e.filename} not found")
            return TransitionCallbackReturn.ERROR
        except json.JSONDecodeError as e:
            self.get_logger().error(f"One of the JSON files is invalid")
            return TransitionCallbackReturn.ERROR
        
        # Subscribe to the drone's current position
        self.position_subscriber = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.position_callback, self.qos_profile, callback_group=self.position_cb
        )
        # Subscribe to the drone's current gps position
        self.gps_subscriber = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, self.sensor_qos, callback_group=self.gps_cb
        )

        self.current_position = None
        self.drone_latitude = None
        self.drone_longitude = None
        self.drone_altitude = None
        self.home_lat = None
        self.home_long = None

        self.position_missing_retry = 0

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node...")

        if any(var is None for var in [
            self.current_position,
            self.drone_latitude,
            self.drone_longitude,
            self.drone_altitude,
            self.home_lat,
            self.home_long
        ]):
            self.position_missing_retry += 1
            if self.position_missing_retry > 5:
                self.get_logger().error("Can't activate node, one or more required position variables are None!")
                self.get_logger().error("Is MAVROS running?")
                
            return TransitionCallbackReturn.FAILURE

        self.map_srv = self.create_service(Trigger, 'data_server/map', self.map_callback, callback_group=self.files_cb)
        self.mission_srv = self.create_service(Trigger, 'data_server/mission', self.mission_callback, callback_group=self.files_cb)
        self.hardware_srv = self.create_service(Trigger, 'data_server/hardware', self.hardware_callback, callback_group=self.files_cb)
        self.position_srv = self.create_service(Trigger, 'data_server/drone_position', self.position_service_callback, callback_group=self.position_cb)
        self.home_srv = self.create_service(Trigger, 'data_server/home_position', self.home_service_callback, callback_group=self.gps_cb)
        self.gps_pos_srv = self.create_service(Trigger, 'data_server/gps_position', self.gps_pos_service_callback, callback_group=self.gps_cb)
        self.mission_update_publisher = self.create_publisher(String, 'data_server/mission_updates', 10)

        # self.test_update()

        return TransitionCallbackReturn.SUCCESS


    ##############################################

    def test_update(self):
        def test_update_callback():
            self.destroy_timer(self.test_timer)
            self.get_logger().info("Publishing mission updates")
            self.publish_mission_updates([
                {
                    'type': 'remove',
                    'area': 'region_3',
                    'command': 'take_picture',
                },
                {
                    'type': 'add',
                    'area': 'region_4',
                    'command': 'take_picture',
                },
            ])
        self.test_timer = self.create_timer(85-5, test_update_callback)
    
    def gps_pos_service_callback(self, request, response):
        inside_regions = self.compute_region()

        if inside_regions is None:
            response.success = False

            return response
    
        response.success = True
        response.message = f'{self.drone_latitude} {self.drone_longitude} {self.drone_altitude} {",".join(inside_regions)}'
        return response

    def home_service_callback(self, request, response):
        response.message = f'{self.home_lat} {self.home_long}'
        return response

    def gps_callback(self, msg):
        self.drone_latitude = msg.latitude
        self.drone_longitude = msg.longitude
        self.drone_altitude = msg.altitude
        if self.home_lat is None:
            # Saves the home latitude and longitude
            self.home_lat = self.drone_latitude
            self.home_long = self.drone_longitude


    def position_service_callback(self, request, response):
        response.message = f'{self.current_position.x} {self.current_position.y} {self.current_position.z}'
        return response
    
    def position_callback(self, msg):
        """
        Updates the drone's current position.

        Parameters
        ----------
        msg : PoseStamped
            The current position of the drone.
        """
        self.current_position = msg.pose.position

    def map_callback(self, request, response):
        # self.get_logger().info(f"Received request to send map")
        response.message = json.dumps(self.map)
        return response

    def mission_callback(self, request, response):
        # self.get_logger().info(f"Received request to send mission")
        response.message = json.dumps(self.mission)
        return response

    def hardware_callback(self, request, response):
        # self.get_logger().info(f"Received request to send hardware")
        response.message = json.dumps(self.hardware)
        return response
    
    def compute_region(self):

        if self.map is None:
            self.get_logger().error('Cant compute region, map is unknow')    
            return None

        if not hasattr(self, 'drone_altitude') or self.drone_altitude is None:
            self.get_logger().error('Cant compute region, drone position is unknow')    
            return None

        regions = []
        region_names = []
        for r in self.map['bases'] + self.map['roi']:
            region = []
            for point in r['geo_points']:
                region.append((point[1],point[0]))
            regions.append(region)
            region_names.append(r['name'])

        start_pos = (self.drone_latitude, self.drone_longitude)

        inside_regions = []
        for i in range(len(regions)):
            if Polygon(regions[i]).contains(Point(start_pos)):
                inside_regions.append(region_names[i])

        return inside_regions
    
    def publish_mission_updates(self, updates):
        self.get_logger().info(f"publishing mission updates")
        msg = String()
        msg.data = json.dumps(updates)
        self.mission_update_publisher.publish(msg)

def main(args=None):
    """
    Main function to initialize the data_server node and handle spinning.
    """
    rclpy.init(args=args)
    data_server = DataServer()
    executor = MultiThreadedExecutor()
    executor.add_node(data_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        data_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    data_server.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()