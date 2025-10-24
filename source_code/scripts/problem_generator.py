#!/usr/bin/env python3
import rclpy
from std_srvs.srv import Trigger
import json
import math
import time

from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import String

class ProblemGenerator(LifecycleNode):
    def __init__(self):
        super().__init__('problem_generator')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info("Configuring node...")

            self.data = {
                "map": None,
                "hardware": None,
                "mission": None
            }
            self.current_region = None
            self.last_problem = None

            self.map_cli = self.create_client(Trigger, 'data_server/map')
            self.hardware_cli = self.create_client(Trigger, 'data_server/hardware')
            self.mission_cli = self.create_client(Trigger, 'data_server/mission')
            self.gps_cli = self.create_client(Trigger, 'data_server/gps_position')
            self.mission_updates_subscription = self.create_subscription(String, 'data_server/mission_updates', self.mission_update_callback, 10)

            self.getData()

            self.get_current_position()

            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during configuration: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node...")

        if self.data["map"] is None or self.data["hardware"] is None or self.data["mission"] is None:
            self.get_logger().info("Can't activate node: map, hardware or mission data is missing")
            return TransitionCallbackReturn.FAILURE
        
        if self.current_region is None:
            self.get_logger().info("Can't activate node: current region is missing")
            return TransitionCallbackReturn.FAILURE

        self.srv = self.create_service(Trigger, 'problem_generator/get_problem', self.get_problem_callback)

        self.problem_update_publisher = self.create_publisher(String, 'problem_generator/problem_update', 10)

        return TransitionCallbackReturn.SUCCESS

    def getData(self):

        def handle_response(future, attr_name):
            if not future.result():
                self.get_logger().error(f"Failed to retrieve {attr_name} data")
                return
            
            self.get_logger().info(f"Received {attr_name} data successfully")
            self.data[attr_name] = json.loads(future.result().message)


        # Request map data
        map_req = Trigger.Request()
        future = self.map_cli.call_async(map_req)
        future.add_done_callback(lambda f: handle_response(f, "map"))

        # Request hardware status
        hardware_req = Trigger.Request()
        future = self.hardware_cli.call_async(hardware_req)
        future.add_done_callback(lambda f: handle_response(f, "hardware"))

        # Request mission data
        mission_req = Trigger.Request()
        future = self.mission_cli.call_async(mission_req)
        future.add_done_callback(lambda f: handle_response(f, "mission"))
    
    def get_problem_callback(self, request, response):

        if self.data['map'] is None or self.data['hardware'] is None or self.data['mission'] is None:
            response.success = False
            self.get_logger().error("map, hardware or mission data is missing")
            raise ValueError("map, hardware or mission data is missing")
            # return response

        self.last_problem = self.generate_problem()
        response.success = True
        response.message = json.dumps(self.last_problem)
        return response
    
    def generate_problem(self):

        result = self.current_region
        if result is None:
            self.get_logger().info("Failed to get position")
            return
        lat, lon, alt, inside_region = result
        
        if inside_region == "":
            raise ValueError("Drone is not inside any region")

        EARTH_RADIUS_M = 6371000  # Raio da Terra em metros
        def haversine(reg1, reg2):
            lat1 = reg1['center'][1]
            lon1 = reg1['center'][0]
            lat2 = reg2['center'][1]
            lon2 = reg2['center'][0]
           
            # Converte graus para radianos
            lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

            # Diferenças das coordenadas
            dlat = lat2 - lat1
            dlon = lon2 - lon1

            # Fórmula de Haversine
            a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

            # Distância em metros
            return EARTH_RADIUS_M * c

        
        regions_list = []
        instances = {
            'region': [],
            'base': [],
        }
        predicates = []
        functions = []
        goals = []


        for base in self.data["map"]["bases"]:
            instances['base'].append(base["name"])
            regions_list.append(base)

        for reg in self.data["map"]["roi"]:
            instances['region'].append(reg["name"])
            regions_list.append(reg)

        
        for region1 in regions_list:
            for region2 in regions_list:
                if region1['name'] == region2['name']:
                    functions.append(f"(distance {region1['name']} {region1['name']}) 0.0")
                else:
                    functions.append(f"(distance {region1['name']} {region2['name']}) {haversine(region1, region2)}")

        for mission_command in self.data['mission']['mission_execution']:
            command = mission_command["command"]
            area = mission_command["instructions"]["area"]

            if command == "take_picture":
                predicates.append(f"picture_goal {area}")
                goals.append(f"taken_image {area}")
            
            elif command == "end":
                goals.append(f"at {area}")
            
            else:
                self.get_logger().error(f"Comando '{command}' desconhecido")

        functions.append(f"(battery_capacity) {self.data['hardware']['battery-capacity']}")
        # functions.append(f"(= (discharge_rate_battery) {self.data['hardware']['discharge-rate-battery']})")
        # functions.append(f"(discharge_rate_battery) {0.1}")
        # functions.append(f"(velocity) {self.data['hardware']['efficient_velocity']}")
        functions.append(f"(input_capacity) {self.data['hardware']['input-capacity']}")

        functions.append(f"(battery_amount) {self.data['hardware']['battery-capacity']}")
        # functions.append(f"(battery_amount) 0")
        functions.append(f"(input_amount) {self.data['hardware']['input-capacity']}")
        predicates.append(f"at {inside_region}")
        
        functions.append(f"(mission_length) 0.0")

        return {
            "instances": [ f'{name} {key}' for key in instances.keys() for name in instances[key] ],
            "predicates": predicates,
            "functions": functions,
            "goals": goals
        }
    
    def get_current_position(self, strm=""):
        
        if not self.gps_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('GPS service is not available!')
            return None


        def gps_callback(furure):
            
            if future.result() is None:
                self.get_logger().error('Service call failed')
                return None

            if not future.result().success:
                self.get_logger().error('Service could not return valid response')
                return None
            lat, lon, alt, inside_regions = future.result().message.split(" ")
            inside_region = inside_regions.split(',')[0]

            self.current_region = lat, lon, alt, inside_region

        gps_req = Trigger.Request()
        future = self.gps_cli.call_async(gps_req)
        future.add_done_callback(gps_callback)
    
    def mission_update_callback(self, msg: String):

        try:
            updates = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"ERROR, the mission update json in invalid")
            return
        
        if self.last_problem is None:
            self.get_logger().error("Got update before processing the problem")
            raise ValueError("Got update before processing the problem")

        mission_before = self.data['mission']['mission_execution'].copy()
        self.update_mission(updates)
        mission_after = self.data['mission']['mission_execution'].copy()

        # check if the mission changed
        if self.are_missions_equal(mission_before, mission_after):
            self.get_logger().info('Mission update didnt change the mission, so problem update will not be published')
            return
        
        self.generate_and_publish_problem_update()

    def update_mission(self, updates):
        for update in updates:
            found_idx = [] # stores the indexes of the selected commands
            for i, cmd, in enumerate(self.data["mission"]['mission_execution']):
                if cmd['instructions']['area'] == update['area'] and cmd['command'] == update['command']:
                    found_idx.append(i)
            if update['type'] == 'remove':
                for i in sorted(found_idx, reverse=True):
                    if 0 <= i < len(self.data["mission"]['mission_execution']):
                        del self.data["mission"]['mission_execution'][i]
            elif update['type'] == 'add':
                if len(found_idx) == 0: # avoid copies of the same command
                    self.data["mission"]['mission_execution'].append({'instructions': {'area': update['area']}, 'command': update['command']},)

    def are_missions_equal(self, mission_1, mission_2):

        if len(mission_1) != len(mission_2):
            return False

        m = [[],[]]
        for i in range(2):
            for cmd in [mission_1,mission_2][i]:
                m[i].append(f"{cmd['command']},{cmd['instructions']['area']}")
            m[i] = sorted(m[i], reverse=True)
        
        self.get_logger().info(f"comparing: {m}")

        for i in range(len(mission_1)):
            if m[0] != m[1]:
                return False
        
        return True
    
    def generate_and_publish_problem_update(self):
        
        # generate new problem
        new_problem = self.generate_problem()
        
        problem_updates = {}
        # check what changed in the problem and saves in the problem_updates dict what was added and what was removed
        for object_name in ['instances', 'predicates', 'functions']:
            problem_updates[object_name] = {'add': []}

            old_objects = self.last_problem[object_name].copy()

            for new_object in new_problem[object_name]:
                try:
                    old_idx = old_objects.index(new_object)
                    del old_objects[old_idx]
                except ValueError:
                    problem_updates[object_name]['add'].append(new_object)

            problem_updates[object_name]['remove'] = old_objects # the remaining objects in the old_objects are removed ones 

        # Convert the problem updates to the format required by the plansys_interface
        formatted_updates = []
        for key, value in problem_updates.items():
            self.get_logger().info(f'{key}: {value}')

            for action in ['add', 'remove']:
                if len(value[action]) > 0:
                    formatted_updates.append({
                        'type': f'{action}_{key}',
                        'values': value[action]
                    })
        if len(new_problem['goals']) > 0:
            formatted_updates.append({
                'type': f'set_goals',
                'values': new_problem['goals']
            })

        # saves the problem
        self.last_problem = new_problem
        
        # publish updates
        msg = String()
        msg.data = json.dumps(formatted_updates)
        self.problem_update_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProblemGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
