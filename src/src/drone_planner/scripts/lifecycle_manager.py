#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, TransitionEvent

class LifecycleMannager(Node):
    def __init__(self):
        super().__init__('lifecycle_controller')

        self.nodes = {}
        self.state_clients = {}
        self.change_state_clients = {}
        self.pending_service_calls = {}
        self.wait_for_nodes_timer = None

        self.activating_nodes = False
        self.activating_nods_timer = None

    def add_nodes(self, nodes):
        for node in nodes:
            self.add_node(node['node_name'], node['depends_on'])

    def add_node(self, node_name, depends_on):
        # self.get_logger().info(f'Adding node {node_name}')

        sub = self.create_subscription(
            TransitionEvent,
            f'{node_name}/transition_event',
            lambda msg: self.transition_event_callback(node_name, msg),
            10)
        
        self.nodes[node_name] = {
            'depends_on': depends_on,
            'state': None,
            'sub': sub,
            'update_check': False,
            'pending_transition': None,
        }

        tries = 0
        def check_service_availability(node_name, timer, client):
            nonlocal tries
            tries += 1
            if client.service_is_ready():
                # self.get_logger().info(f'{node_name} OK')
                timer.cancel()  # Stop the timer

                # Create a request
                def handle_get_state_response(future):
                    try:
                        response = future.result()
                    except Exception as e:
                        self.get_logger().error(f'Get state request for {node_name} failed: {e}')
                        return

                    if response is not None:
                        state = response.current_state.label
                        # self.get_logger().info(f'get state: {node_name} -> {state}')
                        self.nodes[node_name]['state'] = state

                request = GetState.Request()
                future = client.call_async(request)
                self.track_future(f'get_state:{node_name}:initial', future, handle_get_state_response)

            elif tries > 100:
                self.get_logger().error(f'Could not find lifecycle node {node_name}.')
                timer.cancel()
                del self.nodes[node_name]


        client = self.create_client(GetState, f'/{node_name}/get_state')
        self.state_clients[node_name] = client
        self.change_state_clients[node_name] = self.create_client(ChangeState, f'/{node_name}/change_state')
        timer = self.create_timer(0.2, lambda: check_service_availability(node_name, timer, client))
        
    def wait_for_nodes_and_start(self):

        if self.wait_for_nodes_timer is None:

            def wait_for_nodes():
                # self.get_logger().info(f'Checking nodes...')

                if all([node['state'] is not None for node in self.nodes.values()]):
                    self.wait_for_nodes_timer.cancel()
                    self.wait_for_nodes_timer = None
                    self.get_logger().info('All nodes are running, starting active nodes...')
                    self.start_nodes_activation()

            self.wait_for_nodes_timer = self.create_timer(0.2, wait_for_nodes)
        else:
            self.get_logger().warn('wait_for_nodes_timer already exists')

    def start_nodes_activation(self):
        self.activating_nodes = True

        if self.activating_nods_timer is not None:
            self.activating_nods_timer.cancel()

        self.activating_nods_timer = self.create_timer(0.5, self.configure_unconfigured_nodes)

    def stop_nodes_activation(self):
        self.activating_nods_timer.cancel()
        self.activating_nodes = False

    def configure_unconfigured_nodes(self):

        states = [ (key, self.nodes[key]['state']) for key in self.nodes.keys() ]
        # self.get_logger().info(f'Current states: {states}')

        if all([ node['state'] == 'active' for node in self.nodes.values() ]):
            self.get_logger().info('Finished activating all lifecycle nodes')
            self.stop_nodes_activation()
            return

        for node_name, node in self.nodes.items():

            # if all dependencies are active
            if node['state']=='unconfigured' and all([ self.nodes[dep]['state'] == 'active' for dep in node['depends_on'] ]):
                if node['pending_transition'] is not None:
                    continue
                # self.get_logger().info(f'{node_name} is unconfigured and all its dependent nodes are active. Starting...')
                self.trigger_node_transition(node_name, 'configure')

            if node['state']=='inactive' and all([ self.nodes[dep]['state'] == 'active' for dep in node['depends_on'] ]):
                if node['pending_transition'] is not None:
                    continue
                self.trigger_node_transition(node_name, 'activate')

    def trigger_node_transition(self, node_name, transition):

        # self.get_logger().info(f'Triggering {transition} for node {node_name}')
        if self.nodes[node_name]['pending_transition'] is not None:
            return

        self.nodes[node_name]['pending_transition'] = transition
        self.nodes[node_name]['update_check'] = False
        if 'tries' not in self.nodes[node_name]:
            self.nodes[node_name]['tries'] = 0
    
        def node_transition_callback(future):
            try:
                response = future.result()
            except Exception as e:
                self.nodes[node_name]['pending_transition'] = None
                self.get_logger().error(f'Request to {transition} node {node_name} failed: {e}')
                return

            if response is None:
                self.nodes[node_name]['pending_transition'] = None
                self.get_logger().error(f'Request to {transition} node {node_name} failed')
                return
            
            if not response.success:
                # self.get_logger().info(f'Failed to {transition} node {node_name}')
                self.nodes[node_name]['pending_transition'] = None
                self.refresh_node_state(node_name)

                return
            
            # self.get_logger().info(f'{transition} request for {node_name} succeeded')
            del self.nodes[node_name]['tries']
            if transition == 'configure':
                self.nodes[node_name]['state'] = 'inactive'
            elif transition == 'activate':
                self.nodes[node_name]['state'] = 'active'
            self.nodes[node_name]['pending_transition'] = None
            self.refresh_node_state(node_name)

        self.call_change_state(node_name, transition, lambda f: node_transition_callback(f))

    def refresh_node_state(self, node_name):
        client = self.state_clients[node_name]
        if not client.service_is_ready():
            return

        def handle_get_state_response(future):
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error(f'Get state request for {node_name} failed: {e}')
                return

            if response is not None:
                self.nodes[node_name]['state'] = response.current_state.label

        request = GetState.Request()
        future = client.call_async(request)
        self.track_future(f'get_state:{node_name}:refresh', future, handle_get_state_response)

    def call_change_state(self, node_name, transition, callback):

        client = self.change_state_clients[node_name]
        if not client.service_is_ready():
            self.get_logger().error(f'Could not change state of node {node_name}. Service not available.')
            self.nodes[node_name]['pending_transition'] = None
            return
        
        transition_label_to_id = {
            'configure': Transition.TRANSITION_CONFIGURE,
            'cleanup': Transition.TRANSITION_CLEANUP,
            'activate': Transition.TRANSITION_ACTIVATE,
            'deactivate': Transition.TRANSITION_DEACTIVATE,
        }
        
        # Create a request
        request = ChangeState.Request()
        request.transition.id = transition_label_to_id[transition]
        request.transition.label = transition
        
        future = client.call_async(request)
        self.track_future(f'change_state:{node_name}:{transition}', future, callback)

    def track_future(self, key, future, callback):
        self.pending_service_calls[key] = future

        def tracked_callback(done_future):
            try:
                callback(done_future)
            finally:
                self.pending_service_calls.pop(key, None)

        future.add_done_callback(tracked_callback)

    def transition_event_callback(self, node_name, msg):
        state = msg.goal_state.label
        # self.get_logger().info(f'get transition: {node_name} -> {state}')
        self.nodes[node_name]['state'] = state
        self.nodes[node_name]['update_check'] = True

        if state == 'active':
            self.get_logger().info(f'{node_name} activated successfully')

nodes = [
    {
        'node_name': 'data_server',
        'depends_on': ['drone_control']
    },
    {
        'node_name': 'problem_generator',
        'depends_on': ['data_server']
    },
    {
        'node_name': 'path_planner',
        'depends_on': ['data_server']
    },
    {
        'node_name': 'drone_control',
        'depends_on': []
    },
    {
        'node_name': 'mission_controller',
        'depends_on': ['action_planner', 'problem_generator'] # not anymore 'plansys_interface'
    },
    {
        'node_name': 'action_planner',
        'depends_on': ['go_to', 'take_image', 'recharge_battery', 'take_off', 'land']
    },
    {
        'node_name': 'go_to',
        'depends_on': ['path_planner']
    },
    {
        'node_name': 'take_image',
        'depends_on': []
    },
    {
        'node_name': 'recharge_battery',
        'depends_on': []
    },
    {
        'node_name': 'take_off',
        'depends_on': ['drone_control']
    },
    {
        'node_name': 'land',
        'depends_on': ['drone_control']
    },
]

def main(args=None):
    rclpy.init(args=args)
    manager_node = LifecycleMannager()

    manager_node.add_nodes(nodes)
    manager_node.wait_for_nodes_and_start()

    rclpy.spin(manager_node)
    manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
