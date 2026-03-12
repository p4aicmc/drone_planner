#!/usr/bin/env python3

import rclpy
import json
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from harpia_msgs.srv import StrInOut


class MapUpdater(Node):

    def __init__(self):
        super().__init__('map_updater')

        package_share_dir = get_package_share_directory('route_executor2')
        self.update_file = os.path.join(package_share_dir, 'data', 'map update1.json')

        self.client = self.create_client(StrInOut, 'data_server/update_map')

        # Call the update after 90 seconds
        self.timer = self.create_timer(90.0, self.send_update)
        self.get_logger().info('Map updater started — will send update in 90 seconds')

    def send_update(self):
        self.destroy_timer(self.timer)

        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('data_server/update_map service not available')
            return

        try:
            with open(self.update_file, 'r') as f:
                map_data = f.read()
            # Validate JSON before sending
            json.loads(map_data)
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {self.update_file}')
            return
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in update file: {e}')
            return

        request = StrInOut.Request()
        request.message = map_data

        self.get_logger().info('Sending map update...')
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Map updated successfully: {response.message}')
            else:
                self.get_logger().error(f'Map update failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MapUpdater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
