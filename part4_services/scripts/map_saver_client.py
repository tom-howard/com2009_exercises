#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav2_msgs.srv import SaveMap

class MapSaverClient(Node):

    def __init__(self):
        super().__init__('map_saver_client')
        
        self.client = self.create_client(
            srv_type=SaveMap, 
            srv_name='/map_saver/save_map'
        )
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for service..."
            )

    def send_request(self):
        request = SaveMap.Request()

        """
        More info:
        https://docs.nav2.org/configuration/packages/configuring-map-server.html
        https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/README.md#services
        """

        request.map_topic = 'map'
        request.map_url = 'my/amazing/map' # relative to home directory
        request.image_format = 'png'
        request.map_mode = 'trinary'
        # request.free_thresh = 
        # request.occupied_thresh = 
        
        return self.client.call_async(request)

def main():
    rclpy.init()
    client = MapSaverClient()

    client.get_logger().info(
        f"Sending the request..."
    )
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    
    client.get_logger().info(
        f"The server has responded:\n"
        f" - result: {response.result}."
    )
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()