#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node

from nav_msgs.msg import Odometry

from part2_navigation_modules.tb3_tools import quaternion_to_euler

class OdomSubscriber(Node): 

    def __init__(self): 
        super().__init__("odom_subscriber") 

        self.my_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.msg_callback,
            qos_profile=10,
        ) 

        self.counter = 0

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        ) 

    def msg_callback(self, topic_message: Odometry):

        pose = topic_message.pose.pose
        
        pos_x = pose.position.x
        pos_y = pose.position.y
        
        _, _, yaw = quaternion_to_euler(pose.orientation)

        if self.counter > 10:
            self.counter = 0
            self.get_logger().info(
                f"x = {pos_x:.3f} (m), y = {pos_y:.3f} (m), theta_z = {yaw:.2f} (radians)"
            )
        else:
            self.counter += 1

def main(args=None): 
    rclpy.init(args=args)
    node = OdomSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()