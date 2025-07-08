#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from math import atan2

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
        
        yaw = self.quaternion_to_euler(pose.orientation)

        if self.counter > 10:
            self.counter = 0
            self.get_logger().info(
                f"x = {pos_x:.3f} (m), y = {pos_y:.3f} (m), yaw = {yaw:.2f} (radians)"
            )
        else:
            self.counter += 1
    
    def quaternion_to_euler(self, orientation: Quaternion):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        a = +2.0 * (w * z + x * y)
        b = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(a, b)

        return yaw # (in radians)

def main(args=None): 
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()