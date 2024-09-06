#!/usr/bin/env python3
# A simple ROS2 Publisher

import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class SimplePublisher(Node):
    
    def __init__(self):
        super().__init__("simple_publisher")
        
        self.my_publisher = self.create_publisher(
            msg_type=String,
            topic="my_topic",
            qos_profile=10,
        )

        publish_rate = 1 # Hz
        self.timer = self.create_timer(1/publish_rate, self.timer_callback)
                
        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")

    def timer_callback(self):
        ros_time = self.get_clock().now().seconds_nanoseconds()

        topic_msg = String()
        topic_msg.data = f"The ROS time is {ros_time[0]} (seconds)."
        self.my_publisher.publish(topic_msg)
        self.get_logger().info(f"Publishing: '{topic_msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    my_simple_publisher = SimplePublisher()
    rclpy.spin(my_simple_publisher)
    my_simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    