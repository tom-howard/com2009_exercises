#!/usr/bin/env python3
# A simple ROS2 Subscriber

import rclpy 
from rclpy.node import Node

from part1_pubsub.msg import Example

class SimpleSubscriber(Node): 

    def __init__(self): 
        super().__init__("simple_subscriber") 

        self.my_subscriber = self.create_subscription(
            msg_type=Example,
            topic="my_topic",
            callback=self.msg_callback,
            qos_profile=10,
        ) 

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        ) 

    def msg_callback(self, topic_message: Example): 
        
        self.get_logger().info(f"The '{self.get_name()}' node heard:") 
        self.get_logger().info(f"  '{topic_message.info}'")
        self.get_logger().info(f"  '{topic_message.time:.0f}'")

def main(args=None): 
    rclpy.init(args=args)
    my_simple_subscriber = SimpleSubscriber()
    rclpy.spin(my_simple_subscriber)
    my_simple_subscriber.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()