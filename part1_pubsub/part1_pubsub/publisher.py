# A simple ROS publisher node in Python

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 

class PublisherNode(Node): 

    def __init__(self):
        node_name = "simple_publisher"
        super().__init__(node_name) 
        topic_name = "chatter" 

        self.publisher = self.create_publisher(String, topic_name, 10) 
        
        node_rate = 10 # hz
        self.timer = self.create_timer(1/node_rate, self.timer_callback)

        self.counter = 0

        self.get_logger().info(f"The '{node_name}' node is active.")

    def timer_callback(self):
        msg =  f"Publisher loop iteration: {self.counter}"
        topic_msg = String()
        topic_msg.data = msg
        self.publisher.publish(topic_msg)
        self.counter += 1

def main():
    rclpy.init()
    
    node = PublisherNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
