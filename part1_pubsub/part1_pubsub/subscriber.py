# A simple ROS subscriber node in Python

import rclpy
from rclpy.node import Node 
from std_msgs.msg import String

class SubscriberNode(Node): 

    def __init__(self):
        node_name = "simple_subscriber"
        super().__init__(node_name)
        topic_name = "chatter"

        self.subscriber = self.create_subscription(
            String,
            topic_name,
            self.subscriber_callback,
            10
        )
        self.subscriber # prevent unused variable warning

    def subscriber_callback(self, topic_msg: String):
        self.get_logger().info(f"The subscriber heard: '{topic_msg.data}'")

def main():
    rclpy.init()
    node = SubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()
