#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist

class Circle(Node):
    
    def __init__(self):
        super().__init__("move_circle")
        
        self.my_publisher = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )

        publish_rate = 10 # Hz
        self.timer = self.create_timer(
            1/publish_rate,
            self.timer_callback
        )
                
        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )

        self.shutdown = False
    
    def on_shutdown(self):
        print("Stopping the robot...")
        self.my_publisher.publish(Twist())
        self.shutdown = True

    def timer_callback(self):
        radius = 0.5 # [m]
        linear_velocity = 0.1 # [m/s]
        angular_velocity = linear_velocity / radius

        topic_msg = Twist() 
        topic_msg.linear.x = linear_velocity
        topic_msg.angular.z = angular_velocity
        self.my_publisher.publish(topic_msg)
        
        self.get_logger().info(
            f"Linear Velocity: {topic_msg.linear.x:.2f} [m/s], "
            f"Angular Velocity: {topic_msg.angular.z:.2f} [rad/s].",
            throttle_duration_sec=1,
        )

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    move_circle = Circle()
    try:
        rclpy.spin(move_circle)
    except KeyboardInterrupt:
        print(
            f"{move_circle.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally: 
        move_circle.on_shutdown() 
        while not move_circle.shutdown: 
            continue
        move_circle.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    