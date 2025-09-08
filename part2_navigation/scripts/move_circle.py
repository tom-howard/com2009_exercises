#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped

class Circle(Node): 

    def __init__(self):
        super().__init__("move_circle") 

        self.my_publisher = self.create_publisher(
            msg_type=TwistStamped,
            topic="/cmd_vel",
            qos_profile=10,
        ) 

        publish_rate = 1 # Hz
        self.timer = self.create_timer(
            timer_period_sec=1/publish_rate, 
            callback=self.timer_callback
        ) 

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised." 
        )

        self.shutdown = False

    def on_shutdown(self):
        self.get_logger().info(
            "Stopping the robot..."
        )
        self.my_publisher.publish(TwistStamped()) 
        self.shutdown = True
    
    def timer_callback(self):
        radius = 0.5 # meters
        linear_velocity = 0.1 # meters per second [m/s]
        angular_velocity = linear_velocity / radius  

        topic_msg = TwistStamped() 
        topic_msg.twist.linear.x = linear_velocity
        topic_msg.twist.angular.z = angular_velocity
        self.my_publisher.publish(topic_msg) 

        self.get_logger().info( 
            f"Linear Velocity: {topic_msg.twist.linear.x:.2f} [m/s], "
            f"Angular Velocity: {topic_msg.twist.angular.z:.2f} [rad/s].",
            throttle_duration_sec=1, 
        )

def main(args=None): 
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
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