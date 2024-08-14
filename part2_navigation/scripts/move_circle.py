#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Circle(Node):

    def __init__(self, linear_velocity, angular_velocity):
        super().__init__("move_circle")

        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.msg = msg

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        rate = 10  # Hz
        self.timer = self.create_timer(1/rate, self.timer_callback)

        self.shutdown = False
    
    def on_shutdown(self):
        print("Stopping the robot...")
        self.msg = Twist() # By default, velocities within the Twist class are zero
        self.publisher.publish(self.msg)
        self.shutdown = True

    def timer_callback(self):
        self.publisher.publish(self.msg)

def main(args=None):
    radius = 0.5 # meters
    linear_velocity = 0.23 # m/s
    angular_velocity = linear_velocity / radius
        
    # Has an valid velocity been calculated??
    if abs(angular_velocity) > 1.82 or abs(linear_velocity) > 0.26:
        print(f"Not a valid velocity command!")
    else:
        print(
            f"Launching the Move Circle Node with:\n"
            f"    linear = {linear_velocity:.2f} m/s\n"
            f"    angular = {angular_velocity:.3f} rad/s")
        rclpy.init(
            args=args,
            signal_handler_options=rclpy.signals.SignalHandlerOptions.NO
        )
        move_circle = Circle(linear_velocity, angular_velocity)
        try:
            rclpy.spin(move_circle)
        except KeyboardInterrupt:
            print("Shutdown requested with Ctrl+C")
        finally:
            move_circle.on_shutdown()
            
            while not move_circle.shutdown:
                continue

            move_circle.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
