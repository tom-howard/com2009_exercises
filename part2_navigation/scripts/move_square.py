#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 

from part2_navigation.odometry import to_euler

from math import sqrt, pow, pi 

class Square(Node):

    def __init__(self):
        super().__init__("move_square")

        self.first_message = False
        self.turn = False 
        
        self.pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )

        self.vel_msg = Twist()

        self.sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        # wait until the first odom message has been received before proceeding:
        # while not self.first_message:
        #     continue
        
        run_rate = 10 # hz
        self.timer = self.create_timer(
            timer_period_sec=1/run_rate,
            callback=self.timer_callback,
        )

        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.xref = 0.0; self.yref = 0.0; self.theta_zref = 0.0
        self.yaw = 0.0 # a variable to keep track of how far the robot has turned
        self.displacement = 0.0 # a variable to keep track of how far the robot has moved
             
        self.stopped = False
        
        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )

    def on_shutdown(self):
        print("Stopping the robot...")
        self.msg = Twist()
        self.pub.publish(self.msg)
        self.stopped = True

    def odom_callback(self, msg_data: Odometry):
        pose = msg_data.pose.pose 
        position = pose.position
        orientation = pose.orientation

        self.x = position.x 
        self.y = position.y

        (roll, pitch, yaw) = to_euler(orientation)

        self.theta_z = abs(yaw) # abs(yaw) makes life much easier!!

        if not self.first_message: 
            self.first_message = True
            self.xref = self.x
            self.yref = self.y
            self.theta_zref = self.theta_z

    def timer_callback(self):
        if self.turn:
            # turn by 90 degrees...
            # keep track of how much yaw has been accrued during the current turn
            self.yaw = self.yaw + abs(self.theta_z - self.theta_zref)
            self.theta_zref = self.theta_z
            if self.yaw >= pi/2:
                # That's enough, stop turning!
                self.vel_msg = Twist()
                self.turn = False
                self.yaw = 0.0
                self.xref = self.x
                self.yref = self.y
            else:
                # Not there yet, keep going:
                self.vel_msg.angular.z = 0.3
        else:
            # move forwards by 1m...
            # keep track of how much displacement has been accrued so far
            # (Note: Euclidean Distance)
            self.displacement = self.displacement + sqrt(pow(self.x-self.xref, 2) + pow(self.y-self.yref, 2))
            self.xref = self.x
            self.yref = self.y
            if self.displacement >= 1:
                # That's enough, stop moving!
                self.vel_msg = Twist()
                self.turn = True
                self.displacement = 0.0
                self.theta_zref = self.theta_z
            else:
                # Not there yet, keep going:
                self.vel_msg.linear.x = 0.1

        # publish whatever velocity command has been set above:
        self.pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    move_square = Square()
    try:
        rclpy.spin(move_square)
    except KeyboardInterrupt:
        print("Shutdown requested with Ctrl+C")
    finally:
        move_square.on_shutdown()
        while not move_square.stopped:
            continue
        move_square.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
