#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# import the Odometry message from the nav_msgs package:
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from math import atan2, asin

def euler_from_quaternion(quaternions: Quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    adapted from:
    https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    x = quaternions.x
    y = quaternions.y
    z = quaternions.z
    w = quaternions.w
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)
    
    return roll, pitch, yaw # in radians

class OdomSubscriber(Node):

    def __init__(self):
        # Initialise the node with a name (we can call it anything we like)
        super().__init__("odom_subscriber") 
        
        # When setting up the subscriber, the "odom" topic needs to be specified
        # and the message type (Odometry) needs to be provided
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription

        self.counter = 0 # What's this bit for, do you think?

    def odom_callback(self, topic_data: Odometry):
        # We're only interested in the pose part of the Odometry message,
        # so we'll extract this bit first:
        pose = topic_data.pose.pose
        # This contains information about both the "position" and "orientation"
        # of the robot, so let's extract those two parts out next:
        position = pose.position
        orientation = pose.orientation 
        # "position" data is provided in meters, so we don't need to do any
        # conversion on this, and can extract the relevant parts of this directly:
        pos_x = position.x
        pos_y = position.y
        pos_z = position.z
        # "orientation" data is in quaternions, so we need to convert this 
        # using the "euler_from_quaternion" function above 
        
        roll, pitch, yaw = euler_from_quaternion(orientation)

        # Here we print out the values that we're interested in:
        if self.counter > 10:
            self.counter = 0
            print(f"x = {pos_x:.3f} (m), y = {pos_y:.3f} (m), theta_z = {yaw:.3f} (radians)")
        else:
            self.counter += 1

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    odom_subscriber = OdomSubscriber()
    try:
        rclpy.spin(odom_subscriber)
    except KeyboardInterrupt:
        pass

    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()