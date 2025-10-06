#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
import time

state = 1
vel = TwistStamped()

rclpy.init(args=None)
node = rclpy.create_node("basic_velocity_control")
vel_pub = node.create_publisher(TwistStamped, "cmd_vel", 10)

timestamp = node.get_clock().now().nanoseconds

while rclpy.ok():
    time_now = node.get_clock().now().nanoseconds
    elapsed_time = (time_now - timestamp) * 1e-9
    if state == 1: 
        if elapsed_time < 2:
            vel.twist.linear.x = 0.05
            vel.twist.angular.z = 0.0
        else:
            vel.twist.linear.x = 0.0
            vel.twist.angular.z = 0.0
            state = 2
            timestamp = node.get_clock().now().nanoseconds
    elif state == 2:
        if elapsed_time < 4:
            vel.twist.linear.x = 0.0
            vel.twist.angular.z = 0.2
        else:
            vel.twist.linear.x = 0.0
            vel.twist.angular.z = 0.0 
            state = 1
            timestamp = node.get_clock().now().nanoseconds

    node.get_logger().info(
        f"\n[State = {state}] Publishing velocities:\n"
        f"  - linear.x: {vel.twist.linear.x:.2f} [m/s]\n"
        f"  - angular.z: {vel.twist.angular.z:.2f} [rad/s].",
        throttle_duration_sec=1,
    )
    vel_pub.publish(vel)
    
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1) # 10Hz loop rate
    except KeyboardInterrupt:
        print("Ctrl+C detected. Shutting down.")
        break

node.destroy_node()