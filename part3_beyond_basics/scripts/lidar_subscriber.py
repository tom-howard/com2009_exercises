#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions 

from sensor_msgs.msg import LaserScan 

import numpy as np 

class LidarSubscriber(Node): 

    def __init__(self): 
        super().__init__("lidar_subscriber")

        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        ) 

        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")

    def lidar_callback(self, scan_data: LaserScan): 
        front_left = scan_data.ranges[0:21] 
        front_right = scan_data.ranges[-20:] 
        front = np.array(front_right + front_left) 

        front = front[front != float("inf")] 
        front_single_point_ave = front.mean() if np.shape(front)[0] > 0 else float("nan")

        left = np.array(scan_data.ranges[80:101])
        left = left[left != float("inf")]
        left_single_point_ave = left.mean() if np.shape(left)[0] > 0 else float("nan")

        right = np.array(scan_data.ranges[260:281])
        right = right[right != float("inf")]
        right_single_point_ave = right.mean() if np.shape(right)[0] > 0 else float("nan")

        self.get_logger().info(
            f"LiDAR Readings:\n"
            f"  Front: {front_single_point_ave:.3f} meters\n"
            f"  Left: {left_single_point_ave:.3f} meters\n"
            f"  Right: {right_single_point_ave:.3f} meters\n",
            throttle_duration_sec = 1,
        ) 

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown request (Ctrl+C) detected...")
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
