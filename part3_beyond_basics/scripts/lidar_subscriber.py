#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import LaserScan

import numpy as np

class LidarSubscriber(Node): 

    def __init__(self): 
        super().__init__("lidar_subscriber")

        self.my_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        )
        
        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")

    def lidar_callback(self, scan_data: LaserScan): 
        left_20_deg = scan_data.ranges[0:21]
        right_20_deg = scan_data.ranges[-20:]
        front = np.array(left_20_deg + right_20_deg)

        front = front[front != float("inf")]
        front_spa = front.mean() if np.shape(front)[0] > 0 else float("nan")

        left = np.array(scan_data.ranges[80:101])
        left = left[left != float("inf")]
        left_spa = left.mean() if np.shape(left)[0] > 0 else float("nan")

        right = np.array(scan_data.ranges[260:281])
        right = right[right != float("inf")]
        right_spa = right.mean() if np.shape(right)[0] > 0 else float("nan")

        self.get_logger().info(
            f"LiDAR Readings:\n"
            f"  Front: {front_spa:.3f} meters\n"
            f"  Left: {left_spa:.3f} meters\n"
            f"  Right: {right_spa:.3f} meters\n",
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