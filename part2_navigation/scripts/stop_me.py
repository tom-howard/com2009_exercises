#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import TwistStamped 

import time

class Stop(Node):

    def __init__(self):
        super().__init__("stop_me")

        self.vel_pub = self.create_publisher(
            msg_type=TwistStamped,
            topic="cmd_vel",
            qos_profile=10,
        )

        self.stopped = Future()
        self.shutdown()

    def shutdown(self):
        for i in range(5):
            self.vel_pub.publish(TwistStamped())
            time.sleep(0.1)
        self.stopped.set_result('stopped')

def main(args=None):
    rclpy.init(args=args)
    node = Stop()
    rclpy.spin_until_future_complete(node, node.stopped)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()