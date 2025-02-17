#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.signals import SignalHandlerOptions

# Import all the necessary ROS message types:
from part5_actions.action import ExploreForward

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Import some other useful Python Modules
from math import sqrt, pow
import numpy as np

class ExploreForwardServer(Node):
        
    def __init__(self):
        super().__init__("explore_forward_server_node")

        self.posx = 0.0
        self.posy = 0.0
        self.lidar_reading = 0.0
        self.await_odom = True
        self.await_lidar = True
        
        self._loop_rate = self.create_rate(
            frequency=5, 
            clock=self.get_clock()
        )

        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )
        self.vel_pub.publish(Twist())

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic='odom',
            callback=self.odom_callback,
            qos_profile=10
        )

        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic='scan',
            callback=self.lidar_callback,
            qos_profile=10
        )

        self.actionserver = ActionServer(
            node=self, 
            action_type=ExploreForward,
            action_name="explore_forward",
            execute_callback=self.server_execution_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.shutdown = False

    def odom_callback(self, odom_msg: Odometry):
        self.posx = odom_msg.pose.pose.position.x
        self.posy = odom_msg.pose.pose.position.y
        self.await_odom = False
    
    def lidar_callback(self, scan_msg: LaserScan):
        left = scan_msg.ranges[0:21]
        right = scan_msg.ranges[-20:] 
        front = np.array(left + right) 

        valid_data = front[front != float("inf")] 
        if np.shape(valid_data)[0] > 0:
            self.lidar_reading = valid_data.mean() 
        else:
            self.lidar_reading = float("nan")
            self.get_logger().warning(
                "No lidar reading."
            )
        self.await_lidar = False
        
    def goal_callback(self, goal: ExploreForward.Goal):
        goal_ok = True
        if goal.fwd_velocity > 0.26 or goal.fwd_velocity < 0:
            self.get_logger().warn(
                "Invalid Velocity!"
            )
            goal_ok = False
        
        if goal.stopping_distance < 0.2:
            self.get_logger().warn(
                "Invalid distance!"
            )
            goal_ok = False
        
        return GoalResponse.ACCEPT if goal_ok else GoalResponse.REJECT
    
    def cancel_callback(self, goal):
        self.get_logger().info('Received a cancel request...')
        return CancelResponse.ACCEPT

    def on_shutdown(self):
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True

    def server_execution_callback(self, goal):
        result = ExploreForward.Result()
        feedback = ExploreForward.Feedback()
        fwd_vel = goal.request.fwd_velocity
        stop_dist = goal.request.stopping_distance

        self.get_logger().info(
            f"\n#####\n"
            f"The '{self.get_name()}' has been called.\n"
            f"Goal:\n"
            f"  - explore at {fwd_vel:.2f} m/s\n"
            f"  - stop {stop_dist:.2f} m in front of something\n" 
            f"Here we go..."
            f"\n#####\n")
        
        # set the robot's velocity:
        vel_cmd = Twist()
        vel_cmd.linear.x = fwd_vel

        # Get the robot's current position:
        while self.await_odom or self.await_lidar:
            continue
        ref_posx = self.posx
        ref_posy = self.posy
        dist_travelled = 0.0
        
        while (self.lidar_reading > stop_dist) and (self.lidar_reading != float("nan")):
            
            self.vel_pub.publish(vel_cmd)
            
            # check if there has been a request to cancel the action:
            if goal.is_cancel_requested:
                # stop the robot:
                for i in range(5):
                    self.vel_pub.publish(Twist())
                goal.canceled()
                self.get_logger().info(
                    f"Cancelled."
                )
                result.total_distance_travelled = dist_travelled
                result.closest_obstacle = float(self.lidar_reading)
                return result
            
            dist_travelled = sqrt(pow(self.posx-ref_posx, 2) + pow(self.posy-ref_posy, 2))
            
            feedback.current_distance_travelled = dist_travelled
            goal.publish_feedback(feedback)

            self._loop_rate.sleep()
                
        for i in range(5):
            self.vel_pub.publish(Twist())

        self.get_logger().info(
            f"{self.get_name()} complete."
        )
        goal.succeed()
        return result

def main(args=None):
    rclpy.init(args=args,
        signal_handler_options=SignalHandlerOptions.NO)
    node = ExploreForwardServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info(
            "Starting the Server (shut down with Ctrl+C)"
        )
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            "Server shut down with Ctrl+C"
        )
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()