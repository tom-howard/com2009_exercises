#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient 

from tuos_interfaces.action import CameraSweep 

class CameraSweepActionClient(Node):

    def __init__(self):
        super().__init__("camera_sweep_action_client") 
        self.actionclient = ActionClient(
            node=self, 
            action_type=CameraSweep, 
            action_name="camera_sweep"
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('goal_images', 0),
                ('goal_angle', 0)
            ]
        )

    def send_goal(self): 
        images = self.get_parameter(
            'goal_images' 
        ).get_parameter_value().integer_value 
        angle = self.get_parameter(
            'goal_angle'
        ).get_parameter_value().integer_value

        goal = CameraSweep.Goal()
        goal.sweep_angle = float(angle)
        goal.image_count = images

        self.actionclient.wait_for_server()

        # send the goal to the action server:
        self.send_goal_future = self.actionclient.send_goal_async(
            goal=goal, 
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("The goal was rejected by the server.")
            return

        self.get_logger().info("The goal was accepted by the server.")

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"The action has completed.\n"
            f"Result (Image Paths):\n  "
            + "\n  ".join(result.image_paths)
        )
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        fdbk_current_angle = feedback.current_angle
        fdbk_current_image = feedback.current_image
        self.get_logger().info(
            f"\nFEEDBACK:\n"
            f"  - Current angular position = {fdbk_current_angle:.1f} degrees.\n"
            f"  - Image(s) captured so far = {fdbk_current_image}."
        )

def main(args=None): 
    rclpy.init(args=args)
    action_client = CameraSweepActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()