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
        return self.actionclient.send_goal_async(goal)

def main(args=None): 
    rclpy.init(args=args)
    action_client = CameraSweepActionClient()
    future = action_client.send_goal()
    rclpy.spin_until_future_complete(action_client, future)

if __name__ == '__main__':
    main()