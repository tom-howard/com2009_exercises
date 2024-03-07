#!/usr/bin/env python3 

import rospy
from geometry_msgs.msg import Twist 
from tuos_msgs.srv import Approach, ApproachResponse 
from sensor_msgs.msg import LaserScan
import numpy as np

class moveService():

    def __init__(self):
        service_name = "approach_service"
        rospy.init_node(f"{service_name}_server") 

        self.service = rospy.Service(service_name, Approach, self.srv_callback) 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback) 

        rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 

    def scan_callback(self, scan_data: LaserScan):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.ave_distance = front_arc.mean()

        # Optional Extra:
        arc_angles = np.arange(-20, 21)
        self.object_angle = arc_angles[np.argmin(front_arc)]

    def srv_callback(self, request_from_client): 
        vel = Twist()
        response_from_server = ApproachResponse() 

        fwd_vel = request_from_client.approach_velocity
        stop_dist = request_from_client.approach_distance

        vel.linear.x = fwd_vel
        self.pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while self.ave_distance > stop_dist: 
            self.pub.publish(vel)
            continue
        
        rospy.loginfo('Stopping the robot...')

        vel.linear.x = 0.0
        self.pub.publish(vel) 

        response_from_server.response_message = "The service has now completed." 

        return response_from_server
        
    def main(self):
        rospy.spin() 

if __name__ == '__main__':
    server = moveService()
    server.main()