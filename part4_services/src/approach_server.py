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
        self.rate = rospy.Rate(10)

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

    from tuos_msgs.srv import ApproachRequest
    def srv_callback(self, request_from_client: ApproachRequest): 
        vel = Twist()
        response_from_server = ApproachResponse() 

        fwd_vel = request_from_client.approach_velocity
        stop_dist = request_from_client.approach_distance

        invalid_input = False
        if fwd_vel > 0.26 or fwd_vel <= 0:
            invalid_input = True
        
        if stop_dist <= 0.2:
            invalid_input = True
        
        if invalid_input:
            response_from_server.response_message = "Invalid Input!!"
        else:
            vel.linear.x = fwd_vel

            rospy.loginfo(f'Initiating motion at {vel.linear.x:.2f} m/s...')
            while self.ave_distance > stop_dist: 
                self.pub.publish(vel)
                self.rate.sleep()               
        
            rospy.loginfo('Object detected: stopping the robot...')
            self.pub.publish(Twist())
            response_from_server.response_message = f"The robot was stopped with an object {self.ave_distance:.2f} m away." 

        return response_from_server
        
    def main(self):
        rospy.spin() 

if __name__ == '__main__':
    server = moveService()
    try:
        server.main()
    except rospy.ROSInterruptException:
        pass