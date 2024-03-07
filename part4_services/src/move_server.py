#!/usr/bin/env python3 

import rospy
from geometry_msgs.msg import Twist 
from tuos_msgs.srv import SetBool, SetBoolResponse 

class moveService():

    def __init__(self):
        service_name = "move_service"
        rospy.init_node(f"{service_name}_server") 

        self.service = rospy.Service(service_name, SetBool, self.srv_callback) 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 

        rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 

    def srv_callback(self, request_from_client): 
        vel = Twist()
        response_from_server = SetBoolResponse() 

        if request_from_client.request_signal == True: 
            print(f"Server received a 'true' request and the robot will now move for 5 seconds...") 

            StartTime = rospy.get_rostime() 

            vel.linear.x = 0.1
            self.pub.publish(vel) 

            rospy.loginfo('Published the velocity command to /cmd_vel')
            while (rospy.get_rostime().secs - StartTime.secs) < 5: 
                continue

            rospy.loginfo('5 seconds have elapsed, stopping the robot...')

            vel.linear.x = 0.0
            self.pub.publish(vel) 

            response_from_server.response_signal = True 
            response_from_server.response_message = "Request complete."
        else: 
            response_from_server.response_signal = False
            response_from_server.response_message = "Nothing happened, set request_signal to 'true' next time."
        return response_from_server

    def main(self):
        rospy.spin() 

if __name__ == '__main__':
    server = moveService()
    server.main()