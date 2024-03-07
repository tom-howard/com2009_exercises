#!/usr/bin/env python3 

import rospy
from geometry_msgs.msg import Twist 
from tuos_msgs.srv import TimedMovement, TimedMovementResponse 

class moveService():

    def __init__(self):
        service_name = "toms_timed_move_service"
        rospy.init_node(f"{service_name}_server") 

        self.service = rospy.Service(service_name, TimedMovement, self.srv_callback) 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 

        rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 

    from tuos_msgs.srv import TimedMovementRequest
    def srv_callback(self, request_from_client: TimedMovementRequest): 
        
        vel = Twist()
        response_from_server = TimedMovementResponse() 
        movement = request_from_client.movement_request
        duration = request_from_client.duration

        invalid_input = False
        if duration <= 0:
            invalid_input = True
        
        if movement == "fwd":
            vel.linear.x = 0.1 # m/s
        elif movement == "back":
            vel.linear.x = -0.1 # m/s
        elif movement == "left":
            vel.angular.z = 0.2 # rad/s
        elif movement == "right":
            vel.angular.z = -0.2 # rad/s
        else:
            invalid_input = True

        if invalid_input: 
            rospy.loginfo("Woops: Invalid Request!") 
            response_from_server.success = False
        else:
            response_from_server.success = True
            StartTime = rospy.get_rostime() 

            self.pub.publish(vel) 

            rospy.loginfo(f"Published a velocity command to /cmd_vel")
            while (rospy.get_rostime().secs - StartTime.secs) < duration: 
                continue

            rospy.loginfo(f'{duration} second(s) have elapsed, stopping the robot...')

            self.pub.publish(Twist()) 

        return response_from_server

    def main(self):
        rospy.spin() 

if __name__ == '__main__':
    server = moveService()
    try:
        server.main()
    except rospy.ROSInterruptException:
        pass