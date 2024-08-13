#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist 

class Circle():

    vel_cmd = Twist()

    def __init__(self):
        self.node_name = "circle_dance"

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self):
        self.vel_cmd = Twist() # By default, velocities within the Twist class are zero
        self.pub.publish(self.vel_cmd)
        self.ctrl_c = True

    def main(self):
        radius = 0.5 # meters
        lin_vel = 0.23 # m/s
        ang_vel = lin_vel / radius
        
        # Has an valid velocity been calculated??
        if abs(ang_vel) > 1.82:
            print(f"Woops: {ang_vel:.3f} rad/s is not a valid velocity command!")
            self.ctrl_c = True
        else:
            print(f"Robot will move with linear velocity = {lin_vel:.2f} m/s and angular velocity = {ang_vel:.3f} rad/s...")

        while not self.ctrl_c:
            self.vel_cmd.linear.x = lin_vel
            self.vel_cmd.angular.z = ang_vel
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    node = Circle() 
    # Use a try-except to catch those annoying "ROSInterruptException" errors on shutdown...
    try:
        node.main() 
    except rospy.ROSInterruptException:
        pass