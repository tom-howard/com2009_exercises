#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 
from tf.transformations import euler_from_quaternion 
from math import sqrt, pow, pi 

class Square():
    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose 
        position = pose.position
        orientation = pose.orientation

        pos_x = position.x 
        pos_y = position.y

        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        ) 

        self.x = pos_x 
        self.y = pos_y
        self.theta_z = abs(yaw) ## NOTE: abs(yaw) makes life much easier!!

        if not self.first_message: 
            self.first_message = True
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        node_name = "square_dance"
        self.first_message = False
        self.turn = False 
        
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.vel = Twist() 

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the '{node_name}' node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist()) 
        self.ctrl_c = True

    def main(self):
        yaw = 0.0 # a variable to keep track of how far the robot has yawed
        displacement = 0.0 # a variable to keep track of how far the robot has moved
        
        # wait until the first odom message has been recieved before moving:
        while not self.first_message:
            continue
        
        # where the action happens:
        while not self.ctrl_c:
            if self.turn:
                # turn by 90 degrees...
                # keep track of how much yaw has been accrued during the current turn
                yaw = yaw + abs(self.theta_z - self.theta_z0)
                self.theta_z0 = self.theta_z
                if yaw >= pi/2:
                    # That's enough, stop turning!
                    self.vel = Twist()
                    self.turn = False
                    yaw = 0.0
                    self.x0 = self.x
                    self.y0 = self.y
                else:
                    # Not there yet, keep going:
                    self.vel.angular.z = 0.3
            else:
                # move forwards by 1m...
                # keep track of how much displacement has been accrued so far
                # (Note: Euclidean Distance)
                displacement = displacement + sqrt(pow(self.x-self.x0, 2) + pow(self.y-self.y0, 2))
                self.x0 = self.x
                self.y0 = self.y
                if displacement >= 1:
                    # That's enough, stop moving!
                    self.vel = Twist()
                    self.turn = True
                    displacement = 0.0
                    self.theta_z0 = self.theta_z
                else:
                    # Not there yet, keep going:
                    self.vel.linear.x = 0.1

            # publish whatever velocity command has been set above:
            self.pub.publish(self.vel)
            self.rate.sleep() # maintain the loop rate @ 10 hz

if __name__ == "__main__":
    node = Square()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass