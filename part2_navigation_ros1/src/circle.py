#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist 

class Circle(): 

    def __init__(self): 
        self.node_name = "circle" 
        topic_name = "cmd_vel" 
        self.vel_msg = Twist()

        self.publisher = rospy.Publisher(topic_name, Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 

        self.stop = False 
        rospy.on_shutdown(self.shutdownprocesses) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownprocesses(self): 
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.publisher.publish(self.vel_msg)
        self.stop = True

def main():
    node = Circle()

    while not node.stop: 
        node.vel_msg.linear.x = 0.1
        radius = 0.5 # meters
        node.vel_msg.angular.z = node.vel_msg.linear.x / radius
        node.publisher.publish(node.vel_msg)
        node.rate.sleep()

if __name__ == '__main__': 
    try:
        main() 
    except rospy.ROSInterruptException:
        pass
