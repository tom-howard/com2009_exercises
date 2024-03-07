#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from std_msgs.msg import Float64 

class Publisher(): 

    def __init__(self): 
        self.node_name = "simple_publisher" 
        topic_name = "chatter" 

        self.pub = rospy.Publisher(topic_name, Float64, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main(self):
        while not self.ctrl_c: 
            publisher_message = Float64()
            publisher_message.data = rospy.get_time()
            self.pub.publish(publisher_message)
            self.rate.sleep()

if __name__ == '__main__': 
    node = Publisher() 
    node.main()