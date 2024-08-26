#!/usr/bin/env python3
# A simple ROS subscriber node in Python

import rospy 
from std_msgs.msg import Float64

class Subscriber(): 

    def callback(self, topic_message: Float64): 
        print(f"The rospy time is: '{topic_message.data}'")

    def __init__(self): 
        self.node_name = "simple_subscriber"
        topic_name = "chatter"

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber(topic_name, Float64, self.callback)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def main(self):
        rospy.spin() 

if __name__ == '__main__': 
    node = Subscriber()
    node.main()