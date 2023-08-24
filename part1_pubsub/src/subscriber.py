#!/usr/bin/env python3
# A simple ROS subscriber node in Python

import rospy 
from std_msgs.msg import String

class SubscriberNode(): 

    def subscriber_callback(self, topic_message): 
        print(f"The '{self.node_name}' node obtained the following message: '{topic_message.data}'")

    def __init__(self): 
        self.node_name = "simple_subscriber"
        topic_name = "chatter"

        rospy.init_node(self.node_name, anonymous=True)
        self.subscriber = rospy.Subscriber(topic_name, String, self.subscriber_callback)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

def main():
    node = SubscriberNode()
    rospy.spin() 

if __name__ == '__main__': 
    main()
