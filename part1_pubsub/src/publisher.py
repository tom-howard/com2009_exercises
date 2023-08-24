#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from std_msgs.msg import String 

class PublisherNode(): 

    def __init__(self): 
        self.node_name = "simple_publisher" 
        topic_name = "chatter" 

        self.publisher = rospy.Publisher(topic_name, String, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 

        self.stop = False 
        rospy.on_shutdown(self.shutdownprocesses) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownprocesses(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.stop = True

def main():
    node = PublisherNode()
    
    while not node.stop: 
        message = f"rospy time is: {rospy.get_time()}"
        node.publisher.publish(message)
        node.rate.sleep()

if __name__ == '__main__': 
    main()
