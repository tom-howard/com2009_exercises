#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib
from geometry_msgs.msg import Twist
from tuos_msgs.msg import SearchAction, SearchGoal, SearchFeedback, SearchResult

class SearchActionClient():
    msg_counter = 0
    vel = Twist()
    vel.angular.z = 0.5 #rad/s

    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        # print every 1 in 5 feedback messages to the terminal
        if self.msg_counter > 5:
            print(f"FEEDBACK: distance travelled = {self.distance:.3f} meters.")
            self.msg_counter = 0
        else:
            self.msg_counter += 1

    def __init__(self):
        self.distance = 0.0
        self.startup = True
        self.stop = False

        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(5)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.client = actionlib.SimpleActionClient(
            "/toms_search",
            SearchAction
        )
        self.client.wait_for_server()
        rospy.on_shutdown(self.shutdownhook)
        self.startup = False

    def shutdownhook(self):
        
        if self.client.get_state() < 2:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            ## Cancel the current goal request 
            self.client.cancel_goal()
            while self.client.get_state() < 2:
                continue
            rospy.logwarn("Goal Cancelled...")
            self.get_result()
        
        self.pub.publish(Twist())
        self.stop = True

    def get_result(self):
        result: SearchResult = self.client.get_result()
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * total_distance_travelled = {result.total_distance_travelled:.3f} meters")
        print(f"  * closest_object_distance = {result.closest_object_distance:.3f} meters")
        print(f"  * closest_object_angle = {result.closest_object_angle:.1f} degrees.")        

    def main(self):
        while self.startup:
            continue

        while not self.stop:
            ## Send a goal to the action server 
            goal = SearchGoal()
            goal.approach_distance = 0.6 # meters
            goal.fwd_velocity = 0.1 # m/s
            self.client.send_goal(
                goal, feedback_cb=self.feedback_callback
            )
            # monitor action status
            while self.client.get_state() < 2 and not self.stop:
                distance_limit = 0.8
                ## Cancel the goal if distance exceeds `distance_limit` 
                if self.distance > distance_limit:
                    print(f"STOP: Distance exceeded {distance_limit:.2f} meters!!!")
                    self.client.cancel_goal()
                    self.distance = 0.0
                    break
                
                self.rate.sleep()

            if self.client.get_state() == 3:
                self.get_result()

            timestamp = rospy.get_time()
            while (rospy.get_time() - timestamp) < 5 and not self.stop:
                print(f"TURNING: {5 - (rospy.get_time() - timestamp):.1f} second(s) to go...")
                self.pub.publish(self.vel)
                self.rate.sleep()
            
            self.pub.publish(Twist())

if __name__ == '__main__':
    node = SearchActionClient()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass