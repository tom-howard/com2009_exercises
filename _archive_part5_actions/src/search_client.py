#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib

from tuos_msgs.msg import SearchAction, SearchGoal, SearchFeedback, SearchResult

class SearchActionClient():
    goal = SearchGoal()
    msg_counter = 0

    def feedback_callback(self, feedback_data: SearchFeedback):
        ## Get the current distance travelled, from the feedback message
        ## and assign this to a class variable [DONE]
        self.distance = feedback_data.current_distance_travelled
        # print some info to the terminal every 5 messages
        if self.msg_counter > 5:
            print(f"FEEDBACK: distance travelled = {self.distance:.3f} meters.")
            self.msg_counter = 0
        else:
            self.msg_counter += 1

    def __init__(self):
        self.distance = 0.0

        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)

        ## Setup a "simple action client" with a callback function
        ## and wait for the server to be available [DONE]
        self.client = actionlib.SimpleActionClient(
            "/toms_search",
            SearchAction
        )
        self.client.wait_for_server()
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            ## Cancel the goal request, if this node is shutdown
            ## before the action has completed [DONE]
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")

        ## Print the result here [DONE]
        rospy.sleep(1) # wait for the result to come in
        result: SearchResult = self.client.get_result()
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * total_distance_travelled = {result.total_distance_travelled:.3f} meters")
        print(f"  * closest_object_distance = {result.closest_object_distance:.3f} meters")
        print(f"  * closest_object_angle = {result.closest_object_angle:.1f} degrees.")

    def main(self):
        ## Assign values to all goal parameters
        ## and send the goal to the action server [DONE]
        self.goal.approach_distance = 0.4 # meters
        self.goal.fwd_velocity = 0.1 # m/s
        self.client.send_goal(
            self.goal, feedback_cb=self.feedback_callback
        )

        while self.client.get_state() < 2:
            ## Construct an if statement and cancel the goal if the 
            ## distance travelled exceeds 2 meters [DONE]
            if self.distance > 2:
                print("STOP: Distance exceeded 2 meters!!!")
                # break out of the while loop to stop the node:
                break

            self.rate.sleep()

        self.action_complete = True if self.client.get_state() == 3 else False

if __name__ == '__main__':
    ## Instantiate the node and call the main() method from it [DONE]
    node = SearchActionClient()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass