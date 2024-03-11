# COM2009 Assignment #1 Worked Examples

Solutions to the COM2009 [Assignment #1 Course Exercises](https://tom-howard.github.io/ros/com2009/assignment1/)

## Part 1 (ROS & Linux Basics)
https://tom-howard.github.io/ros/com2009/assignment1/part1/

* Exercise 5 "Creating a publisher node": [The `publisher.py` node](./part1_pubsub/src/publisher.py)
* Exercise 6 "Creating a subscriber node": [The `subscriber.py` node](./part1_pubsub/src/subscriber.py)
    * **Exercise 6 Advanced**: [The `publisher.py` (float64) node](./part1_pubsub/src/float64_publisher.py) and [The `subscriber.py` (float64) node](./part1_pubsub/src/float64_subscriber.py)
* Exercise 8 "Creating a launch file": [The `pubsub.launch` file](./part1_pubsub/launch/pubsub.launch)

## Part 2: Odometry & Navigation
https://tom-howard.github.io/ros/com2009/assignment1/part2/

* Exercise 2 "Creating a Python node to process Odometry data": [The `odom_subscriber.py` node](./part2_navigation/src/odom_subscriber.py)
* Exercise 4 "Creating a Python node to make the robot move in a circle": [The `move_circle.py` node](./part2_navigation/src/move_circle.py)
    * **Exercise 4 Advanced**: [Launch file](./part2_navigation/launch/ex4_advanced.launch)
* Exercise 5 "Making your robot follow a Square motion path": [the `move_square.py` node](./part2_navigation/src/move_square.py)
    * **Exercise 5 Advanced (1)**: [Stopping after 2 squares](./part2_navigation/src/move_square_advanced.py)
    * **Exercise 5 Advanced (2)**: [Launch File](./part2_navigation/launch/ex5_advanced.launch)

## Part 4: ROS Services
https://tom-howard.github.io/ros/com2009/assignment1/part4/

* Exercise 1 "Creating a Service Server in Python and calling it from the command-line": [The `move_server.py` node](./part4_services/src/move_server.py)
* Exercise 2 "Creating a Python Service Client Node": [The `move_client.py` node](./part4_services/src/move_client.py)
* Exercise 3 "Making and calling your own Service": [The `timed_move_server.py` node](./part4_services/src/timed_move_server.py)
* Exercise 4 "Approaching an object using a Service and closed-loop control": [The `approach_server.py` node](./part4_services/src/approach_server.py)

## Part 5: ROS Actions
https://tom-howard.github.io/ros/com2009/assignment1/part5/

* Exercise 2 "Building a Python Action Client Node with Concurrency": [The `action_client.py` node](./part5_actions/src/action_client.py) and [the `concurrent_action_client.py` node](./part5_actions/src/concurrent_action_client.py)
* Exercise 3 "Building a Preemptive Python Action Client Node": [The `preemptive_action_client.py` node](./part5_actions/src/preemptive_action_client.py)
* Exercise 4 "Developing an "Obstacle Avoidance" behaviour using an Action Server":
    * [The `search_server.py`](./part5_actions/src/search_server.py)
    * [The `search_client.py`](./part5_actions/src/search_client.py)

## Part 6: Cameras, Machine Vision & OpenCV
https://tom-howard.github.io/ros/com2009/assignment1/part6/

* Exercise 3 "Locating image features using Image Moments": [The `colour_search.py` Node](./part6_vision/src/colour_search.py)
* Exercise 4 "Line Following": [The `line_follower.py` Node](./part6_vision/src/line_follower.py)