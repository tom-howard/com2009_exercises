# COM2009 Assignment #1 Worked Examples

Solutions to the COM2009 [Assignment #1 Course Exercises](https://tom-howard.github.io/com2009/course/assignment1/)

> [!WARNING]
> This is currently incomplete. Solutions are not currently available for all exercises.

## Part 1 (Getting Started with ROS 2)
https://tom-howard.github.io/com2009/course/assignment1/part1/

* Exercise 5 "Creating a publisher node": [The `publisher.py` node](./part1_pubsub/scripts/publisher.py)
* Exercise 6 "Creating a subscriber node": [The `subscriber.py` node](./part1_pubsub/scripts/subscriber.py)
* Exercise 8 "Using a custom ROS Message": [The `publisher.py` node](./part1_pubsub/scripts/custom_msg_publisher.py) and [The `subscriber.py` node](./part1_pubsub/scripts/custom_msg_subscriber.py)

## Part 2: Odometry & Navigation
https://tom-howard.github.io/com2009/course/assignment1/part2/

* Exercise 2 "Creating a Python node to process Odometry data": [The `odom_subscriber.py` node](./part2_navigation/scripts/odom_subscriber.py)
* Exercise 4 & 5 "Creating a Python node to make the robot move in a circle" (with shutdown): [The `move_circle.py` node](./part2_navigation/scripts/move_circle.py)
* Exercise 5 "Making our Robot Follow a Square Motion Path": [the `move_square.py` node](./part2_navigation/scripts/move_square.py)

## Part 3: Beyond the Basics
https://tom-howard.github.io/com2009/course/assignment1/part3/

* Exercise 1 "Creating a Launch File": [the `pubsub.launch.py` launch file](./part3_beyond_basics/launch/pubsub.launch.py)
* Exercise 2 "Launching Another Launch File": [the `circle.launch.py` launch file](./part3_beyond_basics/launch/circle.launch.py)
* Exercise 4 "Building a LaserScan Callback Function": [the `lidar_subscriber.py` node](./part3_beyond_basics/scripts/lidar_subscriber.py)

## Part 4: Services
https://tom-howard.github.io/com2009/course/assignment1/part4/

* Exercise 4 "Adapting the Number Game Server": [the `my_number_game.py` Service server](./part4_services/scripts/my_number_game.py)
* Exercise 5 "Creating a Python Service Client": [the `number_game_client.py` Service client](./part4_services/scripts/number_game_client.py)
* Exercise 6 "Developing A Map Saver Service Client": [the `map_saver_client.py` node](./part4_services/scripts/map_saver_client.py)

## Part 5: Actions
https://tom-howard.github.io/com2009/course/assignment1/part5/

* Exercise 2 "Building a Python Action Client Node": [`camera_sweep_action_client.py`](./part5_actions/scripts/camera_sweep_action_client_part4.py)
* Exercise 4 "Building the "ExploreForward" Action Server": [the `explore_server.py` Action server](./part5_actions/scripts/explore_server.py)

## Part 6: Cameras, Machine Vision & OpenCV
https://tom-howard.github.io/com2009/course/assignment1/part6/

* Exercise 2 "Object Detection": [the `object_detection.py` node](./part6_vision/scripts/object_detection.py)