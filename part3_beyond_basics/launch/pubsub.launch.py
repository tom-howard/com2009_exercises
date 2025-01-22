from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='part1_pubsub',
            executable='publisher.py',
            name='my_publisher'
        ),
        Node(
            package='part1_pubsub',
            executable='subscriber.py',
            name='my_subscriber'
        )
    ])