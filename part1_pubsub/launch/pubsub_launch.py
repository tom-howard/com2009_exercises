from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='part1_pubsub',
            namespace='my_pub',
            executable='publisher',
        ),
        Node(
            package='part1_pubsub',
            namespace='my_sub',
            executable='subscriber',
            output="screen",
            emulate_tty=True,
        ),
    ])