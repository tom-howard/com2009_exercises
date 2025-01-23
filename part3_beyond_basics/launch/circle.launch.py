import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("turtlebot3_gazebo"),
                    "launch",
                    "empty_world.launch.py"
                )
            )
        ),
        Node(
            package='part2_navigation',
            executable='move_circle.py',
            name='move_circ'
        )
    ])