from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("tuos_simulations"),
                    "launch", 
                    "waffle.launch.py" 
                ])
            ),
            launch_arguments={
                'x_pose': '1.0',
                'y_pose': '0.5'
            }.items()
        )
    ])