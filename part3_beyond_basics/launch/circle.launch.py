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
                    FindPackageShare("turtlebot3_gazebo"), 
                    "launch", 
                    "empty_world.launch.py" 
                ])
            )
        ),
        Node( 
            package='part3_beyond_basics_jazzy', 
            executable='param_circle.py', 
            name='my_param_circle'
        )
    ])