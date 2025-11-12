from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration 

def generate_launch_description(): 
    return LaunchDescription([ 
        DeclareLaunchArgument(
            name='circle_radius', 
            description="Sets the desired radius of the circle (in meters).",
            default_value='1.0'
        ),
        Node( 
            package='part3_beyond_basics_jazzy', 
            executable='param_circle.py', 
            name='my_param_circle_node',
            parameters=[{'radius': LaunchConfiguration('circle_radius')}] 
        )
    ])