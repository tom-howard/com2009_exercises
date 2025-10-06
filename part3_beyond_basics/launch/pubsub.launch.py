from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='part1_pubsub_jazzy', 
            executable='publisher.py', 
            name='my_publisher' 
        ),
        Node( 
            package='part1_pubsub_jazzy', 
            executable='subscriber.py', 
            name='my_subscriber' 
        )
    ])