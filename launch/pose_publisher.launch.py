from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy2pose', 
            executable='pose_publisher_node',  
            name='pose_publisher',    
            output='screen',              
        ),
        Node(
            package='joy',  
            executable='joy_node', 
            name='joy_node',   
            output='screen',            
        )
    ])
