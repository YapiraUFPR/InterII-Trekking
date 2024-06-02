from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(package='drivers', launch='drivers_launch.py'),
        
        Node(
            package='visualizer',
            executable='visualization_node',
        ),
        Node(
            package='controller_node',
            executable='steering_node',
        ),
    ])