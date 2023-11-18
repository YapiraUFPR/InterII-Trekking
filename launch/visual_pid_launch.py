from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_drivers',
            executable='camera_publisher',
        ),
        Node(
            package='sensor_drivers',
            executable='color_publisher',
        ),
        Node(
            package='signal',
            executable='led_controller',
        ),
        Node(
            package='visual_pid',
            executable='2d_pid',
        ),
    ])