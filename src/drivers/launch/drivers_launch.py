from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drivers',
            executable='camera_publisher',
        ),
        Node(
            package='drivers',
            executable='led_listener',
        ),
        Node(
            package='drivers',
            executable='bno_publisher.py',
        ),
        Node(
            package='drivers',
            executable='vl5_publisher.py',
        ),
        Node(
            package='drivers',
            executable='ina_publisher.py',
        ),
        Node(
            package='drivers',
            executable='tcs32_publisher.py',
        ),
        # Node(
        #     package='drivers',
        #     executable='motor_listener',
        # ),
    ])