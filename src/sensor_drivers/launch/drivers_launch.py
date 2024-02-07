from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_drivers',
            node_executable='camera_publisher',
            node_name='camera_publisher'
        ),
        Node(
            package='sensor_drivers',
            node_executable='color_publisher',
            node_name='color_publisher'
        ),
        Node(
            package='sensor_drivers',
            node_executable='imu_publisher',
            node_name='imu_publisher'
        ),
        Node(
            package='sensor_drivers',
            node_executable='irdist_publisher',
            node_name='irdist_publisher'
        ),
        Node(
            package='sensor_drivers',
            node_executable='battery_publisher',
            node_name='battery_publisher'
        ),
    ])