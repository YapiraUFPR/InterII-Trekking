from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

launch_args = [
    DeclareLaunchArgument(name="camera_enable", default_value="true", description="enable camera node"),
    DeclareLaunchArgument(name="imu_enable", default_value="true", description="enable IMU node"),
    DeclareLaunchArgument(name="status_led_enable", default_value="true", description="enable led node"),
    DeclareLaunchArgument(name="flare_enable", default_value="false", description="enable flare node"),
    DeclareLaunchArgument(name="dist_sensor_enable", default_value="false", description="enable distance sensor node"),
    DeclareLaunchArgument(name="bat_monitor_enable", default_value="false", description="enable battery monitor node"),
    DeclareLaunchArgument(name="color_sensor_enable", default_value="false", description="enable color sensor node"),
    DeclareLaunchArgument(name="bluetooth_enable", default_value="true", description="enable bluetooth node"),
    DeclareLaunchArgument(name="motor_enable", default_value="true", description="enable motor node"),
]

def launch_setup(context):
    return [
        Node(
            package='drivers',
            condition=IfCondition(LaunchConfiguration("camera_enable")),
            executable='camera_publisher',
        ),
        Node(
            package='drivers',
            condition=IfCondition(LaunchConfiguration("status_led_enable")),
            executable='led_listener',
        ),
        Node(
            package='drivers',
            condition=IfCondition(LaunchConfiguration("flare_enable")),
            executable='flare_listener.py'
        ),
        Node(
            package='drivers',
            condition=IfCondition(LaunchConfiguration("imu_enable")),
            executable='bno_publisher.py',
        ),
        Node(
            package='drivers',
            condition=IfCondition(LaunchConfiguration("dist_sensor_enable")),
            executable='vl5_publisher.py',
        ),
        Node(
            package='drivers',
            condition=IfCondition(LaunchConfiguration("bat_monitor_enable")),
            executable='ina_publisher.py',
        ),
        Node(
            package='drivers',
            condition=IfCondition(LaunchConfiguration("color_sensor_enable")),
            executable='tcs32_publisher.py',
        ),
        Node(
            package='drivers',
            condition=IfCondition(LaunchConfiguration("bluetooth_enable")),
            executable='bluetooth_gamepad.py',
        ),
        Node(
            package='drivers',
            condition=IfCondition(LaunchConfiguration("motor_enable")),
            executable='motor_listener.py',
        ),
    ]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
