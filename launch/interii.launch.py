from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, AndSubstitution, NotSubstitution
from launch_ros.actions import Node

launch_args = [
    DeclareLaunchArgument(name="map", default_value="false", description="Run in mapping mode"),
    DeclareLaunchArgument(name="viz", default_value="false", description="Enable visualization"),

    # Bag args
    DeclareLaunchArgument(name="bag", default_value="false", description="Play bag file"),
    DeclareLaunchArgument(name="bag_file", default_value="rosbag2_2024_06_27-20_40_24", description="Name of the bag folder to play"),
]
    
def launch_setup(context):
    bag_path = PathJoinSubstitution(["/home/user/ws/Data/bags", LaunchConfiguration("bag_file")])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([join(
                get_package_share_directory('drivers'), 'launch'),
                '/drivers_launch.py']
            )
        ),
        
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("bag")),
            cmd=["ros2", "bag", "play", bag_path, "--topics", "/sensors/imx/image_raw", "/sensors/bno08x/raw"],
            output="screen"
        ),

        Node(
            package='mapper',
            executable='mapper',
            condition=IfCondition(LaunchConfiguration("map")),
        ),
        Node(
            package='controller',
            executable='teleop_node.py',
            condition=IfCondition(LaunchConfiguration("map")),
        ),

        Node(
            package='controller',
            executable='cone_detector.py',
            condition=IfCondition(NotSubstitution(LaunchConfiguration('map'))),
        ),

        Node(
            package='controller',
            executable='mark_detector.py',
            condition=IfCondition(NotSubstitution(LaunchConfiguration('map'))),
        ),

        Node(
            package='controller',
            executable='steering_node',
            condition=IfCondition(NotSubstitution(LaunchConfiguration('map'))),
        ),

        Node(
           package='controller',
           condition=IfCondition(LaunchConfiguration("viz")),
           executable='visualization_node',
        ),

        Node(
           package='controller',
           executable='imu_tracking_node.py',
        ),
    ]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
