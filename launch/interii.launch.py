from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

launch_args = [
    
]
    
def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([join(
                get_package_share_directory('drivers'), 'launch'),
                '/drivers_launch.py']
            )
        ),

        Node(
            package='vins',
            executable='vins_node',
            parameters=[
                {'config': '/home/user/ws/src/config/interii_mono_imu_config.yaml'},
            ],
        ),
        Node(
            package='visualizer',
            executable='visualization_node',
        ),
        Node(
            package='controller_node',
            executable='steering_node',
        ),
    ]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld