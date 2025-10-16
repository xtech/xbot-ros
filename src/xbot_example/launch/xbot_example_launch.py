from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('xbot_example'), 'launch'])
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [launch_dir, 'xbot_base_launch.py']
            )
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [launch_dir, 'cartographer_launch.py']
            )
        )
    ])
