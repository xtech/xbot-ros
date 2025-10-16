from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('xbot_example'), 'launch'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'bind_ip_arg',
            default_value=EnvironmentVariable('BIND_IP', default_value='0.0.0.0'),
            description='Bind IP for the comms node. Set via BIND_IP environment variable.'
        ),
        Node(
            package='xbot_comms',
            executable='xbot_comms',
            name='xbot_comms',
            parameters=[{'bind_ip': LaunchConfiguration('bind_ip_arg')}]
    ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [launch_dir, 'rplidar_launch.py']
            )
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [launch_dir, 'urdf_launch.py']
            )
        )
    ])
