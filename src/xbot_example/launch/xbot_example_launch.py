from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bind_ip_arg',
            default_value=EnvironmentVariable('BIND_IP', default_value='0.0.0.0'),
            description='Bind IP for the comms node. Set via BIND_IP environment variable.'
        ),
        Node(
            package='xbot_comms',
            namespace='xbot_comms',
            executable='xbot_comms',
            name='xbot_comms',
            parameters=[{'bind_ip': LaunchConfiguration('bind_ip_arg')}]
    ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'urdf_launch.py'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'cartographer_launch.py'
            )
        )
    ])
