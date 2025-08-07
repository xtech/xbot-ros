from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'bind_ip_arg',
        #     default_value=EnvironmentVariable('BIND_IP', default_value='0.0.0.0'),
        #     description='Bind IP for the comms node. Set via BIND_IP environment variable.'
        # ),
        # Node(
        #     package='xbot_comms',
        #     executable='xbot_comms',
        #     name='xbot_comms',
        #     parameters=[{'bind_ip': LaunchConfiguration('bind_ip_arg')}]
        # ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[
                {
                    'enable_button': 0,
                    'axis_linear.x': 1,
                    'axis_linear.y': -1,
                    'axis_linear.z': -1,
                    'axis_angular.yaw': 0,
                    'axis_angular.pitch': -1,
                    'axis_angular.roll': -1,
                    'scale_angular.yaw': 2.0,
                    'scale_linear.x': 0.5
                }
            ]
        )
    ])
