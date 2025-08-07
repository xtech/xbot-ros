import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch.logging

logger = launch.logging.get_logger('cartographer_launch')

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('xbot_example'),
        'config'
    )

    logger.info(f'Using config directory: {config_dir}')

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'xbot_cartographer_2d.lua'
            ],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu'),
                ('odom', '/odom')
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                'publish_period_sec': 1.0
            }]
        )
    ])
