from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='odom',
        description='Target frame for the point cloud'
    )

    max_cloud_size_arg = DeclareLaunchArgument(
        'max_cloud_size',
        default_value='10000',
        description='Maximum number of points in the point cloud'
    )

    # Create node
    scan_to_pointcloud_node = Node(
        package='xbot_utils',
        executable='scan_to_pointcloud_node',
        name='scan_to_pointcloud',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'max_cloud_size': LaunchConfiguration('max_cloud_size')
        }],
        remappings=[
            ('scan', '/scan'),
            ('pointcloud', '/pointcloud')
        ],
        output='screen'
    )

    # Create launch description
    return LaunchDescription([
        target_frame_arg,
        max_cloud_size_arg,
        scan_to_pointcloud_node
    ])
