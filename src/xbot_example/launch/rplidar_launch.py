from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            parameters=[
                {'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser', 'inverted': False, 'angle_compensate': True,
                 'baudrate': 115200}]
        )
    ])
