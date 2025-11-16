from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='gps_publisher',
            name='gps_publisher'
        ),
        Node(
            package='your_package',
            executable='gps_subscriber',
            name='gps_subscriber'
        ),
    ])
