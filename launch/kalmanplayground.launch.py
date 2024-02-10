from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kalmanplayground',
            executable='function_publisher',
            name='function_publisher'
        ),
        Node(
            package='kalmanplayground',
            executable='function_subscriber',
            name='function_subscriber'
        ),
        Node(
            package='kalmanplayground',
            executable='kalman_filter',
            name='kalman_filter'
        ),
        Node(
            package='kalmanplayground',
            executable='eval',
            name='eval'
        ),
    ])