from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='skai_dashboard',
            executable='dashboard',
            name='skai_dashboard',
            output='screen',
        )
    ])
