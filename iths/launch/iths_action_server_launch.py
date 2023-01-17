from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='iths',
            executable='iths_action_server.py',
            output='screen'
        )
    ])