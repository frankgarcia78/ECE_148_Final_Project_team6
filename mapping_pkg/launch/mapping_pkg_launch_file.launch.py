from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapping_pkg',
            executable='mapping',
            output='screen'),
    ])