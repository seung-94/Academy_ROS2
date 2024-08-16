from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project_14',
            executable='camera_setup',
            name='camera_setup',
            output='screen',
        ),
        Node(
            package='project_14',
            executable='line_follower',
            name='line_follower',
            output='screen',
        ),
    ])
