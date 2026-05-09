from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yahboom_base',
            executable='ctrl',
            name='yahboom_ctrl',
            output='screen',
        ),
    ])
