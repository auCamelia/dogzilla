import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('yahboom_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'yahboom_xgo_rviz.xacro')
    rviz_config = os.path.join(pkg_share, 'config', 'digital_twin.rviz')

    robot_desc = xacro.process_file(xacro_file).toxml()

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 (set false when running headless on the robot)'),

        # Publishes TF transforms for every link from /joint_states + /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # Reads real servo angles via DOGZILLALib and publishes /joint_states at 20 Hz
        Node(
            package='yahboom_dog_joint_state',
            executable='yahboomcar_joint_state',
            name='yahboom_dog_joint_state',
            output='screen',
        ),

        # RViz2 — can be disabled with use_rviz:=false when running on the robot headless
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
