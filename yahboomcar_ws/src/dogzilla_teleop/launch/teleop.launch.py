import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml'
        ))
    )

    web_server = Node(
        package='dogzilla_teleop',
        executable='web_server',
        name='dogzilla_web_server',
        output='screen',
        parameters=[{'port': 8080, 'open_browser': True}],
    )

    return LaunchDescription([rosbridge, web_server])
