from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('unibots_bridge')
    params = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='unibots_bridge',
            executable='file_bridge_node',
            name='file_bridge_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='unibots_bridge',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[params],
        ),
    ])
