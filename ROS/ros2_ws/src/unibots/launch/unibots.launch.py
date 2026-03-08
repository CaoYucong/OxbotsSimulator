from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('unibots')
    params = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='unibots',
            executable='web_bridge_node',
            name='web_bridge_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='unibots',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='unibots',
            executable='pose_estimation_node',
            name='pose_estimation',
            output='screen',
            parameters=[params],
        ),
    ])
