from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('unibots')
    params = os.path.join(pkg_share, 'config', 'params.yaml')
    with open(params, 'r', encoding='utf-8') as f:
        params_payload = yaml.safe_load(f) or {}
    front_camera_params = params_payload.get('front_camera_node', {}).get('ros__parameters', {})
    use_real_sensor = bool(front_camera_params.get('use_real_sensor', False))

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
            executable='radar_sensor_node',
            name='radar_sensor_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='unibots',
            executable='front_camera_node',
            name='front_camera_node',
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
            parameters=[params, {'use_real_sensor': use_real_sensor}],
        ),
    ])
