from setuptools import setup

package_name = 'unibots'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/unibots.launch.py']),
        (
            'share/' + package_name + '/config',
            [
                'config/params.yaml',
                'config/real_camera_intrinsic.yaml',
                'config/simulation_camera_intrinsic.json',
                'config/tag_world_map.json',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UniBots Dev',
    maintainer_email='you@example.com',
    description='ROS2 bridge and decision node for UniBots simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_bridge_node = unibots.web_bridge_node:main',
            'radar_sensor_node = unibots.radar_sensor_node:main',
            'front_camera_node = unibots.front_camera_node:main',
            'timer_node = unibots.timer_node:main',
            'decision_node = unibots.decision_node:main',
            'pose_estimation_node = unibots.pose_estimation_node:main',
            'motion_control_node = unibots.motion_control_node:main',
        ],
    },
)
