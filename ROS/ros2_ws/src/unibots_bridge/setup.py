from setuptools import setup

package_name = 'unibots_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/unibots_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
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
            'file_bridge_node = unibots_bridge.file_bridge_node:main',
            'decision_making_node = unibots_bridge.decision_making_node:main',
            'decision_node = unibots_bridge.decision_node:main',
        ],
    },
)
