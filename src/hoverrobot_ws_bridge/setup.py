from setuptools import setup

package_name = 'hoverrobot_ws_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HoverRobot',
    maintainer_email='',
    description='HoverRobot ROS2 WebSocket Bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = hoverrobot_ws_bridge.bridge_node:main',
        ],
    },
)