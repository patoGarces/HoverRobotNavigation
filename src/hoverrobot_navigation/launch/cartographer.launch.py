from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('hoverrobot_navigation')
    config_dir = os.path.join(pkg_share, 'config')
    config_file = os.path.join(config_dir, 'cartographer_config.lua')
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_config.lua'
        ],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'odom')
        ]
    )
    
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'resolution': 0.05}]
    )
    
    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])