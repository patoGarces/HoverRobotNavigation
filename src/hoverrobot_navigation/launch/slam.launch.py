from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory('hoverrobot_navigation'),
        'config',
        # 'slam_toolbox.yaml'
        'mapper_params_online_async.yaml'
    )

    slam_node = LifecycleNode( 
        package='slam_toolbox', 
        executable='async_slam_toolbox_node', 
        name='slam_toolbox', 
        namespace='', 
        output='screen', 
        parameters=[slam_params_file] )

    lifecycle = LifecycleNode(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        namespace='', 
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 20.0,
            'node_names': ['slam_toolbox']
        }]
    )

    return LaunchDescription([
        slam_node,
        lifecycle
    ])
