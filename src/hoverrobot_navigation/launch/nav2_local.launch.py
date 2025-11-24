from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('hoverrobot_navigation'),
        'config',
        'params.yaml'
    )

    level_log = 'warn'

    return LaunchDescription([
        LifecycleNode(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            namespace='', 
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', level_log]
        ),
        LifecycleNode(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            namespace='', 
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', level_log]
        ),
        LifecycleNode(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            namespace='', 
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', level_log]
        ),
        LifecycleNode(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            namespace='', 
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', level_log]
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_local",
            output="screen",
            namespace='', 
            parameters=[{
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "bt_navigator",
                    "behavior_server"
                ]
            }],
            arguments=['--ros-args', '--log-level', level_log]
        )
    ])
