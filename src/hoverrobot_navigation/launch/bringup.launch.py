from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # === Paths ===
    slam_params_file = os.path.join(
        get_package_share_directory('hoverrobot_navigation'),
        'config', 'slam_toolbox.yaml'
    )

    ps5_cam_launch = os.path.join(
        get_package_share_directory('ros2_ps5_stereo'),
        'launch',
        'ros2_ps5_stereo_launch.py'
    )

    description_launch = os.path.join(
        get_package_share_directory('hoverrobot_description'),
        'launch',
        'display.launch.py'
    )

    nav2_local_launch = os.path.join(
        get_package_share_directory('hoverrobot_navigation'),
        'launch',
        'nav2_local.launch.py'
    )

    cartographer_launch = os.path.join(
        get_package_share_directory('hoverrobot_navigation'),
        'launch',
        'cartographer.launch.py'
    )

    # === Lifecycle Nodes ===
    comms_node = LifecycleNode(
        package='ros2_hoverrobot_comms',
        executable='hoverrobot_comms_node',
        name='hoverrobot_comms',
        namespace='',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[slam_params_file],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # === Include Launch Descriptions ===
    ps5_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ps5_cam_launch)
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch)
    )

    nav2_launch_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_local_launch)
    )

    cartographer_launch_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_launch)
    )

    # === Activación Lifecycle Nodes ===
    lifecycle_manager_comms = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_hoverrobot',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,                  # arranca los nodos automáticamente
            'node_names': ['hoverrobot_comms'], # nodo lifecycle, puede sumar mas nodos
            'bond_timeout': 0.0,                # opcional, 0.0 desactiva el watchdog,
            'autostart_delay': 3.0,             # tiempo entre reintentos 
            'retry_attempts': -1                # Reintentar infinitamente
        }]
    )

    lifecycle_manager_slam = LifecycleNode(
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

    # Configurar y activar SLAM tras 2 segundos (asegura que comms esté activo)
    # activate_slam = TimerAction(
    #     period=2.0,
    #     actions=[
    #         EmitEvent(
    #             event=ChangeState(
    #                 lifecycle_node_matcher=lambda node: node.name == 'slam_toolbox',
    #                 transition_id=Transition.TRANSITION_CONFIGURE
    #             )
    #         ),
    #         EmitEvent(
    #             event=ChangeState(
    #                 lifecycle_node_matcher=lambda node: node.name == 'slam_toolbox',
    #                 transition_id=Transition.TRANSITION_ACTIVATE
    #             )
    #         )
    #     ]
    # )

    # === LaunchDescription final ===
    return LaunchDescription([
        ps5_cam,
        description,
        comms_node,
        lifecycle_manager_comms,
        # slam_node,
        nav2_launch_inc,
        # lifecycle_manager_slam
        # cartographer_launch_inc
    ])
