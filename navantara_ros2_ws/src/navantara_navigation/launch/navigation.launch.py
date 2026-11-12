#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('navantara_navigation')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle stack'
    )

    # Controller Server (Regulated Pure Pursuit)
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # Costmap 2D Node (Rolling obstacle persistence for buoys)
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d_markers',
        name='local_costmap_markers',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': ['controller_server']}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        controller_node,
        costmap_node,
        lifecycle_manager_node,
    ])
