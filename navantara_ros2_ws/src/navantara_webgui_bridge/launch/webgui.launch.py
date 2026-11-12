#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value='mission_logs',
        description='Directory to store mission CSV logs'
    )

    enable_csv_arg = DeclareLaunchArgument(
        'enable_csv_logging',
        default_value='true',
        description='Record mission telemetry to CSV'
    )

    webgui_bridge_node = Node(
        package='navantara_webgui_bridge',
        executable='webgui_bridge_node',
        name='asv_webgui_bridge_node',
        output='screen',
        parameters=[
            {'log_dir': LaunchConfiguration('log_dir')},
            {'enable_csv_logging': LaunchConfiguration('enable_csv_logging')},
        ]
    )

    return LaunchDescription([
        log_dir_arg,
        enable_csv_arg,
        webgui_bridge_node,
    ])
