#!/usr/bin/env python3
"""
view_rviz.launch.py
===================
Launches RViz2 with preconfigured ASV Navantara view.
By default enables software rendering (LIBGL_ALWAYS_SOFTWARE=1)
to prevent the Mesa Intel HD Graphics 4600 GLSL sampler linking error.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('navantara_navigation')
    rviz_config_path = os.path.join(pkg_share, 'config', 'navantara_view.rviz')

    use_software_gl_arg = DeclareLaunchArgument(
        'use_software_gl',
        default_value='true',
        description='Force LIBGL_ALWAYS_SOFTWARE=1 to bypass Intel HD Graphics GLSL sampler conflict'
    )

    set_software_gl = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1',
        condition=IfCondition(LaunchConfiguration('use_software_gl'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        use_software_gl_arg,
        set_software_gl,
        rviz_node,
    ])
