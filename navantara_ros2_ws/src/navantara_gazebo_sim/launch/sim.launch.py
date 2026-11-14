#!/usr/bin/env python3
# ==============================================================================
# sim.launch.py
# Gazebo Simulation & Robot Description Launcher for ASV Navantara (UMRAH)
# Kontes Kapal Cepat Tak Berawak Indonesia (KKI)
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('navantara_gazebo_sim')
    default_urdf = os.path.join(pkg_share, 'urdf', 'asv.urdf.xacro')
    default_world = os.path.join(pkg_share, 'worlds', 'kki_arena_course.world')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path = LaunchConfiguration('world')
    launch_gazebo_gui = LaunchConfiguration('gui')

    # Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to Gazebo world file (.world / .sdf)'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch the Gazebo GUI client'
    )

    # Robot State Publisher (Publishes 3D ASV Model to /robot_description)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', default_urdf]),
            'use_sim_time': use_sim_time
        }]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_gui,
        robot_state_publisher,
        joint_state_publisher,
    ])
