#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('navantara_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'magnetic_declination_radians': 0.0},
            {'yaw_offset': 0.0},
            {'zero_altitude': True},
            {'broadcast_utm_transform': False},
            {'publish_filtered_gps': True},
            {'use_odometry_yaw': False},
            {'wait_for_datum': False},
        ],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/fix'),
            ('odometry/filtered', '/odometry/filtered'),
            ('odometry/gps', '/odometry/gps'),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
        navsat_transform_node,
    ])
