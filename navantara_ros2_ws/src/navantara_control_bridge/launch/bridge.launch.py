#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mock_hw_arg = DeclareLaunchArgument(
        'mock_hardware',
        default_value='false',
        description='Enable laptop mock mode with internal 2D vessel kinematics'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyASV',
        description='Serial port for MCU communication'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='230400',
        description='Serial communication baudrate'
    )

    serial_bridge_node = Node(
        package='navantara_control_bridge',
        executable='serial_bridge_node',
        name='asv_serial_bridge_node',
        output='screen',
        parameters=[
            {'mock_hardware': LaunchConfiguration('mock_hardware')},
            {'port': LaunchConfiguration('port')},
            {'baudrate': LaunchConfiguration('baudrate')},
            {'watchdog_timeout_sec': 0.5},
            {'serial_poll_rate_hz': 50.0},
            {'use_binary_protocol': False},
            {'max_speed_pwm': 1850},
            {'min_speed_pwm': 1150},
            {'neutral_pwm': 1500},
            {'servo_center_deg': 90},
            {'angular_gain': 30.0},
            {'linear_gain': 250.0},
        ]
    )

    return LaunchDescription([
        mock_hw_arg,
        port_arg,
        baudrate_arg,
        serial_bridge_node,
    ])
