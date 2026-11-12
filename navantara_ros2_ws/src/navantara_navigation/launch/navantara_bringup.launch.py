#!/usr/bin/env python3
"""
navantara_bringup.launch.py
===========================
Master Bringup Launch File for ASV Navantara.
Supports both Physical Hardware Mode (on NVIDIA Jetson) and
Laptop Mock Development Mode (on Ubuntu 24.04 laptop without hardware).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Launch Arguments
    use_laptop_mock_arg = DeclareLaunchArgument(
        'use_laptop_mock',
        default_value='true',
        description='Run in lightweight laptop mock mode (synthetic buoys + 2D vessel kinematics) without hardware'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyASV',
        description='Serial port for MCU communication (used when use_laptop_mock:=false)'
    )

    # 2. Static Transform Publishers (Sensor Extrinsics relative to base_link)
    # base_link -> camera_link (x=0.8m forward, z=0.25m above waterline, pitched down 5 deg = 0.087 rad)
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=['--x', '0.8', '--y', '0.0', '--z', '0.25',
                   '--roll', '0.0', '--pitch', '0.087', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
    )

    # base_link -> gps_link (x=0.0m center, z=0.4m mast)
    tf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_gps',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.4',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'gps_link']
    )

    # base_link -> imu_link (x=0.0m center, z=0.1m hull)
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.1',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
    )

    # 3. Perception Node
    # In mock mode: simulation_mode:=True generates synthetic buoy stream and projected pointcloud
    # In hardware mode: runs TensorRT GPU inference
    perception_mock_node = Node(
        package='navantara_perception',
        executable='vision_node',
        name='asv_vision_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_laptop_mock')),
        parameters=[
            {'simulation_mode': True},
            {'publish_annotated_image': True},
            {'inference_rate_hz': 20.0},
            {'target_frame_id': 'base_link'},
        ]
    )

    perception_hw_node = Node(
        package='navantara_perception',
        executable='vision_node',
        name='asv_vision_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_laptop_mock')),
        parameters=[
            {'simulation_mode': False},
            {'model_path': 'best.engine'},
            {'camera_source': 0},
            {'publish_annotated_image': True},
            {'inference_rate_hz': 25.0},
            {'target_frame_id': 'base_link'},
        ]
    )

    # 4. Control Bridge Node
    # In mock mode: runs internal 50Hz 2D kinematic vessel model & dynamic TF (odom -> base_link)
    # In hardware mode: connects to ESP32/STM32 MCU over UART with watchdog
    control_bridge_mock_node = Node(
        package='navantara_control_bridge',
        executable='serial_bridge_node',
        name='asv_serial_bridge_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_laptop_mock')),
        parameters=[
            {'mock_hardware': True},
            {'serial_poll_rate_hz': 50.0},
            {'watchdog_timeout_sec': 0.5},
            {'angular_gain': 30.0},
            {'linear_gain': 250.0},
        ]
    )

    control_bridge_hw_node = Node(
        package='navantara_control_bridge',
        executable='serial_bridge_node',
        name='asv_serial_bridge_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_laptop_mock')),
        parameters=[
            {'mock_hardware': False},
            {'port': LaunchConfiguration('port')},
            {'baudrate': 230400},
            {'watchdog_timeout_sec': 0.5},
            {'serial_poll_rate_hz': 50.0},
            {'use_binary_protocol': False},
        ]
    )

    # 5. WebGUI & Telemetry Bridge Node
    webgui_node = Node(
        package='navantara_webgui_bridge',
        executable='webgui_bridge_node',
        name='asv_webgui_bridge_node',
        output='screen',
        parameters=[
            {'enable_csv_logging': True},
            {'log_dir': 'mission_logs'},
        ]
    )

    return LaunchDescription([
        use_laptop_mock_arg,
        port_arg,
        tf_camera,
        tf_gps,
        tf_imu,
        perception_mock_node,
        perception_hw_node,
        control_bridge_mock_node,
        control_bridge_hw_node,
        webgui_node,
    ])
