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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
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

    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyGPS',
        description='Serial port for RTK GNSS receiver (used when use_laptop_mock:=false)'
    )

    enable_autonomy_arg = DeclareLaunchArgument(
        'enable_autonomy',
        default_value='true',
        description='Launch autonomous buoy gate navigation node (buoy_navigator_node)'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz2 with preconfigured ASV Navantara view'
    )

    # 2. Robot Description & State Publisher (3D Catamaran Hull & Sensor Mounts)
    sim_share = get_package_share_directory('navantara_gazebo_sim')
    urdf_path = os.path.join(sim_share, 'urdf', 'asv.urdf.xacro')
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Note: Sensor frame transforms (base_link -> camera_link, gps_link, imu_link)
    # are published directly by robot_state_publisher from the URDF model.

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
            {'model_path': '/home/navantara/navantara/src/navantara_backend/vision/best.engine'},
            {'camera_source': 0},
            {'publish_annotated_image': True},
            {'inference_rate_hz': 25.0},
            {'target_frame_id': 'base_link'},
        ]
    )

    # 4. RTK GNSS Receiver Node (u-blox ZED-F9P over USB Serial)
    rtk_gnss_node = Node(
        package='navantara_localization',
        executable='rtk_gnss_node',
        name='asv_rtk_gnss_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_laptop_mock')),
        parameters=[
            {'gps_port': LaunchConfiguration('gps_port')},
            {'baudrate': 115200},
            {'frame_id': 'gps_link'},
            {'publish_rate_hz': 10.0},
        ]
    )

    # 5. Control Bridge Node
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

    # 5. Autonomous Buoy Gate Navigator & 2D Rolling Costmap Node
    buoy_navigator_node = Node(
        package='navantara_navigation',
        executable='buoy_navigator_node',
        name='asv_buoy_navigator_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_autonomy')),
        parameters=[
            {'lookahead_distance': 3.5},
            {'cruise_speed': 1.2},
            {'max_angular_speed': 1.5},
            {'persistence_timeout': 4.0},
            {'gate_width_nominal': 2.0},
            {'single_buoy_lateral_offset': 1.0},
            {'costmap_size_x': 20.0},
            {'costmap_size_y': 20.0},
            {'costmap_resolution': 0.1},
        ]
    )

    # 6. WebGUI & Telemetry Bridge Node
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

    # 7. RViz2 3D Visualizer (Optional)
    nav_share = get_package_share_directory('navantara_navigation')
    rviz_config_path = os.path.join(nav_share, 'config', 'navantara_view.rviz')
    set_software_gl = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        use_laptop_mock_arg,
        port_arg,
        gps_port_arg,
        enable_autonomy_arg,
        launch_rviz_arg,
        set_software_gl,
        robot_state_publisher_node,
        perception_mock_node,
        perception_hw_node,
        rtk_gnss_node,
        control_bridge_mock_node,
        control_bridge_hw_node,
        buoy_navigator_node,
        webgui_node,
        rviz_node,
    ])
