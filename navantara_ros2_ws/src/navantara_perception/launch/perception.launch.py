#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('navantara_perception')
    calib_file = os.path.join(pkg_share, 'config', 'camera_calibration.yaml')

    sim_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run perception in mock synthetic buoy mode for laptop development'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='best.engine',
        description='Path to TensorRT .engine or YOLO .pt file'
    )

    vision_node = Node(
        package='navantara_perception',
        executable='vision_node',
        name='asv_vision_node',
        output='screen',
        parameters=[
            {'simulation_mode': LaunchConfiguration('simulation_mode')},
            {'model_path': LaunchConfiguration('model_path')},
            {'camera_source': 0},
            {'publish_annotated_image': True},
            {'inference_rate_hz': 25.0},
            {'target_frame_id': 'base_link'},
        ]
    )

    return LaunchDescription([
        sim_mode_arg,
        model_path_arg,
        vision_node,
    ])
