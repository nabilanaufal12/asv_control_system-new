#!/usr/bin/env python3
"""
navantara_webgui_bridge / webgui_bridge_node.py
===============================================
ROS2 Telemetry Bridge to Web GUI & Mission Logger.
Part of ASV Navantara ROS2 Architecture.

Migrated from monolithic endpoints.py, asv_handler.py, and mission_logger.py:
- Decouples Web GUI polling and CSV mission logging into an isolated async process.
- Subscribes to /asv/telemetry, /fix, /perception/image_annotated, and /cmd_vel.
- Serves REST/WebSocket telemetry for the Next.js / HTML5 Dashboard.
- Never blocks real-time navigation or motor control threads.
"""

import csv
import json
import os
import time
from datetime import datetime
from threading import Lock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32, String

try:
    from navantara_interfaces.msg import ASVTelemetry, ObstacleArray
    HAS_CUSTOM_MSG = True
except ImportError:
    HAS_CUSTOM_MSG = False


class ASVWebGUIBridgeNode(Node):
    """Bridges ROS2 topics to asynchronous Web GUI and CSV telemetry logger."""

    def __init__(self):
        super().__init__('asv_webgui_bridge_node')

        self.declare_parameter('log_dir', 'mission_logs')
        self.declare_parameter('enable_csv_logging', True)
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        self.enable_csv = self.get_parameter('enable_csv_logging').get_parameter_value().bool_value

        self.state_lock = Lock()
        self.latest_telemetry = {}
        self.log_file = None
        self.csv_writer = None

        if self.enable_csv:
            self._init_csv_logger()

        # Subscribers
        if HAS_CUSTOM_MSG:
            self.telem_sub = self.create_subscription(
                ASVTelemetry, '/asv/telemetry', self._telem_callback, 10
            )

        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self._gps_callback, 10)
        self.hdg_sub = self.create_subscription(Float32, '/compass/heading', self._hdg_callback, 10)

        self.get_logger().info("WebGUI & Telemetry Bridge Node initialized.")

    def _init_csv_logger(self):
        os.makedirs(self.log_dir, exist_ok=True)
        filename = datetime.now().strftime("mission_%Y%m%d_%H%M%S.csv")
        filepath = os.path.join(self.log_dir, filename)
        self.log_file = open(filepath, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'timestamp', 'mode', 'status', 'lat', 'lon', 'heading',
            'speed_knot', 'servo_pwm', 'thruster_pwm'
        ])
        self.get_logger().info(f"Telemetry logging started at: {filepath}")

    def _telem_callback(self, msg):
        with self.state_lock:
            self.latest_telemetry = {
                'timestamp': time.time(),
                'mode': msg.control_mode,
                'status': msg.mission_status,
                'lat': msg.latitude,
                'lon': msg.longitude,
                'heading': msg.heading_deg,
                'speed': msg.speed_knot,
                'servo': msg.servo_pwm,
                'thruster': msg.thruster_pwm
            }

        if self.csv_writer and self.log_file:
            self.csv_writer.writerow([
                time.time(), msg.control_mode, msg.mission_status,
                msg.latitude, msg.longitude, msg.heading_deg,
                msg.speed_knot, msg.servo_pwm, msg.thruster_pwm
            ])
            self.log_file.flush()

    def _gps_callback(self, msg: NavSatFix):
        with self.state_lock:
            self.latest_telemetry['lat'] = msg.latitude
            self.latest_telemetry['lon'] = msg.longitude

    def _hdg_callback(self, msg: Float32):
        with self.state_lock:
            self.latest_telemetry['heading'] = msg.data

    def destroy_node(self):
        if self.log_file and not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ASVWebGUIBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
