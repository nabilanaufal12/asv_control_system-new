#!/usr/bin/env python3
"""
navantara_control_bridge / serial_bridge_node.py
================================================
ROS2 Microcontroller Communication Bridge & Hardware Watchdog Node.
Part of ASV Navantara ROS2 Architecture.

Supports:
1. Physical Hardware Mode: Serial UART communication with ESP32/STM32 MCU.
   Translates /cmd_vel to PWM and Rudder angle, enforces hardware watchdog,
   supports deterministic binary CRC16 protocol and legacy ASCII.
2. Laptop Local Mock Mode (`mock_hardware:=True`):
   Bypasses serial port, runs an internal 50 Hz 2D hydrodynamics vessel model,
   integrates /cmd_vel, and publishes simulated /fix (NavSatFix), /compass/heading,
   /odom/raw (Odometry), /asv/telemetry, and dynamic TF 'odom' -> 'base_link'.
"""

import math
import os
import struct
import time
from typing import Optional

import numpy as np
import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float32, Header, String

try:
    import tf2_ros
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False

try:
    from navantara_interfaces.msg import ASVTelemetry
    HAS_CUSTOM_MSG = True
except ImportError:
    HAS_CUSTOM_MSG = False


def calculate_crc16(data: bytes) -> int:
    """Computes CRC-16-CCITT for deterministic binary frames."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Converts a yaw angle (radians) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ASVSerialBridgeNode(Node):
    """ROS2 Hardware Bridge / Vessel Kinematic Simulator for ASV Navantara."""

    FRAME_HEADER = bytes([0xAA, 0x55])
    CMD_DRIVE = 0x01
    CMD_HEARTBEAT = 0x02
    CMD_EMERGENCY_STOP = 0x03

    def __init__(self):
        super().__init__('asv_serial_bridge_node')

        # 1. Parameter Declarations
        self.declare_parameter('mock_hardware', False)
        self.declare_parameter('port', '/dev/ttyASV')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('watchdog_timeout_sec', 0.5)
        self.declare_parameter('serial_poll_rate_hz', 50.0)
        self.declare_parameter('use_binary_protocol', False)
        self.declare_parameter('max_speed_pwm', 1850)
        self.declare_parameter('min_speed_pwm', 1150)
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('servo_center_deg', 90)
        self.declare_parameter('servo_max_deg', 135)
        self.declare_parameter('servo_min_deg', 45)
        self.declare_parameter('angular_gain', 30.0)
        self.declare_parameter('linear_gain', 250.0)

        # Simulation origin coordinates (KKI Venue / Tanjungpinang waters)
        self.declare_parameter('sim_origin_lat', 0.925430)
        self.declare_parameter('sim_origin_lon', 104.455210)

        # Retrieve Parameters
        self.mock_hw = self.get_parameter('mock_hardware').get_parameter_value().bool_value
        self.port_name = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout_sec').get_parameter_value().double_value
        poll_rate = self.get_parameter('serial_poll_rate_hz').get_parameter_value().double_value
        self.use_binary = self.get_parameter('use_binary_protocol').get_parameter_value().bool_value
        self.max_pwm = self.get_parameter('max_speed_pwm').get_parameter_value().integer_value
        self.min_pwm = self.get_parameter('min_speed_pwm').get_parameter_value().integer_value
        self.neutral_pwm = self.get_parameter('neutral_pwm').get_parameter_value().integer_value
        self.servo_center = self.get_parameter('servo_center_deg').get_parameter_value().integer_value
        self.servo_max = self.get_parameter('servo_max_deg').get_parameter_value().integer_value
        self.servo_min = self.get_parameter('servo_min_deg').get_parameter_value().integer_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value

        self.sim_lat = self.get_parameter('sim_origin_lat').get_parameter_value().double_value
        self.sim_lon = self.get_parameter('sim_origin_lon').get_parameter_value().double_value

        # 2. State & Safety Variables
        self.serial_conn: Optional[serial.Serial] = None
        self.read_buffer = b""
        self.last_cmd_vel_time = 0.0
        self.target_thruster_pwm = self.neutral_pwm
        self.target_servo_deg = self.servo_center
        self.target_motor_left = 1000
        self.target_motor_right = 1000
        self.failsafe_triggered = False

        # 3. Kinematic Vessel Simulation State (2D Hydrodynamics)
        # Coordinates in odom frame
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_yaw = 0.0          # Radians (0 = East/X+, pi/2 = North/Y+)
        self.sim_u = 0.0            # Surge velocity (m/s)
        self.sim_r = 0.0            # Yaw rate (rad/s)
        self.target_u = 0.0
        self.target_r = 0.0
        self.last_sim_time = time.time()

        # 4. Setup Publishers
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.gps_pub = self.create_publisher(NavSatFix, '/fix', sensor_qos)
        self.heading_pub = self.create_publisher(Float32, '/compass/heading', sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom/raw', sensor_qos)

        if HAS_CUSTOM_MSG:
            self.telemetry_pub = self.create_publisher(ASVTelemetry, '/asv/telemetry', 10)
        else:
            self.telemetry_pub = None

        self.debug_raw_pub = self.create_publisher(String, '/serial/raw_incoming', 5)

        # TF Broadcaster for mock mode
        if HAS_TF2:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # 5. Setup Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )

        # 6. Initialize Hardware or Mock
        if not self.mock_hw:
            success = self._connect_serial()
            if not success:
                self.get_logger().warn(
                    "Physical serial port not found! Automatically falling back to LAPTOP MOCK HARDWARE mode."
                )
                self.mock_hw = True

        if self.mock_hw:
            self.get_logger().info(
                "Serial Bridge operating in LAPTOP MOCK MODE. Running 50Hz 2D kinematic vessel model & TF broadcaster."
            )

        timer_period = 1.0 / max(10.0, poll_rate)
        self.io_timer = self.create_timer(timer_period, self._main_io_loop)

    def _connect_serial(self) -> bool:
        candidate_ports = [self.port_name, '/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyASV']
        for port in candidate_ports:
            if os.path.exists(port):
                try:
                    self.get_logger().info(f"Opening hardware port {port} @ {self.baudrate}...")
                    self.serial_conn = serial.Serial(port, self.baudrate, timeout=0.02)
                    self.serial_conn.reset_input_buffer()
                    self.serial_conn.reset_output_buffer()
                    self.get_logger().info(f"Connected to MCU on {port}")
                    self.failsafe_triggered = False
                    return True
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to open port {port}: {e}")
        return False

    def _cmd_vel_callback(self, msg: Twist):
        now = time.time()
        self.last_cmd_vel_time = now
        self.failsafe_triggered = False

        self.target_u = msg.linear.x
        self.target_r = msg.angular.z

        # Thruster and rudder mapping
        pwm_delta = int(msg.linear.x * self.linear_gain)
        self.target_thruster_pwm = max(self.min_pwm, min(self.max_pwm, self.neutral_pwm + pwm_delta))
        deg_delta = int(msg.angular.z * self.angular_gain)
        self.target_servo_deg = max(self.servo_min, min(self.servo_max, self.servo_center - deg_delta))

        if not self.mock_hw:
            self._send_actuator_packet(self.target_thruster_pwm, self.target_servo_deg, 1000, 1000)

    def _send_actuator_packet(self, thruster_pwm: int, servo_deg: int, front_left: int, front_right: int):
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        try:
            if self.use_binary:
                payload = struct.pack('>HHHH', thruster_pwm, servo_deg, front_left, front_right)
                header = self.FRAME_HEADER + bytes([self.CMD_DRIVE, len(payload)])
                crc = calculate_crc16(header + payload)
                packet = header + payload + struct.pack('>H', crc)
                self.serial_conn.write(packet)
            else:
                ascii_cmd = f"A,{servo_deg},{thruster_pwm},1000,{front_left},{front_right},1000,1000\n"
                self.serial_conn.write(ascii_cmd.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            self.serial_conn = None

    def _main_io_loop(self):
        now = time.time()

        # Watchdog Check
        if (now - self.last_cmd_vel_time) > self.watchdog_timeout:
            if not self.failsafe_triggered:
                self.get_logger().warn(
                    f"WATCHDOG TRIGGERED: No /cmd_vel for {self.watchdog_timeout}s. Zeroing thrust."
                )
                self.target_u = 0.0
                self.target_r = 0.0
                self.target_thruster_pwm = self.neutral_pwm
                self.target_servo_deg = self.servo_center
                if not self.mock_hw:
                    self._send_actuator_packet(self.neutral_pwm, self.servo_center, 1000, 1000)
                self.failsafe_triggered = True

        if self.mock_hw:
            self._step_kinematic_vessel_simulation(now)
            return

        # Physical Hardware Loop
        if self.serial_conn is None or not self.serial_conn.is_open:
            self._connect_serial()
            return

        try:
            waiting = self.serial_conn.in_waiting
            if waiting > 0:
                chunk = self.serial_conn.read(waiting)
                self.read_buffer += chunk

                while b'\n' in self.read_buffer:
                    line_bytes, self.read_buffer = self.read_buffer.split(b'\n', 1)
                    line_str = line_bytes.decode('utf-8', errors='ignore').strip()
                    if line_str:
                        self._parse_incoming_line(line_str)

                if len(self.read_buffer) > 4096:
                    self.read_buffer = b""
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read exception: {e}")
            self.serial_conn = None

    def _step_kinematic_vessel_simulation(self, current_time: float):
        """
        Integrates a 2D marine vessel kinematic model with water drag and inertia.
        Publishes simulated /odom/raw, dynamic TF (odom -> base_link), /fix, /compass/heading.
        """
        dt = current_time - self.last_sim_time
        self.last_sim_time = current_time
        if dt <= 0.0 or dt > 0.5:
            return

        # Hydrodynamics: 1st-order surge inertia with quadratic water resistance
        # tau_u: acceleration time constant (1.2s), drag coefficient: 0.15
        tau_u = 1.2
        drag = 0.15 * self.sim_u * abs(self.sim_u)
        self.sim_u += ((self.target_u - self.sim_u) / tau_u - drag) * dt
        self.sim_u = float(max(-0.5, min(3.0, self.sim_u)))

        # Yaw rate response: rudder dynamics (tau_r: 0.4s)
        tau_r = 0.4
        self.sim_r += ((self.target_r - self.sim_r) / tau_r) * dt
        self.sim_r = float(max(-1.2, min(1.2, self.sim_r)))

        # Position integration
        self.sim_yaw = (self.sim_yaw + self.sim_r * dt) % (2.0 * math.pi)
        self.sim_x += self.sim_u * math.cos(self.sim_yaw) * dt
        self.sim_y += self.sim_u * math.sin(self.sim_yaw) * dt

        stamp = self.get_clock().now().to_msg()
        q = yaw_to_quaternion(self.sim_yaw)

        # 1. Broadcast Dynamic TF (odom -> base_link)
        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = float(self.sim_x)
            t.transform.translation.y = float(self.sim_y)
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        # 2. Publish Odometry (/odom/raw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.sim_x)
        odom.pose.pose.position.y = float(self.sim_y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = float(self.sim_u)
        odom.twist.twist.angular.z = float(self.sim_r)
        self.odom_pub.publish(odom)

        # 3. Publish Compass Heading (/compass/heading) in degrees (0 = North)
        # In ROS, yaw 0 = East (X+). In compass navigation, 0 = North.
        compass_deg = (90.0 - math.degrees(self.sim_yaw)) % 360.0
        hdg_msg = Float32()
        hdg_msg.data = float(compass_deg)
        self.heading_pub.publish(hdg_msg)

        # 4. Publish GPS NavSatFix (/fix) with WGS-84 metric conversion
        # 1 deg latitude ~ 111,139 m; 1 deg longitude ~ 111,139 * cos(lat) m
        lat_offset = self.sim_y / 111139.0
        lon_offset = self.sim_x / (111139.0 * math.cos(math.radians(self.sim_lat)))
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gps_link'
        fix.latitude = self.sim_lat + lat_offset
        fix.longitude = self.sim_lon + lon_offset
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX  # RTK Fix simulation
        self.gps_pub.publish(fix)

        # 5. Publish ASVTelemetry
        if HAS_CUSTOM_MSG and self.telemetry_pub:
            telem = ASVTelemetry()
            telem.header.stamp = stamp
            telem.control_mode = "AUTO_MOCK"
            telem.mission_status = "RUNNING"
            telem.latitude = fix.latitude
            telem.longitude = fix.longitude
            telem.heading_deg = float(compass_deg)
            telem.speed_knot = float(self.sim_u * 1.94384)
            telem.gps_satellites = 18
            telem.gps_fix_type = 4
            telem.servo_pwm = self.target_servo_deg
            telem.thruster_pwm = self.target_thruster_pwm
            telem.is_connected = True
            self.telemetry_pub.publish(telem)

    def _parse_incoming_line(self, line: str):
        raw_msg = String()
        raw_msg.data = line
        self.debug_raw_pub.publish(raw_msg)

        if line.startswith("{") and line.endswith("}"):
            try:
                import json
                data = json.loads(line)
                stamp = self.get_clock().now().to_msg()

                if "lat" in data and "lon" in data:
                    lat = float(data.get("lat", 0.0))
                    lon = float(data.get("lon", 0.0))
                    if lat != 0.0 and lon != 0.0:
                        fix_msg = NavSatFix()
                        fix_msg.header.stamp = stamp
                        fix_msg.header.frame_id = 'gps_link'
                        fix_msg.latitude = lat
                        fix_msg.longitude = lon
                        fix_msg.status.status = (
                            NavSatStatus.STATUS_GBAS_FIX if data.get("fix", 0) >= 4 else NavSatStatus.STATUS_FIX
                        )
                        self.gps_pub.publish(fix_msg)

                if "hdg" in data:
                    hdg = float(data.get("hdg", 0.0))
                    hdg_msg = Float32()
                    hdg_msg.data = hdg
                    self.heading_pub.publish(hdg_msg)

                if HAS_CUSTOM_MSG and self.telemetry_pub:
                    telem = ASVTelemetry()
                    telem.header.stamp = stamp
                    telem.control_mode = str(data.get("mode", "UNKNOWN"))
                    telem.mission_status = str(data.get("sts", "UNKNOWN"))
                    telem.latitude = float(data.get("lat", 0.0))
                    telem.longitude = float(data.get("lon", 0.0))
                    telem.heading_deg = float(data.get("hdg", 0.0))
                    telem.speed_knot = float(data.get("sog", 0.0))
                    telem.gps_satellites = int(data.get("sat", 0))
                    telem.gps_fix_type = int(data.get("fix", 0))
                    telem.servo_pwm = int(data.get("srv", 90))
                    telem.thruster_pwm = int(data.get("mot", 1500))
                    telem.is_connected = True
                    self.telemetry_pub.publish(telem)
            except Exception:
                pass

    def destroy_node(self):
        if not self.mock_hw and self.serial_conn and self.serial_conn.is_open:
            try:
                self._send_actuator_packet(self.neutral_pwm, self.servo_center, 1000, 1000)
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ASVSerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
