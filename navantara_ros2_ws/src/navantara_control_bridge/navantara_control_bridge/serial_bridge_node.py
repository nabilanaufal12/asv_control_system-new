#!/usr/bin/env python3
"""
navantara_control_bridge / serial_bridge_node.py
================================================
ROS2 Microcontroller Communication Bridge & Hardware Watchdog Node.
Part of ASV Navantara ROS2 Architecture.

Migrated from monolithic serial_service.py and asv_handler.py:
- Decouples serial I/O from Python GIL bottleneck.
- Subscribes to standard /cmd_vel (geometry_msgs/Twist) from Nav2.
- Implements hardware-timed Watchdog Failsafe (kills thrusters if connection drops).
- Translates /cmd_vel to differential thruster PWM and rudder servo angle.
- Supports both Deterministic Framed Binary Protocol (CRC16) and legacy ASCII fallback.
- Publishes /fix (NavSatFix), /compass/heading (Float32), and /asv/telemetry.
"""

import os
import struct
import time
from typing import Optional

import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float32, Header, String

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


class ASVSerialBridgeNode(Node):
    """ROS2 Hardware Bridge to ESP32 / STM32 microcontroller."""

    # Protocol Constants
    FRAME_HEADER = bytes([0xAA, 0x55])
    CMD_DRIVE = 0x01
    CMD_HEARTBEAT = 0x02
    CMD_EMERGENCY_STOP = 0x03

    def __init__(self):
        super().__init__('asv_serial_bridge_node')

        # 1. Parameter Declarations
        self.declare_parameter('port', '/dev/ttyASV')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('watchdog_timeout_sec', 0.5)
        self.declare_parameter('serial_poll_rate_hz', 50.0)
        self.declare_parameter('use_binary_protocol', False)  # Set True for new firmware, False for legacy ASCII
        self.declare_parameter('max_speed_pwm', 1850)
        self.declare_parameter('min_speed_pwm', 1150)
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('servo_center_deg', 90)
        self.declare_parameter('servo_max_deg', 135)
        self.declare_parameter('servo_min_deg', 45)
        self.declare_parameter('angular_gain', 30.0)  # rad/s to degrees deflection
        self.declare_parameter('linear_gain', 250.0)  # m/s to PWM delta

        # Retrieve Parameters
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

        # 2. State & Safety Variables
        self.serial_conn: Optional[serial.Serial] = None
        self.read_buffer = b""
        self.last_cmd_vel_time = 0.0
        self.target_thruster_pwm = self.neutral_pwm
        self.target_servo_deg = self.servo_center
        self.target_motor_left = 1000
        self.target_motor_right = 1000
        self.failsafe_triggered = False

        # 3. Setup Publishers
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.gps_pub = self.create_publisher(NavSatFix, '/fix', sensor_qos)
        self.heading_pub = self.create_publisher(Float32, '/compass/heading', sensor_qos)

        if HAS_CUSTOM_MSG:
            self.telemetry_pub = self.create_publisher(ASVTelemetry, '/asv/telemetry', 10)
        else:
            self.telemetry_pub = None

        self.debug_raw_pub = self.create_publisher(String, '/serial/raw_incoming', 5)

        # 4. Setup Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )

        # 5. Connect to Hardware Port
        self._connect_serial()

        # 6. Periodic Loop (Non-blocking I/O & Watchdog)
        timer_period = 1.0 / max(10.0, poll_rate)
        self.io_timer = self.create_timer(timer_period, self._serial_io_loop)
        self.get_logger().info(f"Serial Bridge Node ready at {poll_rate:.1f} Hz (Watchdog: {self.watchdog_timeout}s)")

    def _connect_serial(self) -> bool:
        """Attempts connection to microcontroller port with udev rule priority."""
        candidate_ports = [self.port_name, '/dev/ttyUSB0', '/dev/ttyACM0']

        for port in candidate_ports:
            if os.path.exists(port):
                try:
                    self.get_logger().info(f"Attempting connection to {port} @ {self.baudrate}...")
                    self.serial_conn = serial.Serial(port, self.baudrate, timeout=0.02)
                    self.serial_conn.reset_input_buffer()
                    self.serial_conn.reset_output_buffer()
                    self.get_logger().info(f"Successfully connected to MCU on {port}")
                    self.failsafe_triggered = False
                    return True
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to open port {port}: {e}")

        self.get_logger().error("No valid serial port found for microcontroller!")
        self.serial_conn = None
        return False

    def _cmd_vel_callback(self, msg: Twist):
        """Translates Nav2 Twist velocity commands into ASV actuator signals."""
        now = time.time()
        self.last_cmd_vel_time = now
        self.failsafe_triggered = False

        vx = msg.linear.x    # Forward velocity in m/s
        wz = msg.angular.z  # Turning rate in rad/s

        # Thruster PWM mapping
        pwm_delta = int(vx * self.linear_gain)
        self.target_thruster_pwm = max(self.min_pwm, min(self.max_pwm, self.neutral_pwm + pwm_delta))

        # Rudder servo mapping: wz > 0 is counter-clockwise (turn LEFT)
        deg_delta = int(wz * self.angular_gain)
        self.target_servo_deg = max(self.servo_min, min(self.servo_max, self.servo_center - deg_delta))

        # Send command immediately to minimize latency
        self._send_actuator_packet(self.target_thruster_pwm, self.target_servo_deg, 1000, 1000)

    def _send_actuator_packet(self, thruster_pwm: int, servo_deg: int, front_left: int, front_right: int):
        """Dispatches actuator commands via binary frame or legacy ASCII."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        try:
            if self.use_binary:
                # Binary Frame: [0xAA 0x55] [CMD_DRIVE] [LEN=8] [thruster(2B)] [servo(2B)] [left(2B)] [right(2B)] [CRC(2B)]
                payload = struct.pack('>HHHH', thruster_pwm, servo_deg, front_left, front_right)
                header = self.FRAME_HEADER + bytes([self.CMD_DRIVE, len(payload)])
                crc = calculate_crc16(header + payload)
                packet = header + payload + struct.pack('>H', crc)
                self.serial_conn.write(packet)
            else:
                # Legacy ASCII fallback format: "A,<servo>,<motor>,<dir>,<m_l>,<m_r>,<dir_l>,<dir_r>\n"
                direction = 1000  # 1000 = forward, 2000 = reverse
                ascii_cmd = f"A,{servo_deg},{thruster_pwm},{direction},{front_left},{front_right},1000,1000\n"
                self.serial_conn.write(ascii_cmd.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            self.serial_conn = None

    def _serial_io_loop(self):
        """Periodic loop handling non-blocking read and hardware watchdog check."""
        now = time.time()

        # 1. Hardware Watchdog Check
        if (now - self.last_cmd_vel_time) > self.watchdog_timeout:
            if not self.failsafe_triggered:
                self.get_logger().warn(
                    f"WATCHDOG FAILSAFE: No /cmd_vel for {self.watchdog_timeout}s! Stopping actuators."
                )
                self._send_actuator_packet(self.neutral_pwm, self.servo_center, 1000, 1000)
                self.failsafe_triggered = True

        # 2. Check Connection Health / Auto-Reconnect
        if self.serial_conn is None or not self.serial_conn.is_open:
            self._connect_serial()
            return

        # 3. Read Incoming Telemetry Stream
        try:
            waiting = self.serial_conn.in_waiting
            if waiting > 0:
                chunk = self.serial_conn.read(waiting)
                self.read_buffer += chunk

                # Process complete newline-terminated lines
                while b'\n' in self.read_buffer:
                    line_bytes, self.read_buffer = self.read_buffer.split(b'\n', 1)
                    line_str = line_bytes.decode('utf-8', errors='ignore').strip()
                    if line_str:
                        self._parse_incoming_line(line_str)

                # Prevent buffer bloat
                if len(self.read_buffer) > 4096:
                    self.read_buffer = b""

        except serial.SerialException as e:
            self.get_logger().error(f"Serial read exception: {e}")
            self.serial_conn = None

    def _parse_incoming_line(self, line: str):
        """Parses telemetry lines from ESP32/STM32."""
        # Publish raw line for diagnostics
        raw_msg = String()
        raw_msg.data = line
        self.debug_raw_pub.publish(raw_msg)

        # Legacy JSON telemetry format from Navantara firmware
        if line.startswith("{") and line.endswith("}"):
            try:
                import json
                data = json.loads(line)
                stamp = self.get_clock().now().to_msg()

                # Publish GPS NavSatFix
                if "lat" in data and "lon" in data:
                    lat = float(data.get("lat", 0.0))
                    lon = float(data.get("lon", 0.0))
                    if lat != 0.0 and lon != 0.0:
                        fix_msg = NavSatFix()
                        fix_msg.header.stamp = stamp
                        fix_msg.header.frame_id = 'gps_link'
                        fix_msg.latitude = lat
                        fix_msg.longitude = lon
                        fix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX if data.get("fix", 0) >= 4 else NavSatStatus.STATUS_FIX
                        self.gps_pub.publish(fix_msg)

                # Publish Heading
                if "hdg" in data:
                    hdg = float(data.get("hdg", 0.0))
                    hdg_msg = Float32()
                    hdg_msg.data = hdg
                    self.heading_pub.publish(hdg_msg)

                # Publish Telemetry
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
        """Safety shutdown: zero actuators and close port."""
        if self.serial_conn and self.serial_conn.is_open:
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
