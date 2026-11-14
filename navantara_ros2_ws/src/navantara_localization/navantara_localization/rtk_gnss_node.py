#!/usr/bin/env python3
"""
navantara_localization / rtk_gnss_node.py
=========================================
ROS 2 Driver & NMEA Parser for u-blox ZED-F9P RTK GNSS Receiver.
Part of ASV Navantara ROS 2 Architecture.

Features:
1. Parses $GNGGA, $GPGGA, $GNRMC, and $GPRMC NMEA sentences.
2. Full RTK Quality support:
   - Fix Type 4 (RTK Fixed): 1-2 cm precision (STATUS_GBAS_FIX)
   - Fix Type 5 (RTK Float): 10-30 cm precision (STATUS_GBAS_FIX)
   - Fix Type 2 (DGPS): ~1m precision (STATUS_SBAS_FIX)
   - Fix Type 1 (SPS Fix): ~2.5m precision (STATUS_FIX)
3. Computes dynamic position covariance matrix scaled by HDOP.
4. Publishes sensor_msgs/NavSatFix to /fix and True Course heading to /compass/heading.
5. Auto-reconnection & device fallback (/dev/ttyGPS -> /dev/ttyACM0 -> /dev/ttyUSB1).
"""

import math
import os
import time
from typing import Optional, Tuple

import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String


def validate_nmea_checksum(sentence: str) -> bool:
    """Verifies standard NMEA XOR checksum (*XX)."""
    if not sentence.startswith('$') or '*' not in sentence:
        return False
    try:
        star_idx = sentence.index('*')
        content = sentence[1:star_idx]
        chk_str = sentence[star_idx + 1:star_idx + 3]
        expected_chk = int(chk_str, 16)

        calc_chk = 0
        for char in content:
            calc_chk ^= ord(char)
        return calc_chk == expected_chk
    except (ValueError, IndexError):
        return False


def parse_lat_deg(val_str: str, hemi_str: str) -> Optional[float]:
    """Converts NMEA DDMM.MMMMM latitude to signed decimal degrees."""
    if not val_str or not hemi_str:
        return None
    try:
        degrees = float(val_str[:2])
        minutes = float(val_str[2:])
        dec = degrees + (minutes / 60.0)
        return -dec if hemi_str.upper() == 'S' else dec
    except (ValueError, IndexError):
        return None


def parse_lon_deg(val_str: str, hemi_str: str) -> Optional[float]:
    """Converts NMEA DDDMM.MMMMM longitude to signed decimal degrees."""
    if not val_str or not hemi_str:
        return None
    try:
        degrees = float(val_str[:3])
        minutes = float(val_str[3:])
        dec = degrees + (minutes / 60.0)
        return -dec if hemi_str.upper() == 'W' else dec
    except (ValueError, IndexError):
        return None


class ASVRTKGNSSNode(Node):
    """ROS 2 Node interfacing u-blox ZED-F9P RTK GNSS over USB serial."""

    def __init__(self):
        super().__init__('asv_rtk_gnss_node')

        # 1. Parameter Declarations
        self.declare_parameter('gps_port', '/dev/ttyGPS')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('reconnect_delay_sec', 2.0)

        self.port_param = self.get_parameter('gps_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.poll_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.reconnect_delay = self.get_parameter('reconnect_delay_sec').get_parameter_value().double_value

        # 2. QoS & Publishers
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.fix_pub = self.create_publisher(NavSatFix, '/fix', sensor_qos)
        self.heading_pub = self.create_publisher(Float32, '/compass/heading', sensor_qos)
        self.quality_pub = self.create_publisher(String, '/gps/rtk_status', 5)

        # 3. Internal State
        self.serial_conn: Optional[serial.Serial] = None
        self.read_buffer = bytearray()
        self.last_reconnect_time = 0.0
        self.active_port = self.port_param

        # Latest RTK Metrics
        self.latest_lat = 0.0
        self.latest_lon = 0.0
        self.latest_alt = 0.0
        self.latest_hdop = 99.9
        self.latest_quality = 0
        self.latest_satellites = 0
        self.latest_status_desc = "NO_CONNECTION"

        # 4. Timer for Serial Polling
        timer_period = 1.0 / max(1.0, self.poll_rate)
        self.poll_timer = self.create_timer(timer_period, self._poll_serial_stream)

        self.get_logger().info(
            f"RTK GNSS Node initialized. Target port: {self.port_param} @ {self.baudrate} baud (Frame: {self.frame_id})"
        )

    def _get_candidate_ports(self):
        """Returns ordered list of candidate serial ports for u-blox GNSS."""
        candidates = [self.port_param]
        fallbacks = [
            '/dev/ttyGPS',
            '/dev/ttyACM0',
            '/dev/ttyACM1',
            '/dev/ttyUSB1',
            '/dev/ttyUSB0',
        ]
        for fb in fallbacks:
            if fb not in candidates:
                candidates.append(fb)
        return candidates

    def _connect_serial(self):
        """Attempts connection to active port or candidate fallbacks."""
        now = time.time()
        if (now - self.last_reconnect_time) < self.reconnect_delay:
            return

        self.last_reconnect_time = now

        for candidate in self._get_candidate_ports():
            if not os.path.exists(candidate):
                continue

            try:
                self.serial_conn = serial.Serial(
                    port=candidate,
                    baudrate=self.baudrate,
                    timeout=0.05,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                self.active_port = candidate
                self.get_logger().info(f"Successfully opened RTK GNSS on {candidate} @ {self.baudrate} baud")
                self.read_buffer.clear()
                return
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f"Failed opening candidate port {candidate}: {e}")
                self.serial_conn = None

    def _poll_serial_stream(self):
        """Reads NMEA sentences from hardware serial and publishes ROS 2 topics."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self._connect_serial()
            return

        try:
            waiting = self.serial_conn.in_waiting
            if waiting > 0:
                chunk = self.serial_conn.read(waiting)
                self.read_buffer.extend(chunk)

                # Process all complete lines
                while b'\n' in self.read_buffer:
                    line_end_idx = self.read_buffer.index(b'\n')
                    raw_line = self.read_buffer[:line_end_idx].decode('ascii', errors='ignore').strip()
                    del self.read_buffer[:line_end_idx + 1]

                    if raw_line.startswith('$'):
                        self._parse_nmea_sentence(raw_line)

                # Prevent unbounded buffer growth
                if len(self.read_buffer) > 4096:
                    self.read_buffer.clear()

        except serial.SerialException as e:
            self.get_logger().error(f"Serial port {self.active_port} error: {e}. Reconnecting...")
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

    def _parse_nmea_sentence(self, sentence: str):
        """Parses GGA (Fix/RTK) and RMC (Course/Heading) sentences."""
        if not validate_nmea_checksum(sentence):
            return

        parts = sentence.split('*')[0].split(',')
        talker_type = parts[0]

        # 1. Parse GGA (Fix Data, Satellites, HDOP, Altitude, RTK Mode)
        # Formats: $GNGGA, $GPGGA
        if talker_type.endswith('GGA') and len(parts) >= 15:
            self._parse_gga(parts)

        # 2. Parse RMC (Recommended Minimum, Speed, Course Over Ground)
        # Formats: $GNRMC, $GPRMC
        elif talker_type.endswith('RMC') and len(parts) >= 12:
            self._parse_rmc(parts)

    def _parse_gga(self, parts: list):
        """Extracts latitude, longitude, RTK fix quality, altitude, and covariance."""
        lat_str = parts[2]
        lat_hemi = parts[3]
        lon_str = parts[4]
        lon_hemi = parts[5]
        quality_str = parts[6]
        sats_str = parts[7]
        hdop_str = parts[8]
        alt_str = parts[9]

        lat = parse_lat_deg(lat_str, lat_hemi)
        lon = parse_lon_deg(lon_str, lon_hemi)

        if lat is None or lon is None:
            return

        quality = int(quality_str) if quality_str.isdigit() else 0
        num_sats = int(sats_str) if sats_str.isdigit() else 0
        hdop = float(hdop_str) if hdop_str else 99.9
        alt = float(alt_str) if alt_str else 0.0

        self.latest_lat = lat
        self.latest_lon = lon
        self.latest_alt = alt
        self.latest_hdop = hdop
        self.latest_quality = quality
        self.latest_satellites = num_sats

        stamp = self.get_clock().now().to_msg()
        fix_msg = NavSatFix()
        fix_msg.header.stamp = stamp
        fix_msg.header.frame_id = self.frame_id
        fix_msg.latitude = float(lat)
        fix_msg.longitude = float(lon)
        fix_msg.altitude = float(alt)

        # Map NMEA quality indicator to ROS 2 NavSatStatus and realistic covariance
        # Quality: 0=Invalid, 1=SPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
        status = fix_msg.status
        status.service = (
            NavSatStatus.SERVICE_GPS |
            NavSatStatus.SERVICE_GLONASS |
            NavSatStatus.SERVICE_GALILEO
        )

        desc = "NO_FIX"
        if quality == 4:
            # RTK Fixed: 1-2 cm baseline accuracy
            status.status = NavSatStatus.STATUS_GBAS_FIX
            sigma_h = max(0.015, 0.015 * hdop)
            sigma_v = 2.0 * sigma_h
            desc = f"RTK_FIXED (Sats:{num_sats}, HDOP:{hdop:.2f})"
        elif quality == 5:
            # RTK Float: 10-30 cm baseline accuracy
            status.status = NavSatStatus.STATUS_GBAS_FIX
            sigma_h = max(0.20, 0.20 * hdop)
            sigma_v = 2.0 * sigma_h
            desc = f"RTK_FLOAT (Sats:{num_sats}, HDOP:{hdop:.2f})"
        elif quality == 2:
            # DGPS / SBAS
            status.status = NavSatStatus.STATUS_SBAS_FIX
            sigma_h = max(0.8, 0.8 * hdop)
            sigma_v = 2.0 * sigma_h
            desc = f"DGPS_FIX (Sats:{num_sats}, HDOP:{hdop:.2f})"
        elif quality == 1:
            # Standard Autonomous SPS Fix
            status.status = NavSatStatus.STATUS_FIX
            sigma_h = max(2.5, 2.5 * hdop)
            sigma_v = 2.5 * sigma_h
            desc = f"GPS_SPS_FIX (Sats:{num_sats}, HDOP:{hdop:.2f})"
        else:
            status.status = NavSatStatus.STATUS_NO_FIX
            sigma_h = 100.0
            sigma_v = 100.0
            desc = "SEARCHING_FOR_SATELLITES"

        self.latest_status_desc = desc

        # Diagonal covariance: [var_x, 0, 0, 0, var_y, 0, 0, 0, var_z]
        var_h = float(sigma_h * sigma_h)
        var_v = float(sigma_v * sigma_v)
        fix_msg.position_covariance = [
            var_h, 0.0, 0.0,
            0.0, var_h, 0.0,
            0.0, 0.0, var_v
        ]
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.fix_pub.publish(fix_msg)

        # Publish diagnostic quality string
        q_msg = String()
        q_msg.data = desc
        self.quality_pub.publish(q_msg)

    def _parse_rmc(self, parts: list):
        """Extracts Course Over Ground (heading in deg True) and Ground Speed."""
        # Parts: [0: $xxRMC, 1: time, 2: status (A/V), 3: lat, 4: N/S, 5: lon, 6: E/W,
        #         7: speed_knots, 8: track_true_deg, 9: date, 10: mag_var, ...]
        status_flag = parts[2]
        if status_flag.upper() != 'A':
            return  # V = Navigation receiver warning (invalid)

        speed_knots_str = parts[7]
        track_deg_str = parts[8]

        if track_deg_str:
            try:
                course_deg = float(track_deg_str)
                speed_knots = float(speed_knots_str) if speed_knots_str else 0.0

                # Only publish COG as vessel heading if moving faster than 0.3 knots (~0.15 m/s)
                # to prevent heading noise when stationary.
                if speed_knots >= 0.3:
                    hdg_msg = Float32()
                    hdg_msg.data = float(course_deg)
                    self.heading_pub.publish(hdg_msg)
            except ValueError:
                pass

    def destroy_node(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ASVRTKGNSSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
