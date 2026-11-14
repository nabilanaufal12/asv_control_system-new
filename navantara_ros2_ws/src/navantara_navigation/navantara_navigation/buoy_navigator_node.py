#!/usr/bin/env python3
"""
navantara_navigation / buoy_navigator_node.py
=============================================
Autonomous Buoy Gate Navigator & 2D Rolling Spatial Memory Costmap.
Part of ASV Navantara ROS 2 Architecture for KKI Autonomous Racing.

Features:
1. 4.0-Second Spatial Memory Buffer:
   - Transforms /perception/obstacles from base_link to odom frame using vessel odometry.
   - Retains buoy positions for 4.0 seconds across wave pitch/roll occlusions.
2. Intelligent Gate Midpoint & Pure Pursuit Control:
   - Pairs closest Red (Port) and Green (Starboard) buoys to compute gate passage midpoint.
   - Falls back to safe ±1.8m lateral offset passing if only one buoy color is visible.
   - Computes curvature and publishes smooth geometry_msgs/Twist to /cmd_vel.
3. 2D Rolling Costmap (OccupancyGrid):
   - Publishes 10 Hz 20m x 20m costmap (0.1m res) on /local_costmap/costmap.
   - Stamps lethal costs (100) and inflation gradients around all remembered buoys.
4. RViz2 Visual Markers:
   - Publishes visualization_msgs/MarkerArray on /navigation/markers (spheres, gate line, target arrow).
"""

import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

try:
    from navantara_interfaces.msg import Obstacle, ObstacleArray
    HAS_CUSTOM_MSG = True
except ImportError:
    HAS_CUSTOM_MSG = False


class RememberedBuoy:
    """Represents a buoy tracked in world/odom coordinates."""
    def __init__(self, label: str, x: float, y: float, confidence: float):
        self.label = label
        self.x = x
        self.y = y
        self.confidence = confidence
        self.last_seen = time.time()
        self.seen_count = 1

    def update(self, x: float, y: float, confidence: float):
        # Exponential moving filter for stable position
        self.x = 0.7 * self.x + 0.3 * x
        self.y = 0.7 * self.y + 0.3 * y
        self.confidence = max(self.confidence, confidence)
        self.last_seen = time.time()
        self.seen_count += 1


class ASVBuoyNavigatorNode(Node):
    """Autonomous navigation node steering ASV Navantara through buoy gates."""

    def __init__(self):
        super().__init__('asv_buoy_navigator_node')

        # 1. Parameter Declarations
        self.declare_parameter('persistence_timeout_sec', 4.0)
        self.declare_parameter('base_cruise_speed_mps', 1.4)
        self.declare_parameter('min_turn_speed_mps', 0.6)
        self.declare_parameter('max_yaw_rate_radps', 0.8)
        self.declare_parameter('lookahead_min_m', 2.5)
        self.declare_parameter('lookahead_max_m', 7.0)
        self.declare_parameter('single_buoy_offset_m', 1.8)
        self.declare_parameter('costmap_size_m', 20.0)
        self.declare_parameter('costmap_resolution_m', 0.1)
        self.declare_parameter('costmap_inflation_radius_m', 1.2)
        self.declare_parameter('control_rate_hz', 20.0)

        # Retrieve Parameters
        self.persistence_timeout = self.get_parameter('persistence_timeout_sec').value
        self.base_speed = self.get_parameter('base_cruise_speed_mps').value
        self.min_turn_speed = self.get_parameter('min_turn_speed_mps').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate_radps').value
        self.lookahead_min = self.get_parameter('lookahead_min_m').value
        self.lookahead_max = self.get_parameter('lookahead_max_m').value
        self.single_buoy_offset = self.get_parameter('single_buoy_offset_m').value
        self.costmap_size = self.get_parameter('costmap_size_m').value
        self.costmap_res = self.get_parameter('costmap_resolution_m').value
        self.costmap_inflation = self.get_parameter('costmap_inflation_radius_m').value
        rate_hz = self.get_parameter('control_rate_hz').value

        # 2. Vessel State Tracking (odom frame)
        self.asv_x = 0.0
        self.asv_y = 0.0
        self.asv_yaw = 0.0
        self.asv_speed = 0.0
        self.odom_received = False

        # Spatial Memory Storage: List of RememberedBuoy
        self.buoy_memory: List[RememberedBuoy] = []
        self.buoy_match_dist = 1.2  # Clustering tolerance in meters

        # Target Navigation Waypoint (odom frame)
        self.target_x = 5.0
        self.target_y = 0.0
        self.current_nav_mode = "SEARCHING"
        self.gates_cleared_count = 0

        # 3. Setup Publishers
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/navigation/markers', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', reliable_qos)

        # 4. Setup Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/raw', self._odom_callback, reliable_qos
        )

        if HAS_CUSTOM_MSG:
            self.obs_sub = self.create_subscription(
                ObstacleArray, '/perception/obstacles', self._obstacles_callback, 10
            )

        # 5. Periodic Control & Costmap Timer
        timer_period = 1.0 / max(5.0, rate_hz)
        self.control_timer = self.create_timer(timer_period, self._control_loop)
        self.get_logger().info(
            f"Buoy Navigator & Spatial Costmap Node initialized at {rate_hz:.1f} Hz (Memory Timeout: {self.persistence_timeout}s)"
        )

    def _odom_callback(self, msg: Odometry):
        """Extracts position and yaw angle from vessel odometry."""
        self.asv_x = msg.pose.pose.position.x
        self.asv_y = msg.pose.pose.position.y
        self.asv_speed = msg.twist.twist.linear.x

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.asv_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def _obstacles_callback(self, msg):
        """Transforms detected 2D/3D obstacles into global odom coordinates and stores in spatial memory."""
        if not self.odom_received:
            return

        now = time.time()
        cos_yaw = math.cos(self.asv_yaw)
        sin_yaw = math.sin(self.asv_yaw)

        for obs in msg.obstacles:
            # Transform relative base_link (x_fwd, y_left) to odom
            x_rel = obs.position.x
            y_rel = obs.position.y

            x_odom = self.asv_x + (x_rel * cos_yaw - y_rel * sin_yaw)
            y_odom = self.asv_y + (x_rel * sin_yaw + y_rel * cos_yaw)

            # Match with existing remembered buoy or add new
            matched = False
            for buoy in self.buoy_memory:
                # Same label and spatial proximity
                if buoy.label == obs.label:
                    dist = math.hypot(buoy.x - x_odom, buoy.y - y_odom)
                    if dist < self.buoy_match_dist:
                        buoy.update(x_odom, y_odom, obs.confidence)
                        matched = True
                        break

            if not matched:
                self.buoy_memory.append(RememberedBuoy(obs.label, x_odom, y_odom, obs.confidence))

    def _clean_spatial_memory(self):
        """Prunes buoys older than 4.0 seconds from spatial memory."""
        now = time.time()
        self.buoy_memory = [
            b for b in self.buoy_memory if (now - b.last_seen) <= self.persistence_timeout
        ]

    def _control_loop(self):
        """Executes gate finding, Pure Pursuit steering, costmap generation, and marker broadcasting."""
        if not self.odom_received:
            return

        self._clean_spatial_memory()

        # 1. Transform active remembered buoys to local vessel frame for gate detection
        cos_yaw = math.cos(self.asv_yaw)
        sin_yaw = math.sin(self.asv_yaw)

        red_buoys_ahead = []
        green_buoys_ahead = []
        blue_buoys_ahead = []

        for b in self.buoy_memory:
            dx = b.x - self.asv_x
            dy = b.y - self.asv_y
            x_v = dx * cos_yaw + dy * sin_yaw      # Forward along bow
            y_v = -dx * sin_yaw + dy * cos_yaw     # Lateral (left is +)

            # Only consider buoys ahead or slightly abreast (x_v > -0.5m)
            if x_v > -0.5:
                dist = math.hypot(x_v, y_v)
                lbl = b.label.lower()
                if 'merah' in lbl or 'red' in lbl:
                    red_buoys_ahead.append((dist, x_v, y_v, b))
                elif 'hijau' in lbl or 'green' in lbl:
                    green_buoys_ahead.append((dist, x_v, y_v, b))
                elif 'biru' in lbl or 'blue' in lbl:
                    blue_buoys_ahead.append((dist, x_v, y_v, b))

        # Sort by distance
        red_buoys_ahead.sort(key=lambda item: item[0])
        green_buoys_ahead.sort(key=lambda item: item[0])
        blue_buoys_ahead.sort(key=lambda item: item[0])

        active_gate_pair = None

        # 2. Compute Guidance Target Point
        if red_buoys_ahead and green_buoys_ahead:
            # === CASE A: BOTH RED & GREEN BUOYS VISIBLE (DUAL GATE PASSAGE) ===
            _, xr_v, yr_v, b_red = red_buoys_ahead[0]
            _, xg_v, yg_v, b_green = green_buoys_ahead[0]

            # Gate Midpoint in vessel frame
            mid_xv = (xr_v + xg_v) / 2.0
            mid_yv = (yr_v + yg_v) / 2.0

            # If a Blue obstacle sits near the gate center, nudge waypoint away from it
            if blue_buoys_ahead:
                _, xb_v, yb_v, _ = blue_buoys_ahead[0]
                dist_mid_to_blue = math.hypot(xb_v - mid_xv, yb_v - mid_yv)
                if dist_mid_to_blue < 1.4:
                    # Nudge away from blue towards the side with more clearance
                    shift_dir = -1.0 if yb_v >= mid_yv else 1.0
                    mid_yv += shift_dir * 0.9

            # Transform midpoint back to odom
            self.target_x = self.asv_x + (mid_xv * cos_yaw - mid_yv * sin_yaw)
            self.target_y = self.asv_y + (mid_xv * sin_yaw + mid_yv * cos_yaw)
            self.current_nav_mode = "DUAL_GATE_TRACKING"
            active_gate_pair = (b_red, b_green)

        elif red_buoys_ahead:
            # === CASE B: ONLY RED BUOY VISIBLE (PORT BUOY) ===
            # Red buoy must be left on port side -> steer to the starboard (right) side by single_buoy_offset
            _, xr_v, yr_v, b_red = red_buoys_ahead[0]
            target_xv = xr_v
            target_yv = yr_v - self.single_buoy_offset  # Shift to starboard (-Y in vessel frame)

            self.target_x = self.asv_x + (target_xv * cos_yaw - target_yv * sin_yaw)
            self.target_y = self.asv_y + (target_xv * sin_yaw + target_yv * cos_yaw)
            self.current_nav_mode = "PASSING_RED_PORT"

        elif green_buoys_ahead:
            # === CASE C: ONLY GREEN BUOY VISIBLE (STARBOARD BUOY) ===
            # Green buoy must be left on starboard side -> steer to the port (left) side by single_buoy_offset
            _, xg_v, yg_v, b_green = green_buoys_ahead[0]
            target_xv = xg_v
            target_yv = yg_v + self.single_buoy_offset  # Shift to port (+Y in vessel frame)

            self.target_x = self.asv_x + (target_xv * cos_yaw - target_yv * sin_yaw)
            self.target_y = self.asv_y + (target_xv * sin_yaw + target_yv * cos_yaw)
            self.current_nav_mode = "PASSING_GREEN_STARBOARD"

        elif blue_buoys_ahead:
            # === CASE D: ONLY BLUE OBSTACLE VISIBLE AHEAD (CORRIDOR OBSTACLE) ===
            # Avoid blue obstacle: nudge laterally away from its center
            _, xb_v, yb_v, _ = blue_buoys_ahead[0]
            target_xv = xb_v
            # If blue obstacle is on left or center (yb_v >= 0), pass on starboard (-Y); else pass on port (+Y)
            offset = -self.single_buoy_offset if yb_v >= 0.0 else self.single_buoy_offset
            target_yv = yb_v + offset

            self.target_x = self.asv_x + (target_xv * cos_yaw - target_yv * sin_yaw)
            self.target_y = self.asv_y + (target_xv * sin_yaw + target_yv * cos_yaw)
            self.current_nav_mode = "AVOIDING_BLUE_OBSTACLE"

        else:
            # === CASE E: NO BUOYS IN SIGHT (CRUISING FORWARD) ===
            # Drive straight forward along current heading
            self.target_x = self.asv_x + 6.0 * cos_yaw
            self.target_y = self.asv_y + 6.0 * sin_yaw
            self.current_nav_mode = "CRUISING_SEARCH"

        # 3. Pure Pursuit Control Law
        dx_tgt = self.target_x - self.asv_x
        dy_tgt = self.target_y - self.asv_y
        dist_tgt = math.hypot(dx_tgt, dy_tgt)

        # Local target coordinates
        x_tgt_v = dx_tgt * cos_yaw + dy_tgt * sin_yaw
        y_tgt_v = -dx_tgt * sin_yaw + dy_tgt * cos_yaw

        # Heading angle error alpha
        alpha = math.atan2(y_tgt_v, max(0.5, x_tgt_v))

        # Lookahead distance bounded
        L = np.clip(dist_tgt, self.lookahead_min, self.lookahead_max)

        # Curvature: kappa = 2 * sin(alpha) / L
        curvature = (2.0 * math.sin(alpha)) / L

        # Dynamic velocity: slow down on sharp turns
        turn_factor = max(0.2, math.cos(alpha))
        cmd_linear = float(np.clip(self.base_speed * turn_factor, self.min_turn_speed, self.base_speed))
        cmd_angular = float(np.clip(cmd_linear * curvature * 1.5, -self.max_yaw_rate, self.max_yaw_rate))

        # Publish cmd_vel
        cmd_msg = Twist()
        cmd_msg.linear.x = cmd_linear
        cmd_msg.angular.z = cmd_angular
        self.cmd_vel_pub.publish(cmd_msg)

        # 4. Generate & Publish Real-Time Rolling Costmap
        self._publish_rolling_costmap()

        # 5. Broadcast RViz2 Visual Markers
        self._publish_visual_markers(active_gate_pair)

    def _publish_rolling_costmap(self):
        """Generates a 2D occupancy grid centered on the ASV stamping remembered buoys."""
        grid_dim = int(self.costmap_size / self.costmap_res)
        grid = np.zeros((grid_dim, grid_dim), dtype=np.int8)

        origin_x = self.asv_x - (self.costmap_size / 2.0)
        origin_y = self.asv_y - (self.costmap_size / 2.0)

        r_lethal = 0.35
        r_infl = self.costmap_inflation

        for b in self.buoy_memory:
            # Check if inside grid bounds
            if (origin_x <= b.x <= origin_x + self.costmap_size and
                origin_y <= b.y <= origin_y + self.costmap_size):

                bx_idx = int((b.x - origin_x) / self.costmap_res)
                by_idx = int((b.y - origin_y) / self.costmap_res)
                cell_radius = int(r_infl / self.costmap_res)

                x_min = max(0, bx_idx - cell_radius)
                x_max = min(grid_dim, bx_idx + cell_radius + 1)
                y_min = max(0, by_idx - cell_radius)
                y_max = min(grid_dim, by_idx + cell_radius + 1)

                for ix in range(x_min, x_max):
                    for iy in range(y_min, y_max):
                        dist_m = math.hypot(
                            (ix - bx_idx) * self.costmap_res,
                            (iy - by_idx) * self.costmap_res
                        )
                        if dist_m <= r_lethal:
                            grid[iy, ix] = max(grid[iy, ix], 100)  # Lethal cost
                        elif dist_m <= r_infl:
                            cost = int(99 * (1.0 - (dist_m - r_lethal) / (r_infl - r_lethal)))
                            grid[iy, ix] = max(grid[iy, ix], cost)

        # Build message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = float(self.costmap_res)
        msg.info.width = grid_dim
        msg.info.height = grid_dim
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.position.z = -0.05
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()

        self.costmap_pub.publish(msg)

    def _publish_visual_markers(self, active_gate: Optional[Tuple[RememberedBuoy, RememberedBuoy]]):
        """Broadcasts markers for remembered buoys, gate connecting line, and guidance target."""
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        marker_id = 0

        # 1. Markers for all buoys in 4-second spatial memory
        for b in self.buoy_memory:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = 'odom'
            m.ns = 'remembered_buoys'
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(b.x)
            m.pose.position.y = float(b.y)
            m.pose.position.z = 0.15
            m.scale = Vector3(x=0.45, y=0.45, z=0.45)

            lbl = b.label.lower()
            if 'merah' in lbl or 'red' in lbl:
                m.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.95)
            elif 'hijau' in lbl or 'green' in lbl:
                m.color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=0.95)
            elif 'biru' in lbl or 'blue' in lbl:
                m.color = ColorRGBA(r=0.1, g=0.3, b=1.0, a=0.95)
            else:
                m.color = ColorRGBA(r=1.0, g=0.8, b=0.1, a=0.95)

            # Fade older buoys
            age = time.time() - b.last_seen
            fade = max(0.3, 1.0 - (age / self.persistence_timeout))
            m.color.a = float(fade)
            markers.markers.append(m)

        # 2. Gate Passage Line (connecting paired Red and Green buoys)
        if active_gate:
            b_red, b_green = active_gate
            gate_line = Marker()
            gate_line.header.stamp = stamp
            gate_line.header.frame_id = 'odom'
            gate_line.ns = 'active_gate'
            gate_line.id = 100
            gate_line.type = Marker.LINE_STRIP
            gate_line.action = Marker.ADD
            gate_line.scale.x = 0.12  # Line thickness
            gate_line.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.9)  # Cyan

            p_red = Point(x=float(b_red.x), y=float(b_red.y), z=0.15)
            p_green = Point(x=float(b_green.x), y=float(b_green.y), z=0.15)
            gate_line.points = [p_red, p_green]
            markers.markers.append(gate_line)

        # 3. Target Waypoint Arrow
        tgt_marker = Marker()
        tgt_marker.header.stamp = stamp
        tgt_marker.header.frame_id = 'odom'
        tgt_marker.ns = 'target_waypoint'
        tgt_marker.id = 200
        tgt_marker.type = Marker.ARROW
        tgt_marker.action = Marker.ADD

        p_start = Point(x=float(self.asv_x), y=float(self.asv_y), z=0.2)
        p_end = Point(x=float(self.target_x), y=float(self.target_y), z=0.2)
        tgt_marker.points = [p_start, p_end]
        tgt_marker.scale = Vector3(x=0.12, y=0.25, z=0.3)
        tgt_marker.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.95)  # Bright Yellow
        markers.markers.append(tgt_marker)

        # 4. Text Status Billboard above vessel
        status_text = Marker()
        status_text.header.stamp = stamp
        status_text.header.frame_id = 'odom'
        status_text.ns = 'asv_status'
        status_text.id = 300
        status_text.type = Marker.TEXT_VIEW_FACING
        status_text.action = Marker.ADD
        status_text.pose.position.x = float(self.asv_x)
        status_text.pose.position.y = float(self.asv_y)
        status_text.pose.position.z = 1.0
        status_text.scale.z = 0.4
        status_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        status_text.text = f"MODE: {self.current_nav_mode}\nSPEED: {self.asv_speed:.1f} m/s | MEM BUOYS: {len(self.buoy_memory)}"
        markers.markers.append(status_text)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ASVBuoyNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
