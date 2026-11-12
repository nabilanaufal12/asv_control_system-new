#!/usr/bin/env python3
"""
navantara_perception / vision_node.py
=====================================
ROS2 Node for Real-time Buoy & Obstacle Perception.
Part of ASV Navantara ROS2 Architecture.

Supports:
1. Production Mode: TensorRT FP16 .engine inference on NVIDIA Jetson GPU.
2. Local Laptop / Mock Mode: Auto-fallback or `simulation_mode:=True` synthetic buoy
   generator and OpenCV CPU inference for local development without hardware.
3. Hybrid Distance Estimation: Combines pinhole width with pitch-compensated ground-plane
   ray projection for robust metric distance estimation.
"""

import math
import struct
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    from navantara_interfaces.msg import Obstacle, ObstacleArray
    HAS_CUSTOM_MSG = True
except ImportError:
    HAS_CUSTOM_MSG = False

try:
    from ultralytics import YOLO
    HAS_ULTRALYTICS = True
except ImportError:
    HAS_ULTRALYTICS = False


class ASVVisionNode(Node):
    """ROS2 node wrapping TensorRT inference and 3D spatial obstacle projection."""

    def __init__(self):
        super().__init__('asv_vision_node')

        # 1. Parameter Declarations
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('model_path', 'best.engine')
        self.declare_parameter('camera_source', 0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_topic_input', False)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('camera_hfov_deg', 70.0)
        self.declare_parameter('camera_vfov_deg', 55.0)
        self.declare_parameter('camera_height_m', 0.45)        # Height above water line
        self.declare_parameter('camera_pitch_deg', 5.0)        # Downward tilt angle
        self.declare_parameter('target_frame_id', 'base_link')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('inference_rate_hz', 25.0)

        # Buoy physical size constants (diameter/width in meters)
        self.declare_parameter('buoy_ball_diameter_m', 0.30)
        self.declare_parameter('buoy_box_width_m', 0.35)

        # 2. Retrieve Parameters
        self.sim_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().integer_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.use_topic_input = self.get_parameter('use_topic_input').get_parameter_value().bool_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_thresh = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.frame_w = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_h = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.hfov_deg = self.get_parameter('camera_hfov_deg').get_parameter_value().double_value
        self.vfov_deg = self.get_parameter('camera_vfov_deg').get_parameter_value().double_value
        self.cam_height = self.get_parameter('camera_height_m').get_parameter_value().double_value
        self.cam_pitch_rad = math.radians(self.get_parameter('camera_pitch_deg').get_parameter_value().double_value)
        self.target_frame = self.get_parameter('target_frame_id').get_parameter_value().string_value
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        rate_hz = self.get_parameter('inference_rate_hz').get_parameter_value().double_value

        self.ball_diameter = self.get_parameter('buoy_ball_diameter_m').get_parameter_value().double_value
        self.box_width = self.get_parameter('buoy_box_width_m').get_parameter_value().double_value

        # Calculate focal lengths in pixels
        self.fx = (self.frame_w / 2.0) / math.tan(math.radians(self.hfov_deg / 2.0))
        self.fy = (self.frame_h / 2.0) / math.tan(math.radians(self.vfov_deg / 2.0))
        self.cx = self.frame_w / 2.0
        self.cy = self.frame_h / 2.0

        # Synthetic buoy simulation states
        self.sim_tick = 0.0

        # 3. Setup Publishers
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        if HAS_CUSTOM_MSG:
            self.obstacle_pub = self.create_publisher(ObstacleArray, '/perception/obstacles', 10)
        else:
            self.obstacle_pub = None

        self.pointcloud_pub = self.create_publisher(PointCloud2, '/perception/obstacle_points', sensor_qos)

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, '/perception/image_annotated', 2)

        self.bridge = CvBridge() if CvBridge else None

        # 4. Initialize Perception Engine (Hardware TensorRT or Mock Fallback)
        self.model = None
        self.engine_ready = False
        if not self.sim_mode:
            self._load_engine()

        if self.sim_mode or not self.engine_ready:
            self.get_logger().info(
                "Vision Node operating in LAPTOP MOCK / SIMULATION mode. Generating synthetic buoy detections."
            )
            self.sim_mode = True

        # 5. Input Mode Configuration
        self.cap = None
        if not self.sim_mode:
            if self.use_topic_input:
                self.image_sub = self.create_subscription(
                    Image, self.image_topic, self._image_callback, sensor_qos
                )
                self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
            else:
                self._init_camera_capture()

        timer_period = 1.0 / max(1.0, rate_hz)
        self.timer = self.create_timer(timer_period, self._main_loop_timer)
        self.get_logger().info(f"Vision Node initialized at {rate_hz:.1f} Hz (Frame ID: {self.target_frame})")

    def _load_engine(self):
        """Attempts loading TensorRT .engine with PyTorch CUDA checks."""
        if not HAS_ULTRALYTICS:
            self.get_logger().warn("Ultralytics library not available. Switching to simulation mode.")
            return

        try:
            import torch
            if not torch.cuda.is_available():
                self.get_logger().warn("CUDA is not available on this system. Falling back to mock vision mode.")
                return
        except Exception:
            return

        model_file = Path(self.model_path)
        if not model_file.exists():
            candidates = [
                Path("src/navantara_backend/vision/best.engine"),
                Path("best.engine"),
                Path("best.pt"),
            ]
            for cand in candidates:
                if cand.exists():
                    model_file = cand
                    break

        if model_file.exists():
            try:
                self.get_logger().info(f"Loading TensorRT/YOLO model from: {model_file}")
                self.model = YOLO(str(model_file), task='detect')
                self.engine_ready = True
                self.get_logger().info("Model successfully loaded into GPU memory.")
            except Exception as e:
                self.get_logger().warn(f"Failed to load engine on current device ({e}). Fallback to simulation.")
        else:
            self.get_logger().warn(f"Model file '{self.model_path}' not found. Fallback to simulation.")

    def _init_camera_capture(self):
        """Initializes hardware camera with V4L2 backend."""
        self.cap = cv2.VideoCapture(self.camera_source, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_source)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
            self.get_logger().info(f"Hardware camera opened at /dev/video{self.camera_source}")
        else:
            self.get_logger().warn(f"Cannot open camera device /dev/video{self.camera_source}. Fallback to mock.")
            self.sim_mode = True

    def _main_loop_timer(self):
        """Periodic perception callback."""
        if self.sim_mode:
            self._generate_synthetic_buoy_stream()
            return

        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                self._process_frame(frame)
            else:
                self.get_logger().warn("Dropped frame from hardware camera.")

    def _image_callback(self, msg: Image):
        if self.sim_mode or self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")

    def _calculate_hybrid_distance(self, bw: float, y2: float, label: str) -> float:
        """
        Calculates distance by fusing:
        1. Pinhole horizontal bounding box width (size-based).
        2. Vertical base (y2) pitch-compensated water plane raycast (geometry-based).
        """
        real_dim = self.box_width if 'box' in label.lower() or 'kotak' in label.lower() else self.ball_diameter

        # 1. Pinhole width distance
        d_width = (self.fx * real_dim) / max(1.0, bw)

        # 2. Ground-plane raycast distance
        # Elevation angle below camera optical axis
        alpha = math.atan2(y2 - self.cy, self.fy)
        total_angle = self.cam_pitch_rad + alpha

        if total_angle > 0.05:  # Looking downwards at water plane
            d_ground = self.cam_height / math.tan(total_angle)
            weight_ground = float(np.clip((d_ground - 2.0) / 10.0, 0.2, 0.6))
            d_fused = (1.0 - weight_ground) * d_width + weight_ground * d_ground
        else:
            d_fused = d_width

        return float(np.clip(d_fused, 0.5, 35.0))

    def _process_frame(self, frame: np.ndarray):
        """Runs TensorRT GPU inference and projects detections."""
        if self.model is None:
            return

        h, w, _ = frame.shape
        stamp = self.get_clock().now().to_msg()

        results = self.model.predict(
            source=frame,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            verbose=False,
            device=0
        )

        detected_obstacles = []
        cloud_points = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                label = self.model.names[cls_id] if hasattr(self.model, 'names') else str(cls_id)

                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = xyxy
                bw = max(1.0, x2 - x1)
                bh = max(1.0, y2 - y1)
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                dist_m = self._calculate_hybrid_distance(bw, y2, label)

                norm_x = (cx - self.cx) / self.cx
                bearing_rad = norm_x * math.radians(self.hfov_deg / 2.0)
                bearing_deg = math.degrees(bearing_rad)

                pos_x = dist_m * math.cos(bearing_rad)
                pos_y = -dist_m * math.sin(bearing_rad)
                pos_z = 0.0

                cloud_points.append((pos_x, pos_y, pos_z))

                if HAS_CUSTOM_MSG:
                    obs = Obstacle()
                    obs.header.stamp = stamp
                    obs.header.frame_id = self.target_frame
                    obs.label = label
                    obs.confidence = conf
                    obs.position.x = float(pos_x)
                    obs.position.y = float(pos_y)
                    obs.position.z = float(pos_z)
                    obs.distance_m = float(dist_m)
                    obs.bearing_deg = float(bearing_deg)
                    obs.bbox_center_x = float(cx)
                    obs.bbox_center_y = float(cy)
                    obs.bbox_width = float(bw)
                    obs.bbox_height = float(bh)
                    detected_obstacles.append(obs)

                if self.publish_annotated:
                    color = (0, 255, 0) if 'hijau' in label.lower() or 'green' in label.lower() else (0, 0, 255)
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    text = f"{label} {dist_m:.1f}m {bearing_deg:+.1f}deg"
                    cv2.putText(frame, text, (int(x1), max(20, int(y1) - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

        self._publish_perception_data(stamp, detected_obstacles, cloud_points, frame)

    def _generate_synthetic_buoy_stream(self):
        """
        Generates simulated buoys in front of base_link for local testing on laptops.
        Creates a pair of buoys (Red on Port/Left, Green on Starboard/Right) that
        shift smoothly with time to simulate vessel movement along a slalom course.
        """
        self.sim_tick += 0.05
        stamp = self.get_clock().now().to_msg()

        # Canvas for synthetic image
        sim_frame = np.zeros((self.frame_h, self.frame_w, 3), dtype=np.uint8)
        horizon_y = int(self.cy - 30)
        sim_frame[:horizon_y, :] = (210, 180, 140)  # Sky
        sim_frame[horizon_y:, :] = (80, 45, 20)     # Water

        # Buoy 1: Red Buoy (Port side: Y > 0 in base_link)
        dist_red = 10.0 + 4.0 * math.sin(self.sim_tick * 0.3)
        y_red = 2.5 + 0.5 * math.cos(self.sim_tick * 0.2)
        bearing_red = math.atan2(y_red, dist_red)

        # Buoy 2: Green Buoy (Starboard side: Y < 0 in base_link)
        dist_green = 10.0 + 4.0 * math.cos(self.sim_tick * 0.3)
        y_green = -2.5 - 0.5 * math.sin(self.sim_tick * 0.2)
        bearing_green = math.atan2(y_green, dist_green)

        synthetic_buoys = [
            ("bola-merah", dist_red, y_red, bearing_red, (0, 0, 240)),
            ("bola-hijau", dist_green, y_green, bearing_green, (0, 240, 0)),
        ]

        detected_obstacles = []
        cloud_points = []

        for label, dist, y_lateral, bearing_rad, bgr_color in synthetic_buoys:
            pos_x = dist * math.cos(bearing_rad)
            pos_y = dist * math.sin(bearing_rad)
            pos_z = 0.0
            bearing_deg = math.degrees(bearing_rad)

            # Project into image pixel space
            norm_x = -bearing_rad / math.radians(self.hfov_deg / 2.0)
            cx = self.cx + norm_x * self.cx
            alpha = math.atan2(self.cam_height, dist) - self.cam_pitch_rad
            cy = self.cy + math.tan(alpha) * self.fy

            pixel_size = max(15.0, (self.fx * self.ball_diameter) / dist)
            x1 = cx - pixel_size / 2.0
            y1 = cy - pixel_size / 2.0
            x2 = cx + pixel_size / 2.0
            y2 = cy + pixel_size / 2.0

            cloud_points.append((pos_x, pos_y, pos_z))

            if HAS_CUSTOM_MSG:
                obs = Obstacle()
                obs.header.stamp = stamp
                obs.header.frame_id = self.target_frame
                obs.label = label
                obs.confidence = 0.94
                obs.position.x = float(pos_x)
                obs.position.y = float(pos_y)
                obs.position.z = float(pos_z)
                obs.distance_m = float(dist)
                obs.bearing_deg = float(bearing_deg)
                obs.bbox_center_x = float(cx)
                obs.bbox_center_y = float(cy)
                obs.bbox_width = float(pixel_size)
                obs.bbox_height = float(pixel_size)
                detected_obstacles.append(obs)

            if self.publish_annotated and (0 <= cx < self.frame_w and 0 <= cy < self.frame_h):
                cv2.circle(sim_frame, (int(cx), int(cy)), int(pixel_size / 2), bgr_color, -1)
                cv2.rectangle(sim_frame, (int(x1), int(y1)), (int(x2), int(y2)), bgr_color, 2)
                text = f"{label} {dist:.1f}m ({bearing_deg:+.1f}d)"
                cv2.putText(sim_frame, text, (int(x1), max(20, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        if self.publish_annotated:
            status_text = f"MOCK PERCEPTION | Sim T: {self.sim_tick:.1f}s"
            cv2.putText(sim_frame, status_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        self._publish_perception_data(stamp, detected_obstacles, cloud_points, sim_frame)

    def _publish_perception_data(self, stamp, obstacles, points, frame):
        """Broadcasts ObstacleArray, PointCloud2, and Image topics."""
        if HAS_CUSTOM_MSG and self.obstacle_pub:
            arr_msg = ObstacleArray()
            arr_msg.header.stamp = stamp
            arr_msg.header.frame_id = self.target_frame
            arr_msg.obstacles = obstacles
            self.obstacle_pub.publish(arr_msg)

        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame
        pc2_msg = self._create_pointcloud2(header, points)
        self.pointcloud_pub.publish(pc2_msg)

        if self.publish_annotated and self.bridge and frame is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header = header
                self.annotated_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug image: {e}")

    def _create_pointcloud2(self, header: Header, points: List[Tuple[float, float, float]]) -> PointCloud2:
        """Packages a list of (x, y, z) into a sensor_msgs/PointCloud2 binary message."""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        buffer = bytearray()
        for p in points:
            for z_offset in [-0.2, 0.0, 0.2]:
                buffer.extend(struct.pack('fff', p[0], p[1], p[2] + z_offset))

        total_pts = len(points) * 3

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = total_pts
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * total_pts
        msg.is_dense = True
        msg.data = bytes(buffer)
        return msg

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ASVVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
