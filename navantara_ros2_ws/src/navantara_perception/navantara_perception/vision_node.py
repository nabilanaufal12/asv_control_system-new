#!/usr/bin/env python3
"""
navantara_perception / vision_node.py
=====================================
ROS2 Node for Real-time Buoy & Obstacle Perception using YOLOv11 TensorRT.
Part of ASV Navantara ROS2 Architecture.

Migrated from monolithic vision_service.py:
- Decoupled from Flask, Eventlet, SocketIO, and Navigation logic.
- Executes TensorRT engine inference on NVIDIA Jetson GPU.
- Projects 2D bounding boxes into metric (x, y) relative to 'base_link'.
- Publishes custom ObstacleArray and PointCloud2 for Nav2 local costmap injection.
"""

import math
import struct
import time
from pathlib import Path
from typing import List, Optional

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
        self.declare_parameter('model_path', 'best.engine')
        self.declare_parameter('camera_source', 0)  # /dev/video0 or RTSP URL or topic
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_topic_input', False)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('camera_hfov_deg', 70.0)
        self.declare_parameter('target_frame_id', 'base_link')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('inference_rate_hz', 25.0)

        # Buoy physical size constants (diameter in meters)
        self.declare_parameter('buoy_ball_diameter_m', 0.30)
        self.declare_parameter('buoy_box_width_m', 0.35)

        # 2. Retrieve Parameters
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().integer_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.use_topic_input = self.get_parameter('use_topic_input').get_parameter_value().bool_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_thresh = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.frame_w = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_h = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.hfov_deg = self.get_parameter('camera_hfov_deg').get_parameter_value().double_value
        self.target_frame = self.get_parameter('target_frame_id').get_parameter_value().string_value
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        rate_hz = self.get_parameter('inference_rate_hz').get_parameter_value().double_value

        self.ball_diameter = self.get_parameter('buoy_ball_diameter_m').get_parameter_value().double_value
        self.box_width = self.get_parameter('buoy_box_width_m').get_parameter_value().double_value

        # Calculate focal length in pixels: fx = (W / 2) / tan(HFOV / 2)
        self.fx = (self.frame_w / 2.0) / math.tan(math.radians(self.hfov_deg / 2.0))

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
            self.get_logger().warn("navantara_interfaces not found. ObstacleArray topic disabled.")

        self.pointcloud_pub = self.create_publisher(PointCloud2, '/perception/obstacle_points', sensor_qos)

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, '/perception/image_annotated', 2)

        self.bridge = CvBridge() if CvBridge else None

        # 4. Load TensorRT Engine / YOLO Model
        self.model = None
        self._load_engine()

        # 5. Input Mode Configuration
        self.cap = None
        if self.use_topic_input:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self._image_callback, sensor_qos
            )
            self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        else:
            self._init_camera_capture()
            timer_period = 1.0 / max(1.0, rate_hz)
            self.timer = self.create_timer(timer_period, self._camera_timer_callback)
            self.get_logger().info(f"Camera capture loop initialized at {rate_hz:.1f} Hz")

    def _load_engine(self):
        """Loads TensorRT .engine or YOLO .pt fallback."""
        if not HAS_ULTRALYTICS:
            self.get_logger().error("Ultralytics library is not installed! Cannot run inference.")
            return

        model_file = Path(self.model_path)
        if not model_file.exists():
            self.get_logger().warn(f"Engine file not found at '{self.model_path}'. Trying fallback...")
            # Search common Jetson paths
            candidates = [
                Path("src/navantara_backend/vision/best.engine"),
                Path("best.engine"),
                Path("best.pt"),
            ]
            for cand in candidates:
                if cand.exists():
                    model_file = cand
                    break

        try:
            self.get_logger().info(f"Loading YOLO/TensorRT model from: {model_file}")
            self.model = YOLO(str(model_file), task='detect')
            self.get_logger().info("YOLO Model loaded successfully on GPU/TensorRT.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

    def _init_camera_capture(self):
        """Initializes OpenCV VideoCapture with V4L2 backend."""
        self.cap = cv2.VideoCapture(self.camera_source, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            # Fallback to default
            self.cap = cv2.VideoCapture(self.camera_source)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.get_logger().info(f"Connected to camera device /dev/video{self.camera_source}")
        else:
            self.get_logger().error(f"Failed to open camera device {self.camera_source}")

    def _camera_timer_callback(self):
        """Timer callback to read frame from camera and run perception."""
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Dropped camera frame.")
            return

        self._process_frame(frame)

    def _image_callback(self, msg: Image):
        """Topic callback when frames are fed via ROS Image topic."""
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")

    def _process_frame(self, frame: np.ndarray):
        """Runs TensorRT detection and projects obstacles into 3D metric coordinates."""
        if self.model is None:
            return

        h, w, _ = frame.shape
        stamp = self.get_clock().now().to_msg()

        # Run inference
        results = self.model.predict(
            source=frame,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            verbose=False,
            device=0  # Jetson CUDA device 0
        )

        detected_obstacles = []
        cloud_points = []  # List of (x, y, z) in meters

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                label = self.model.names[cls_id] if hasattr(self.model, 'names') else str(cls_id)

                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = xyxy
                bw = max(1.0, x2 - x1)
                bh = max(1.0, y2 - y1)
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                # Metric distance estimation using pinhole camera model
                # Z = (focal_length * real_object_dimension) / pixel_dimension
                real_dim = self.box_width if 'box' in label.lower() or 'kotak' in label.lower() else self.ball_diameter
                dist_m = (self.fx * real_dim) / bw
                dist_m = float(np.clip(dist_m, 0.5, 40.0))

                # Bearing calculation: horizontal angle relative to camera optical axis
                # Normalized offset from image center [-1.0 to 1.0]
                norm_x = (cx - (w / 2.0)) / (w / 2.0)
                bearing_rad = norm_x * math.radians(self.hfov_deg / 2.0)
                bearing_deg = math.degrees(bearing_rad)

                # Project into ASV base_link frame (X: forward, Y: left, Z: up)
                pos_x = dist_m * math.cos(bearing_rad)
                pos_y = -dist_m * math.sin(bearing_rad)  # Image right is negative Y in ROS standard
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

                # Draw on frame if debugging
                if self.publish_annotated:
                    color = (0, 255, 0) if 'hijau' in label or 'green' in label else (0, 0, 255)
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    text = f"{label} {conf:.2f} | {dist_m:.1f}m {bearing_deg:+.1f}deg"
                    cv2.putText(frame, text, (int(x1), max(20, int(y1) - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 1. Publish ObstacleArray message
        if HAS_CUSTOM_MSG and self.obstacle_pub:
            arr_msg = ObstacleArray()
            arr_msg.header.stamp = stamp
            arr_msg.header.frame_id = self.target_frame
            arr_msg.obstacles = detected_obstacles
            self.obstacle_pub.publish(arr_msg)

        # 2. Publish PointCloud2 message for direct Nav2 Costmap injection
        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame
        pc2_msg = self._create_pointcloud2(header, cloud_points)
        self.pointcloud_pub.publish(pc2_msg)

        # 3. Publish Annotated image
        if self.publish_annotated and self.bridge:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header = header
                self.annotated_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish annotated image: {e}")

    def _create_pointcloud2(self, header: Header, points: List[tuple]) -> PointCloud2:
        """Packages a list of (x, y, z) into a sensor_msgs/PointCloud2 binary message."""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        buffer = bytearray()
        for p in points:
            # Replicate each obstacle point into a small 3D pillar for costmap raytracing
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
