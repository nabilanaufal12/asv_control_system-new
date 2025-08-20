# backend/services/vision_service.py
# --- VERSI FINAL: dengan camera_index dari config.vision + auto-scan kamera ---

import cv2
import torch
import threading
import time
from pathlib import Path
import os
import pathlib
import requests
import numpy as np
import sys
import base64

from backend.vision.overlay_utils import create_overlay_from_html, apply_overlay

if os.name == 'nt':
    pathlib.PosixPath = pathlib.WindowsPath

yolov5_path = Path(__file__).parents[1] / "yolov5"
sys.path.append(str(yolov5_path))

from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_boxes
from utils.plots import Annotator, colors


def send_telemetry_to_firebase(telemetry_data, config):
    try:
        FIREBASE_URL = config.get("api_urls", {}).get("firebase_url")
        if not FIREBASE_URL: return
        data_to_send = {
            "gps": {"lat": telemetry_data.get('latitude', 0.0), "lng": telemetry_data.get('longitude', 0.0)},
            "hdg": telemetry_data.get('heading', 0.0), "cog": telemetry_data.get('cog', 0.0),
            "sog": telemetry_data.get('speed', 0.0), "jam": time.strftime("%H:%M:%S"),
        }
        requests.put(FIREBASE_URL, json=data_to_send, timeout=5)
    except Exception:
        pass


def upload_image_to_supabase(image_buffer, filename, config):
    try:
        api_config = config.get("api_urls", {})
        ENDPOINT_TEMPLATE, TOKEN = api_config.get("supabase_endpoint"), api_config.get("supabase_token")
        if not ENDPOINT_TEMPLATE or not TOKEN: return
        ENDPOINT = ENDPOINT_TEMPLATE.replace("{filename}", filename)
        headers = {'Authorization': f'Bearer {TOKEN}', 'Content-Type': 'image/jpeg', 'x-upsert': 'true'}
        requests.post(ENDPOINT, headers=headers, data=image_buffer.tobytes(), timeout=10)
        print(f"✅ Gambar '{filename}' berhasil diunggah ke Supabase.")
    except Exception:
        pass


class VisionService:
    def __init__(self, config, asv_handler, socketio=None):
        self.socketio = socketio
        self.config = config
        self.asv_handler = asv_handler
        self.running = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.output_frame = None
        self.lock = threading.Lock()
        self.model = None
        self.is_inverted = False
        self.mode_auto = False
        self.restart_camera = False
        self.surface_image_count = 1
        self.underwater_image_count = 1

        # 🔹 Ambil parameter dari config.vision lebih dulu
        vision_cfg = self.config.get("vision", {})
        self.debug = bool(vision_cfg.get("debug", True))
        self.conf_thresh = float(vision_cfg.get("conf_threshold", 0.4))
        self.iou_thresh = float(vision_cfg.get("iou_threshold", 0.45))
        self.device = str(vision_cfg.get("device", "cuda" if torch.cuda.is_available() else "cpu"))
        self.camera_index = int(vision_cfg.get("camera_index", 0))

        self._initialize_model()

    def _initialize_model(self):
        weights_path = Path(__file__).parents[1] / "yolov5" / "besto.pt"
        try:
            print("[Vision] Mempersiapkan model YOLOv5...")
            self.model = DetectMultiBackend(str(weights_path), device=self.device)
            self.model.conf = self.conf_thresh
            self.model.iou = self.iou_thresh
            print(f"[Vision] Model YOLOv5 berhasil dimuat dari lokal pada device {self.device}.")
            print(f"[Vision] Threshold: conf={self.model.conf}, iou={self.model.iou}")
        except Exception as e:
            print(f"[Vision] KRITIS: Gagal memuat model YOLOv5. Error: {e}")
            self.model = None

    def start(self):
        if self.model is None:
            return
        self.running = True
        self.thread.start()

    def run(self):
        while self.running:
            self.restart_camera = False
            cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
            if not cap.isOpened():
                print(f"[Vision] ERROR: Kamera indeks {self.camera_index} tidak tersedia.")
                # 🔹 Auto-scan kamera aktif
                found = False
                for idx in range(5):  # coba index 0–4
                    test_cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
                    if test_cap.isOpened():
                        print(f"[Vision] ✅ Kamera ditemukan di indeks {idx}, mengganti camera_index.")
                        self.camera_index = idx
                        cap = test_cap
                        found = True
                        break
                    test_cap.release()
                if not found:
                    print("[Vision] ❌ Tidak ada kamera yang tersedia, retry dalam 2 detik...")
                    time.sleep(2)
                    continue

            print(f"[Vision] Kamera indeks {self.camera_index} dibuka. Memulai deteksi...")
            
            while self.running and not self.restart_camera:
                ret, im0 = cap.read()
                if not ret:
                    time.sleep(0.01)
                    continue

                im = cv2.cvtColor(im0, cv2.COLOR_BGR2RGB)
                im = torch.from_numpy(im).to(self.device)
                im = im.permute(2, 0, 1).float()
                im = im.unsqueeze(0) / 255.0

                pred = self.model(im, augment=False, visualize=False)
                pred = non_max_suppression(pred, self.model.conf, self.model.iou)

                annotator = Annotator(im0, line_width=2, example=str(self.model.names))

                detected_red_buoys, detected_green_buoys = [], []
                detected_green_boxes, detected_blue_boxes = [], []

                for det in pred:
                    if len(det):
                        det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                        for *xyxy, conf, cls in reversed(det):
                            c = int(cls)
                            label = f"{self.model.names[c]} {conf:.2f}"
                            annotator.box_label(xyxy, label, color=colors(c, True))

                            bbox_data = {
                                'xyxy': [float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])],
                                'center': (int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)),
                                'class': self.model.names[c]
                            }

                            if "red_buoy" in self.model.names[c]:
                                detected_red_buoys.append(bbox_data)
                            elif "green_buoy" in self.model.names[c]:
                                detected_green_buoys.append(bbox_data)
                            if "green_box" in self.model.names[c]:
                                detected_green_boxes.append(bbox_data)
                            elif "blue_box" in self.model.names[c]:
                                detected_blue_boxes.append(bbox_data)

                annotated_frame = annotator.result()

                if self.debug:
                    total_dets = len(detected_red_buoys) + len(detected_green_buoys) + len(detected_green_boxes) + len(detected_blue_boxes)
                    print(f"[Vision][DEBUG] Frame: {total_dets} deteksi | Red={len(detected_red_buoys)} Green={len(detected_green_buoys)} GBox={len(detected_green_boxes)} BBox={len(detected_blue_boxes)}")

                self.handle_photography_mission(im0, detected_green_boxes, detected_blue_boxes)

                if self.mode_auto:
                    self.handle_auto_control(im0, annotated_frame, detected_red_buoys, detected_green_buoys)
                
                with self.lock:
                    self.output_frame = annotated_frame.copy()
            
                if hasattr(self, "socketio") and self.socketio:
                    _, buffer = cv2.imencode(".jpg", annotated_frame)
                    self.socketio.emit("video_frame", {"image": base64.b64encode(buffer).decode("utf-8")})

            cap.release()
            if self.restart_camera:
                print(f"[Vision] Me-restart stream kamera...")
                with self.lock: self.output_frame = None
            else:
                print("[Vision] Thread layanan visi dihentikan.")

    def handle_photography_mission(self, original_frame, green_boxes, blue_boxes):
        mission_name = "Surface Imaging" if green_boxes else "Underwater Imaging" if blue_boxes else None
        if mission_name:
            current_telemetry = self.asv_handler.get_current_state()
            send_telemetry_to_firebase(current_telemetry, self.config)
            
            overlay_image = create_overlay_from_html(current_telemetry, mission_type=mission_name)
            snapshot_image = apply_overlay(original_frame.copy(), overlay_image)
            
            if mission_name == "Surface Imaging":
                filename = f"surface_{self.surface_image_count}.jpg"
                self.surface_image_count += 1
            else:
                filename = f"underwater_{self.underwater_image_count}.jpg"
                self.underwater_image_count += 1
                
            ret, buffer = cv2.imencode('.jpg', snapshot_image)
            if ret:
                upload_thread = threading.Thread(target=upload_image_to_supabase, args=(buffer, filename, self.config))
                upload_thread.start()

    def handle_auto_control(self, im0, annotated_frame, red_buoys, green_buoys):
        detection_config = self.config.get("camera_detection", {})
        TARGET_JARAK_CM = detection_config.get("target_activation_distance_cm", 100.0)
        
        target_midpoint = self.find_target_midpoint(im0, annotated_frame, red_buoys, green_buoys)

        if target_midpoint:
            degree = self.calculate_degree(im0, target_midpoint)
            servo_angle, motor_pwm = self.convert_degree_to_actuators(degree)
            
            command_string = f"S{motor_pwm};D{servo_angle}\n"
            vision_payload = {'status': 'ACTIVE', 'command': command_string}
            self.asv_handler.process_command("VISION_OVERRIDE", vision_payload)
        else:
            self.asv_handler.process_command("VISION_OVERRIDE", {'status': 'INACTIVE'})

    def find_target_midpoint(self, im0, annotated_frame, red_buoys, green_buoys):
        if red_buoys and green_buoys:
            best_pair = max(((r, g) for r in red_buoys for g in green_buoys), 
                            key=lambda pair: self.get_score(pair[0], is_pair=True, other_ball=pair[1]))
            red_ball, green_ball = best_pair
            return ((red_ball['center'][0] + green_ball['center'][0]) // 2, 
                    (red_ball['center'][1] + green_ball['center'][1]) // 2)
        elif red_buoys or green_buoys:
            best_single_ball = max(red_buoys + green_buoys, key=self.get_score)
            return best_single_ball['center']
        return None

    def stop(self):
        self.running = False

    def get_frame(self):
        with self.lock:
            frame_to_encode = self.output_frame
            if frame_to_encode is None:
                frame_to_encode = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame_to_encode, "Menunggu Kamera...", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode)
            return encodedImage.tobytes() if flag else None

    def list_available_cameras(self):
        arr = []
        for index in range(5):
            cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
            if cap.isOpened():
                arr.append(index)
                cap.release()
        return arr

    def process_command(self, command, payload):
        if command == "SELECT_CAMERA":
            new_index = int(payload)
            if new_index != self.camera_index:
                print(f"[Vision] Perintah diterima untuk ganti kamera ke indeks {new_index}")
                self.camera_index = new_index
                self.restart_camera = True
        elif command == "SET_INVERT":
            self.is_inverted = bool(payload)
            print(f"[Vision] Logika invers diatur ke: {self.is_inverted}")
        elif command == "SET_MODE":
            self.mode_auto = (payload == "AUTO")
            print(f"[Vision] Mode AUTO diatur ke: {self.mode_auto}")

    def get_score(self, ball, is_pair=False, other_ball=None):
        size = (ball['xyxy'][2] - ball['xyxy'][0]) * (ball['xyxy'][3] - ball['xyxy'][1])
        if is_pair and other_ball:
            other_size = (other_ball['xyxy'][2] - other_ball['xyxy'][0]) * (other_ball['xyxy'][3] - other_ball['xyxy'][1])
            avg_y = (ball['center'][1] + other_ball['center'][1]) / 2
            return 0.6 * (size + other_size) + 0.4 * avg_y
        else:
            return 0.6 * size + 0.4 * ball['center'][1]

    def calculate_degree(self, frame, midpoint):
        h, w, _ = frame.shape
        center_x = w / 2
        error = midpoint[0] - center_x
        degree = 90 + (error / center_x) * 90 
        return np.clip(degree, 0, 180)

    def convert_degree_to_actuators(self, degree):
        error = degree - 90
        if self.is_inverted:
            error = -error
        
        actuator_config = self.config.get("actuators", {})
        servo_default = actuator_config.get("servo_default_angle", 90)
        servo_range = servo_default - actuator_config.get("servo_min_angle", 45)
        
        servo_angle = servo_default - (error / 90.0) * servo_range
        servo_angle = int(np.clip(servo_angle, 0, 180))

        motor_base = actuator_config.get("motor_pwm_auto_base", 1650)
        reduction = actuator_config.get("motor_pwm_auto_reduction", 100)
        motor_pwm = motor_base - (abs(error) / 90.0) * reduction
        
        return int(servo_angle), int(motor_pwm)
