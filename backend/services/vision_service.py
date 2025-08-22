# backend/services/vision_service.py
# --- VERSI MODIFIKASI: Penyesuaian Logika Aktuator untuk Logging Akurat ---

import sys
from pathlib import Path
import os
import traceback
import pathlib

# --- Patch untuk masalah PosixPath di Windows ---
if os.name == "nt":
    pathlib.PosixPath = pathlib.WindowsPath

# Menambahkan path YOLOv5 ke sistem SEBELUM impor
backend_dir = Path(__file__).resolve().parents[1]
yolov5_path = backend_dir / "yolov5"
if str(yolov5_path) not in sys.path:
    sys.path.insert(0, str(yolov5_path))

import cv2
import torch
import threading
import time
import requests
import numpy as np
from PySide6.QtCore import QObject, Signal, Slot

from backend.vision.overlay_utils import create_overlay_from_html, apply_overlay

# Impor modul YOLOv5 setelah path-nya ditambahkan
from models.common import DetectMultiBackend
from utils.plots import Annotator


# (Fungsi helper tidak berubah)
def send_telemetry_to_firebase(telemetry_data, config):
    try:
        FIREBASE_URL = config.get("api_urls", {}).get("firebase_url")
        if not FIREBASE_URL:
            return
        data_to_send = {
            "gps": {
                "lat": telemetry_data.get("latitude", 0.0),
                "lng": telemetry_data.get("longitude", 0.0),
            },
            "hdg": telemetry_data.get("heading", 0.0),
            "cog": telemetry_data.get("cog", 0.0),
            "sog": telemetry_data.get("speed", 0.0),
            "jam": time.strftime("%H:%M:%S"),
        }
        response = requests.put(FIREBASE_URL, json=data_to_send, timeout=5)
    except Exception:
        pass


def upload_image_to_supabase(image_buffer, filename, config):
    try:
        api_config = config.get("api_urls", {})
        ENDPOINT_TEMPLATE, TOKEN = api_config.get("supabase_endpoint"), api_config.get(
            "supabase_token"
        )
        if not ENDPOINT_TEMPLATE or not TOKEN:
            return
        ENDPOINT = ENDPOINT_TEMPLATE.replace("{filename}", filename)
        headers = {
            "Authorization": f"Bearer {TOKEN}",
            "Content-Type": "image/jpeg",
            "x-upsert": "true",
        }
        response = requests.post(
            ENDPOINT, headers=headers, data=image_buffer.tobytes(), timeout=10
        )
        if response.ok:
            print(f"\n[Vision] ✅ Gambar '{filename}' berhasil diunggah ke Supabase.")
        else:
            print(
                f"\n[Vision] 🔥 Gagal mengunggah gambar: {response.status_code} {response.text}"
            )
    except Exception as e:
        print(f"\n[Vision] 🔥 Error koneksi Supabase: {e}")


class VisionService(QObject):
    frame_ready = Signal(np.ndarray)

    def __init__(self, config, asv_handler):
        super().__init__()
        self.config = config
        self.asv_handler = asv_handler
        self.running = False
        self.model = None
        self.is_inverted = False
        self.mode_auto = False
        self.restart_camera = False
        self.surface_image_count = 1
        self.underwater_image_count = 1

        vision_cfg = self.config.get("vision", {})
        self.conf_thresh = float(vision_cfg.get("conf_threshold", 0.4))
        self.iou_thresh = float(vision_cfg.get("iou_threshold", 0.45))
        self.device = str(
            vision_cfg.get("device", "cuda" if torch.cuda.is_available() else "cpu")
        )
        self.camera_index = int(vision_cfg.get("camera_index", 0))

        self._initialize_model()

    def _initialize_model(self):
        try:
            weights_path = yolov5_path / "besto.pt"
            if not yolov5_path.exists() or not weights_path.exists():
                print(
                    f"[Vision] KRITIS: Direktori '{yolov5_path}' atau file bobot '{weights_path}' tidak ditemukan."
                )
                self.model = None
                return
            print("[Vision] Mempersiapkan model YOLOv5 dari sumber lokal...")
            self.model = torch.hub.load(
                str(yolov5_path),
                "custom",
                path=str(weights_path),
                source="local",
                force_reload=True,
                trust_repo=True,
            )
            self.model.conf = self.conf_thresh
            self.model.iou = self.iou_thresh
            print(f"[Vision] Model YOLOv5 berhasil dimuat di device {self.device}.")
        except Exception:
            print(f"[Vision] KRITIS: Gagal memuat model YOLOv5.")
            traceback.print_exc()
            self.model = None

    @Slot()
    def run(self):
        if self.model is None:
            print("[Vision] Model tidak tersedia, layanan visi tidak dapat dimulai.")
            return

        self.running = True
        cap = None

        while self.running:
            self.restart_camera = False
            print(
                f"[Vision] Mencoba membuka kamera indeks {self.camera_index} dengan backend DSHOW..."
            )
            cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)

            if not cap.isOpened():
                print(
                    f"[Vision] ERROR: Gagal membuka kamera indeks {self.camera_index}. Mencari kamera lain..."
                )
                found_cam_index = self.find_working_camera()
                if found_cam_index is not None:
                    self.camera_index = found_cam_index
                    cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
                else:
                    print(
                        "[Vision] ❌ Tidak ada kamera yang bisa dibuka. Mencoba lagi dalam 2 detik..."
                    )
                    time.sleep(2)
                    continue

            print(
                f"[Vision] Kamera {self.camera_index} berhasil dibuka. Memulai deteksi..."
            )

            while self.running and not self.restart_camera:
                try:
                    if not cap.isOpened():
                        break
                    ret, im0 = cap.read()
                    if not ret:
                        break

                    annotated_frame = self.process_frame(im0)
                    self.frame_ready.emit(annotated_frame)
                    time.sleep(0.01)

                except Exception as e:
                    print(f"\n[Vision] ERROR di dalam loop deteksi: {e}")
                    traceback.print_exc()
                    break

            if cap and cap.isOpened():
                cap.release()

            if self.restart_camera:
                print(f"\n[Vision] Me-restart stream kamera...")

        print("\n[Vision] Thread layanan visi telah dihentikan.")

    def process_frame(self, im0):
        results = self.model(im0)
        annotated_frame = results.render()[0].copy()

        (
            detected_red_buoys,
            detected_green_buoys,
            detected_green_boxes,
            detected_blue_boxes,
        ) = ([], [], [], [])
        df = results.pandas().xyxy[0]
        for _, row in df.iterrows():
            bbox_data = {
                "xyxy": [row["xmin"], row["ymin"], row["xmax"], row["ymax"]],
                "center": (
                    int((row["xmin"] + row["xmax"]) / 2),
                    int((row["ymin"] + row["ymax"]) / 2),
                ),
                "class": row["name"],
            }
            if "red_buoy" in row["name"]:
                detected_red_buoys.append(bbox_data)
            elif "green_buoy" in row["name"]:
                detected_green_buoys.append(bbox_data)
            if "green_box" in row["name"]:
                detected_green_boxes.append(bbox_data)
            elif "blue_box" in row["name"]:
                detected_blue_boxes.append(bbox_data)

        if detected_green_boxes or detected_blue_boxes:
            self.handle_photography_mission(
                im0, detected_green_boxes, detected_blue_boxes
            )

        if self.mode_auto:
            annotated_frame = self.handle_auto_control(
                im0, annotated_frame, detected_red_buoys, detected_green_buoys
            )

        return annotated_frame

    def handle_photography_mission(self, original_frame, green_boxes, blue_boxes):
        mission_name = (
            "Surface Imaging"
            if green_boxes
            else "Underwater Imaging" if blue_boxes else None
        )
        if not mission_name:
            return

        self.asv_handler.process_command(
            "VISION_TARGET_UPDATE", {"active": True, "degree": 90}
        )
        time.sleep(0.5)

        current_telemetry = self.asv_handler.get_current_state()
        send_telemetry_to_firebase(current_telemetry, self.config)
        overlay_image = create_overlay_from_html(
            current_telemetry, mission_type=mission_name
        )
        snapshot_image = apply_overlay(original_frame.copy(), overlay_image)

        filename = (
            f"surface_{self.surface_image_count}.jpg"
            if mission_name == "Surface Imaging"
            else f"underwater_{self.underwater_image_count}.jpg"
        )
        if mission_name == "Surface Imaging":
            self.surface_image_count += 1
        else:
            self.underwater_image_count += 1

        ret, buffer = cv2.imencode(".jpg", snapshot_image)
        if ret:
            threading.Thread(
                target=upload_image_to_supabase,
                args=(buffer, filename, self.config),
                daemon=True,
            ).start()

        self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})

    def handle_auto_control(self, im0, annotated_frame, red_buoys, green_buoys):
        detection_config = self.config.get("camera_detection", {})
        PANJANG_FOKUS_PIKSEL = detection_config.get("focal_length_pixels", 600)
        TARGET_JARAK_CM = detection_config.get("target_activation_distance_cm", 100.0)
        
        LEBAR_BOLA_TUNGGAL_CM = detection_config.get("single_ball_real_width_cm", 10.0)
        LEBAR_BOLA_GANDA_CM = detection_config.get("dual_ball_real_width_cm", 200.0)
        VIRTUAL_TARGET_OFFSET_CM = detection_config.get("virtual_target_offset_cm", 80.0)

        target_object, target_midpoint, lebar_asli_cm = None, None, 0
        keputusan = "NAVIGATING_WAYPOINT"

        h, w, _ = annotated_frame.shape
        activation_y_percent = detection_config.get("activation_zone_y_percent", 0.5)
        zona_y_start = int(h * activation_y_percent)
        
        if red_buoys and green_buoys:
            keputusan = "AVOIDING_BUOYS (Gerbang)"
            best_pair = max(
                ((r, g) for r in red_buoys for g in green_buoys),
                key=lambda p: self.get_score(p[0], True, p[1]),
            )
            target_object, red_ball, green_ball = best_pair, best_pair[0], best_pair[1]
            target_midpoint = (
                (red_ball["center"][0] + green_ball["center"][0]) // 2,
                (red_ball["center"][1] + green_ball["center"][1]) // 2,
            )
            lebar_asli_cm = LEBAR_BOLA_GANDA_CM
        elif red_buoys or green_buoys:
            keputusan = "AVOIDING_BUOYS (Virtual)"
            best_single_ball = max(red_buoys + green_buoys, key=self.get_score)
            is_red = "red_buoy" in best_single_ball["class"]
            is_left = (is_red and not self.is_inverted) or (
                not is_red and self.is_inverted
            )

            w_px = best_single_ball["xyxy"][2] - best_single_ball["xyxy"][0]
            dist_cm = (
                (LEBAR_BOLA_TUNGGAL_CM * PANJANG_FOKUS_PIKSEL) / w_px if w_px > 0 else 0
            )
            
            offset_px = (VIRTUAL_TARGET_OFFSET_CM * PANJANG_FOKUS_PIKSEL) / dist_cm if dist_cm > 0 else 0

            cx, cy = best_single_ball["center"]
            v_cx = cx + int(offset_px) if is_left else cx - int(offset_px)

            target_object, target_midpoint, lebar_asli_cm = (
                (best_single_ball,),
                (v_cx, cy),
                LEBAR_BOLA_TUNGGAL_CM,
            )
            cv2.circle(annotated_frame, target_midpoint, 10, (255, 255, 0), -1)
            cv2.putText(
                annotated_frame,
                "VT",
                (target_midpoint[0] + 15, target_midpoint[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                2,
            )

        if target_object:
            annotator = Annotator(annotated_frame, line_width=2)
            x1, y1 = min(t["xyxy"][0] for t in target_object), min(
                t["xyxy"][1] for t in target_object
            )
            x2, y2 = max(t["xyxy"][2] for t in target_object), max(
                t["xyxy"][3] for t in target_object
            )

            lebar_px = x2 - x1
            jarak_cm = (
                (lebar_asli_cm * PANJANG_FOKUS_PIKSEL) / lebar_px if lebar_px > 0 else 0
            )

            annotator.box_label([x1, y1, x2, y2], f"Jarak: {jarak_cm:.1f} cm")
            annotated_frame = annotator.result()

            if 0 < jarak_cm < TARGET_JARAK_CM and y1 > zona_y_start:
                degree = self.calculate_degree(im0, target_midpoint)
                servo_angle, motor_pwm = self.convert_degree_to_actuators(degree)

                print(f"\n[VISION] ZONA AKTIVASI TERPENUHI (Jarak: {jarak_cm:.1f} cm). Mengambil alih kontrol.")
                print(f"   => Perintah Dihitung: Motor PWM={motor_pwm}, Servo Angle={servo_angle}°")
                
                self.asv_handler.process_command(
                    "VISION_TARGET_UPDATE", {"active": True, "degree": degree, "is_inverted": self.is_inverted}
                )
            else:
                self.asv_handler.process_command(
                    "VISION_TARGET_UPDATE", {"active": False}
                )
                print(
                    f"\r[VISION] STATUS: PASIF | Objek di luar jangkauan | Jarak: {jarak_cm:.1f}cm",
                    end="",
                )
        else:
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})
            print(
                f"\r[VISION] STATUS: PASIF | {keputusan} | Mengikuti Waypoint...",
                end="",
            )

        return annotated_frame

    def stop(self):
        self.running = False
        print("\n[Vision] Perintah stop diterima.")

    def find_working_camera(self, max_indices=5):
        print("[Vision] Memindai kamera dengan backend DSHOW...")
        for idx in range(max_indices):
            test_cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
            if test_cap and test_cap.isOpened():
                print(f"[Vision]   -> Kamera ditemukan di indeks {idx}")
                test_cap.release()
                return idx
        return None

    @Slot(str)
    def set_mode(self, mode):
        self.mode_auto = mode == "AUTO"
        print(f"\n[Vision] Mode AUTO diatur ke: {self.mode_auto}")

    @Slot(bool)
    def set_inversion(self, is_inverted):
        self.is_inverted = is_inverted
        print(f"\n[Vision] Logika invers diatur ke: {self.is_inverted}")

    def get_score(self, ball, is_pair=False, other_ball=None):
        size = (ball["xyxy"][2] - ball["xyxy"][0]) * (ball["xyxy"][3] - ball["xyxy"][1])
        if is_pair and other_ball:
            other_size = (other_ball["xyxy"][2] - other_ball["xyxy"][0]) * (
                other_ball["xyxy"][3] - other_ball["xyxy"][1]
            )
            avg_y = (ball["center"][1] + other_ball["center"][1]) / 2
            return 0.6 * (size + other_size) + 0.4 * avg_y
        return 0.6 * size + 0.4 * ball["center"][1]

    def calculate_degree(self, frame, midpoint):
        h, w, _ = frame.shape
        bar_left, bar_right = w * 0.1, w * 0.9
        relative_pos = np.clip((midpoint[0] - bar_left) / (bar_right - bar_left), 0, 1)
        return int(relative_pos * 180)

    def convert_degree_to_actuators(self, degree):
        """
        Mencerminkan logika yang sama persis dengan AsvHandler
        untuk memastikan logging yang akurat.
        """
        error = degree - 90
        if self.is_inverted:
            error = -error

        actuators = self.config.get("actuators", {})
        servo_def = actuators.get("servo_default_angle", 90)
        servo_min = actuators.get("servo_min_angle", 45)
        servo_max = actuators.get("servo_max_angle", 135)
        
        normalized_error = error / 90.0
        if normalized_error < 0:
            servo_range = servo_def - servo_min
            servo = servo_def + (normalized_error * servo_range)
        else:
            servo_range = servo_max - servo_def
            servo = servo_def + (normalized_error * servo_range)
            
        servo = int(np.clip(servo, servo_min, servo_max))

        motor_base = actuators.get("motor_pwm_auto_base", 1650)
        reduction = actuators.get("motor_pwm_auto_reduction", 100)
        pwm = motor_base - (abs(normalized_error) * reduction)

        return int(servo), int(pwm)