# gui/components/camera/detection_thread.py
# --- FINAL: Mengaktifkan kembali pesan sukses pengiriman Firebase ---

import sys
import os
import warnings

warnings.filterwarnings(
    "ignore", message="`torch.cuda.amp.autocast(args...)` is deprecated.*"
)

import cv2
import torch
import numpy as np
import time
from pathlib import Path
import requests
import threading

if os.name == "nt":
    import pathlib

    pathlib.PosixPath = pathlib.WindowsPath

from .overlay_utils import create_overlay_from_html, apply_overlay

try:
    CURRENT_FILE_DIR = Path(os.path.abspath(__file__)).resolve()
    PROJECT_ROOT = CURRENT_FILE_DIR.parents[3]
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"Peringatan: Gagal mengimpor modul: {e}")
    YOLO_AVAILABLE = False

from PySide6.QtCore import QThread, Signal, Slot


def send_telemetry_to_firebase(telemetry_data, config):
    """Mengirim data telemetri (dict) ke Firebase Realtime Database."""
    try:
        FIREBASE_URL = config.get("api_urls", {}).get("firebase_url")
        if not FIREBASE_URL:
            print("🔥 Peringatan: firebase_url tidak ditemukan di config.json")
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
        if response.ok:
            # --- PERBAIKAN DI SINI ---
            # Mengaktifkan kembali pesan print yang sebelumnya dinonaktifkan
            print(f"✅ Telemetri dikirim ke Firebase @ {data_to_send['jam']}")
            # -------------------------
        else:
            print(f"🔥 Gagal mengirim telemetri: {response.status_code}")
    except Exception as e:
        print(f"🔥 Error koneksi Firebase: {e}")


def upload_image_to_supabase(image_buffer, filename, config):
    """Mengunggah buffer gambar ke Supabase Storage."""
    try:
        api_config = config.get("api_urls", {})
        ENDPOINT_TEMPLATE, TOKEN = api_config.get("supabase_endpoint"), api_config.get(
            "supabase_token"
        )
        if not ENDPOINT_TEMPLATE or not TOKEN:
            print(
                "🔥 Peringatan: supabase_endpoint atau supabase_token tidak ditemukan."
            )
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
            print(f"✅ Gambar '{filename}' berhasil diunggah ke Supabase.")
        else:
            print(f"🔥 Gagal mengunggah gambar: {response.status_code} {response.text}")
    except Exception as e:
        print(f"🔥 Error koneksi Supabase: {e}")


class DetectionThread(QThread):
    frame_ready = Signal(np.ndarray)
    vision_command_status = Signal(dict)

    def __init__(self, source_idx, weights_path, config, parent=None):
        super().__init__(parent)
        self.config = config
        self.source_idx, self.weights_path = source_idx, weights_path
        self.running, self.mode_auto, self.is_inverted = True, False, False
        self.snapshot_dir = PROJECT_ROOT / "snapshots"
        os.makedirs(self.snapshot_dir, exist_ok=True)
        self.latest_telemetry = {}
        self.firebase_thread = threading.Thread(
            target=self.firebase_updater, daemon=True
        )
        self.surface_image_count = 1
        self.underwater_image_count = 1

    def firebase_updater(self):
        while self.running:
            if self.latest_telemetry:
                send_telemetry_to_firebase(self.latest_telemetry, self.config)
            time.sleep(0.5)

    @Slot(dict)
    def update_telemetry(self, data):
        self.latest_telemetry = data

    def get_score(self, ball, is_pair=False, other_ball=None):
        size = (ball["xyxy"][2] - ball["xyxy"][0]) * (ball["xyxy"][3] - ball["xyxy"][1])
        if is_pair and other_ball:
            other_size = (other_ball["xyxy"][2] - other_ball["xyxy"][0]) * (
                other_ball["xyxy"][3] - other_ball["xyxy"][1]
            )
            avg_y = (ball["center"][1] + other_ball["center"][1]) / 2
            return 0.6 * (size + other_size) + 0.4 * avg_y
        else:
            y_pos = ball["center"][1]
            return 0.6 * size + 0.4 * y_pos

    def run(self):
        self.firebase_thread.start()
        cap = None
        try:
            print("Mempersiapkan model YOLOv5...")
            model = torch.hub.load(
                "ultralytics/yolov5",
                "custom",
                path=self.weights_path,
                force_reload=True,
            )
            model.conf = 0.4
            model.iou = 0.45
            from ultralytics.utils.plotting import Annotator

            print(f"Mencoba membuka kamera indeks: {self.source_idx}...")
            cap = cv2.VideoCapture(self.source_idx, cv2.CAP_DSHOW)
            if not cap.isOpened():
                print(f"ERROR: Tidak bisa membuka kamera {self.source_idx}.")
                return

            detection_config = self.config.get("camera_detection", {})
            PANJANG_FOKUS_PIKSEL = detection_config.get("focal_length_pixels", 600)
            TARGET_JARAK_CM = detection_config.get(
                "target_activation_distance_cm", 100.0
            )
            LEBAR_BOLA_TUNGGAL_CM = detection_config.get(
                "single_ball_real_width_cm", 10.0
            )
            LEBAR_BOLA_GANDA_CM = detection_config.get("dual_ball_real_width_cm", 200.0)

            print("Kamera dan model berhasil dimuat. Memulai deteksi...")
            while self.running and cap.isOpened():
                ret, im0 = cap.read()
                if not ret:
                    print("Peringatan: Gagal membaca frame dari kamera.")
                    break

                results = model(im0)
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

                mission_name = (
                    "Surface Imaging"
                    if detected_green_boxes
                    else "Underwater Imaging" if detected_blue_boxes else None
                )
                if mission_name:
                    overlay_image = create_overlay_from_html(
                        self.latest_telemetry, mission_type=mission_name
                    )
                    snapshot_image = apply_overlay(im0.copy(), overlay_image)
                    supabase_filename = (
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
                        upload_image_to_supabase(buffer, supabase_filename, self.config)

                if self.mode_auto:
                    target_object, target_midpoint, lebar_asli_cm = None, None, 0
                    if detected_red_buoys and detected_green_buoys:
                        best_pair = max(
                            (
                                (r, g)
                                for r in detected_red_buoys
                                for g in detected_green_buoys
                            ),
                            key=lambda pair: self.get_score(
                                pair[0], is_pair=True, other_ball=pair[1]
                            ),
                        )
                        target_object, red_ball, green_ball = (
                            best_pair,
                            best_pair[0],
                            best_pair[1],
                        )
                        target_midpoint = (
                            (red_ball["center"][0] + green_ball["center"][0]) // 2,
                            (red_ball["center"][1] + green_ball["center"][1]) // 2,
                        )
                        lebar_asli_cm = LEBAR_BOLA_GANDA_CM
                    elif detected_red_buoys or detected_green_buoys:
                        best_single_ball = max(
                            detected_red_buoys + detected_green_buoys,
                            key=lambda b: self.get_score(b),
                        )
                        is_left_buoy = "red_buoy" in best_single_ball["class"]
                        if self.is_inverted:
                            is_left_buoy = "green_buoy" in best_single_ball["class"]
                        lebar_objek_piksel_single = (
                            best_single_ball["xyxy"][2] - best_single_ball["xyxy"][0]
                        )
                        jarak_estimasi_cm_single = (
                            (LEBAR_BOLA_TUNGGAL_CM * PANJANG_FOKUS_PIKSEL)
                            / lebar_objek_piksel_single
                            if lebar_objek_piksel_single > 0
                            else 0
                        )
                        pixel_offset = (
                            (100.0 * PANJANG_FOKUS_PIKSEL) / jarak_estimasi_cm_single
                            if jarak_estimasi_cm_single > 0
                            else 0
                        )
                        ball_center_x, ball_center_y = best_single_ball["center"]
                        virtual_target_x = (
                            ball_center_x + int(pixel_offset)
                            if is_left_buoy
                            else ball_center_x - int(pixel_offset)
                        )
                        target_object = (best_single_ball,)
                        target_midpoint = (virtual_target_x, ball_center_y)
                        lebar_asli_cm = LEBAR_BOLA_TUNGGAL_CM
                        cv2.circle(
                            annotated_frame, target_midpoint, 10, (255, 255, 0), -1
                        )
                        cv2.putText(
                            annotated_frame,
                            "Virtual Target",
                            (target_midpoint[0] + 15, target_midpoint[1]),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 0),
                            2,
                        )

                    if target_object:
                        annotator = Annotator(
                            annotated_frame, line_width=2, example="Jarak"
                        )
                        x1, y1, x2, y2 = (
                            min(t["xyxy"][0] for t in target_object),
                            min(t["xyxy"][1] for t in target_object),
                            max(t["xyxy"][2] for t in target_object),
                            max(t["xyxy"][3] for t in target_object),
                        )
                        lebar_objek_piksel = x2 - x1
                        jarak_estimasi_cm = (
                            (lebar_asli_cm * PANJANG_FOKUS_PIKSEL) / lebar_objek_piksel
                            if lebar_objek_piksel > 0
                            else 0
                        )
                        annotator.box_label(
                            [x1, y1, x2, y2], f"Jarak: {jarak_estimasi_cm:.1f} cm"
                        )
                        annotated_frame = annotator.result()

                        batas_posisi_y = im0.shape[0] * 0.5
                        if (
                            jarak_estimasi_cm > 0
                            and jarak_estimasi_cm < TARGET_JARAK_CM
                            and y1 > batas_posisi_y
                        ):
                            print(
                                f"ZONA AKTIVASI TERPENUHI (Jarak: {jarak_estimasi_cm:.1f} cm). Mengikuti objek."
                            )
                            degree = self.calculate_degree(im0, target_midpoint)
                            servo_angle, motor_pwm = self.convert_degree_to_actuators(
                                degree
                            )
                            print(
                                f"   => Perintah Dihitung: Motor PWM={motor_pwm}, Servo Angle={servo_angle}"
                            )
                            self.vision_command_status.emit(
                                {
                                    "status": "ACTIVE",
                                    "command": f"S{motor_pwm};D{servo_angle}\n",
                                }
                            )
                        else:
                            self.vision_command_status.emit({"status": "INACTIVE"})
                    else:
                        self.vision_command_status.emit({"status": "INACTIVE"})

                self.frame_ready.emit(annotated_frame)
        except Exception as e:
            print(f"Error di dalam thread deteksi: {e}")
        finally:
            self.stop()
            if cap and cap.isOpened():
                cap.release()
            print("Pembersihan thread deteksi selesai.")

    def stop(self):
        self.running = False

    @Slot(str)
    def set_mode(self, mode):
        """Menerima sinyal mode dari GUI untuk mengaktifkan/menonaktifkan logika deteksi."""
        self.mode_auto = mode == "AUTO"
        print(f"Mode deteksi diatur ke: {'AKTIF' if self.mode_auto else 'NONAKTIF'}")

    def calculate_degree(self, frame, midpoint):
        h, w, _ = frame.shape
        bar_left, bar_right = 50, w - 50
        relative_pos = np.clip((midpoint[0] - bar_left) / (bar_right - bar_left), 0, 1)
        return int(relative_pos * 180)

    def convert_degree_to_actuators(self, degree):
        error = degree - 90
        if self.is_inverted:
            error = -error
        servo_angle = max(0, min(180, int(90 - (error / 90.0) * 45)))
        motor_pwm = 1600 - ((abs(error) / 90.0) * 100)
        return int(servo_angle), int(motor_pwm)
