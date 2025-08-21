# backend/services/vision_service.py
# --- VERSI FINAL LENGKAP DENGAN LOGIKA DETEKSI CANGGIH ---

import sys
from pathlib import Path
import os
import traceback
import pathlib

# --- Patch untuk masalah PosixPath di Windows ---
if os.name == 'nt':
    pathlib.PosixPath = pathlib.WindowsPath

# Menambahkan path YOLOv5 ke sistem SEBELUM impor
backend_dir = Path(__file__).resolve().parents[1]
yolov5_path = backend_dir / 'yolov5'
if str(yolov5_path) not in sys.path:
    sys.path.insert(0, str(yolov5_path))

import cv2
import torch
import threading
import time
import requests
import numpy as np
from PySide6.QtCore import QObject, Signal

from backend.vision.overlay_utils import create_overlay_from_html, apply_overlay

# Impor modul YOLOv5 setelah path-nya ditambahkan
from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_boxes
from utils.plots import Annotator, colors

# (Fungsi helper tidak berubah)
def send_telemetry_to_firebase(telemetry_data, config):
    try:
        FIREBASE_URL = config.get("api_urls", {}).get("firebase_url")
        if not FIREBASE_URL: return
        data_to_send = {
            "gps": {"lat": telemetry_data.get('latitude', 0.0), "lng": telemetry_data.get('longitude', 0.0)},
            "hdg": telemetry_data.get('heading', 0.0), "cog": telemetry_data.get('cog', 0.0),
            "sog": telemetry_data.get('speed', 0.0), "jam": time.strftime("%H:%M:%S"),
        }
        response = requests.put(FIREBASE_URL, json=data_to_send, timeout=5)
        if response.ok:
            print(f"✅ Telemetri dikirim ke Firebase @ {data_to_send['jam']}")
    except Exception as e:
        print(f"🔥 Error koneksi Firebase: {e}")


def upload_image_to_supabase(image_buffer, filename, config):
    try:
        api_config = config.get("api_urls", {})
        ENDPOINT_TEMPLATE, TOKEN = api_config.get("supabase_endpoint"), api_config.get("supabase_token")
        if not ENDPOINT_TEMPLATE or not TOKEN: return
        ENDPOINT = ENDPOINT_TEMPLATE.replace("{filename}", filename)
        headers = {'Authorization': f'Bearer {TOKEN}', 'Content-Type': 'image/jpeg', 'x-upsert': 'true'}
        response = requests.post(ENDPOINT, headers=headers, data=image_buffer.tobytes(), timeout=10)
        if response.ok:
            print(f"✅ Gambar '{filename}' berhasil diunggah ke Supabase.")
        else:
            print(f"🔥 Gagal mengunggah gambar: {response.status_code} {response.text}")
    except Exception as e:
        print(f"🔥 Error koneksi Supabase: {e}")


class VisionService(QObject):
    frame_ready = Signal(np.ndarray)

    def __init__(self, config, asv_handler):
        super().__init__()
        # (Inisialisasi atribut tetap sama)
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
        self.debug = bool(vision_cfg.get("debug", True))
        self.conf_thresh = float(vision_cfg.get("conf_threshold", 0.4))
        self.iou_thresh = float(vision_cfg.get("iou_threshold", 0.45))
        self.device = str(vision_cfg.get("device", "cuda" if torch.cuda.is_available() else "cpu"))
        self.camera_index = int(vision_cfg.get("camera_index", 0))

        self._initialize_model()

    def _initialize_model(self):
        try:
            weights_path = yolov5_path / 'besto.pt'
            if not yolov5_path.exists() or not weights_path.exists():
                print(f"[Vision] KRITIS: Direktori '{yolov5_path}' atau file bobot '{weights_path}' tidak ditemukan.")
                self.model = None
                return
            print("[Vision] Mempersiapkan model YOLOv5 dari sumber lokal...")
            self.model = torch.hub.load(
                str(yolov5_path), 'custom', path=str(weights_path),
                source='local', force_reload=True
            )
            self.model.conf = self.conf_thresh
            self.model.iou = self.iou_thresh
            print(f"[Vision] Model YOLOv5 berhasil dimuat di device {self.device}.")
        except Exception:
            print(f"[Vision] KRITIS: Gagal memuat model YOLOv5. Error Sebenarnya di Bawah Ini:")
            traceback.print_exc()
            self.model = None

    def run(self):
        # (Metode run tetap sama seperti sebelumnya)
        if self.model is None:
            print("[Vision] Model tidak tersedia, layanan visi tidak dapat dimulai.")
            return

        self.running = True
        while self.running:
            self.restart_camera = False
            cap = cv2.VideoCapture(self.camera_index, cv2.CAP_MSMF)
            
            if not cap.isOpened():
                print(f"[Vision] ERROR: Kamera indeks {self.camera_index} tidak tersedia. Mencari...")
                found_cam_index = self.find_working_camera()
                if found_cam_index is not None:
                    self.camera_index = found_cam_index
                    cap = cv2.VideoCapture(self.camera_index, cv2.CAP_MSMF)
                else:
                    print("[Vision] ❌ Tidak ada kamera, mencoba lagi...")
                    time.sleep(2)
                    continue

            print(f"[Vision] Kamera {self.camera_index} dibuka. Memulai deteksi...")
            
            while self.running and not self.restart_camera:
                try:
                    if not cap.isOpened(): break 
                    ret, im0 = cap.read()
                    if not ret: break

                    annotated_frame = self.process_frame(im0)
                    self.frame_ready.emit(annotated_frame)
                    time.sleep(0.01)
                
                except cv2.error as e:
                    print(f"\n[Vision] Terjadi error OpenCV: {e}. Mencoba menyambung kembali...")
                    break

            cap.release()
            if self.restart_camera:
                print(f"\n[Vision] Me-restart stream kamera...")
        
        print("\n[Vision] Thread layanan visi telah dihentikan.")
    
    def process_frame(self, im0):
        # (Metode process_frame tetap sama)
        results = self.model(im0)
        annotated_frame = results.render()[0].copy()
        detected_red_buoys, detected_green_buoys, detected_green_boxes, detected_blue_boxes = [], [], [], []
        df = results.pandas().xyxy[0]
        for _, row in df.iterrows():
            bbox_data = {
                'xyxy': [row['xmin'], row['ymin'], row['xmax'], row['ymax']],
                'center': (int((row['xmin'] + row['xmax']) / 2), int((row['ymin'] + row['ymax']) / 2)),
                'class': row['name']
            }
            if "red_buoy" in row['name']: detected_red_buoys.append(bbox_data)
            elif "green_buoy" in row['name']: detected_green_buoys.append(bbox_data)
            if "green_box" in row['name']: detected_green_boxes.append(bbox_data)
            elif "blue_box" in row['name']: detected_blue_boxes.append(bbox_data)
        if self.debug:
            total_dets = len(detected_red_buoys) + len(detected_green_buoys) + len(detected_green_boxes) + len(detected_blue_boxes)
            print(f"\r[Vision][DEBUG] Frame: {total_dets} deteksi", end="")
        self.handle_photography_mission(im0, detected_green_boxes, detected_blue_boxes)
        if self.mode_auto:
            self.handle_auto_control(im0, annotated_frame, detected_red_buoys, detected_green_buoys)
        return annotated_frame

    def handle_photography_mission(self, original_frame, green_boxes, blue_boxes):
        # (Tidak ada perubahan di sini)
        mission_name = "Surface Imaging" if green_boxes else "Underwater Imaging" if blue_boxes else None
        if not mission_name: return
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
            threading.Thread(target=upload_image_to_supabase, args=(buffer, filename, self.config), daemon=True).start()

    # --- PERUBAHAN UTAMA: Logika deteksi canggih diintegrasikan di sini ---
    def handle_auto_control(self, im0, annotated_frame, red_buoys, green_buoys):
        detection_config = self.config.get("camera_detection", {})
        PANJANG_FOKUS_PIKSEL = detection_config.get("focal_length_pixels", 600)
        TARGET_JARAK_CM = detection_config.get("target_activation_distance_cm", 100.0)
        LEBAR_BOLA_TUNGGAL_CM = detection_config.get("single_ball_real_width_cm", 10.0)
        LEBAR_BOLA_GANDA_CM = detection_config.get("dual_ball_real_width_cm", 200.0)
        
        target_object = None
        target_midpoint = None
        lebar_asli_cm = 0

        # --- Logika untuk mendeteksi pasangan buoy ---
        if red_buoys and green_buoys:
            best_pair = max(((r, g) for r in red_buoys for g in green_buoys), key=lambda pair: self.get_score(pair[0], is_pair=True, other_ball=pair[1]))
            target_object = best_pair
            red_ball, green_ball = best_pair
            target_midpoint = ((red_ball['center'][0] + green_ball['center'][0]) // 2, (red_ball['center'][1] + green_ball['center'][1]) // 2)
            lebar_asli_cm = LEBAR_BOLA_GANDA_CM
        
        # --- Logika untuk mendeteksi buoy tunggal dan membuat target virtual ---
        elif red_buoys or green_buoys:
            best_single_ball = max(red_buoys + green_buoys, key=self.get_score)
            is_left_buoy = 'red_buoy' in best_single_ball['class']
            if self.is_inverted:
                is_left_buoy = 'green_buoy' in best_single_ball['class']
            
            lebar_objek_piksel_single = best_single_ball['xyxy'][2] - best_single_ball['xyxy'][0]
            jarak_estimasi_cm_single = (LEBAR_BOLA_TUNGGAL_CM * PANJANG_FOKUS_PIKSEL) / lebar_objek_piksel_single if lebar_objek_piksel_single > 0 else 0
            
            # Buat target virtual di samping buoy yang terdeteksi
            pixel_offset = (100.0 * PANJANG_FOKUS_PIKSEL) / jarak_estimasi_cm_single if jarak_estimasi_cm_single > 0 else 0
            ball_center_x, ball_center_y = best_single_ball['center']
            virtual_target_x = ball_center_x + int(pixel_offset) if is_left_buoy else ball_center_x - int(pixel_offset)
            
            target_object = (best_single_ball,)
            target_midpoint = (virtual_target_x, ball_center_y)
            lebar_asli_cm = LEBAR_BOLA_TUNGGAL_CM # Jarak dihitung berdasarkan buoy tunggal

            # Gambar target virtual untuk debugging
            cv2.circle(annotated_frame, target_midpoint, 10, (255, 255, 0), -1)
            cv2.putText(annotated_frame, "Virtual Target", (target_midpoint[0] + 15, target_midpoint[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # --- Logika aktivasi berdasarkan jarak ---
        if target_object:
            annotator = Annotator(annotated_frame, line_width=2, example="Jarak")
            x1 = min(t['xyxy'][0] for t in target_object)
            y1 = min(t['xyxy'][1] for t in target_object)
            x2 = max(t['xyxy'][2] for t in target_object)
            y2 = max(t['xyxy'][3] for t in target_object)
            
            lebar_objek_piksel = x2 - x1
            jarak_estimasi_cm = (lebar_asli_cm * PANJANG_FOKUS_PIKSEL) / lebar_objek_piksel if lebar_objek_piksel > 0 else 0
            
            annotator.box_label([x1, y1, x2, y2], f"Jarak: {jarak_estimasi_cm:.1f} cm")
            annotated_frame = annotator.result()
            
            batas_posisi_y = im0.shape[0] * 0.5
            if 0 < jarak_estimasi_cm < TARGET_JARAK_CM and y1 > batas_posisi_y:
                print(f"ZONA AKTIVASI TERPENUHI (Jarak: {jarak_estimasi_cm:.1f} cm). Mengikuti objek.")
                degree = self.calculate_degree(im0, target_midpoint)
                servo_angle, motor_pwm = self.convert_degree_to_actuators(degree)
                command_string = f"S{motor_pwm};D{servo_angle}\n"
                vision_payload = {'status': 'ACTIVE', 'command': command_string}
                self.asv_handler.process_command("VISION_OVERRIDE", vision_payload)
            else:
                self.asv_handler.process_command("VISION_OVERRIDE", {'status': 'INACTIVE'})
        else:
            self.asv_handler.process_command("VISION_OVERRIDE", {'status': 'INACTIVE'})


    def find_target_midpoint(self, im0, red_buoys, green_buoys):
        # (Ini adalah fungsi helper yang dipanggil oleh handle_auto_control, tidak berubah)
        if red_buoys and green_buoys:
            best_pair = max(((r, g) for r in red_buoys for g in green_buoys), key=lambda pair: self.get_score(pair[0], is_pair=True, other_ball=pair[1]))
            red_ball, green_ball = best_pair
            return ((red_ball['center'][0] + green_ball['center'][0]) // 2, (red_ball['center'][1] + green_ball['center'][1]) // 2)
        elif red_buoys or green_buoys:
            best_single_ball = max(red_buoys + green_buoys, key=self.get_score)
            return best_single_ball['center']
        return None

    def stop(self):
        self.running = False
        print("[Vision] Perintah untuk menghentikan layanan visi diterima.")

    def find_working_camera(self, max_indices=5):
        for idx in range(max_indices):
            test_cap = cv2.VideoCapture(idx, cv2.CAP_MSMF)
            if test_cap.isOpened():
                test_cap.release()
                return idx
        return None

    def set_mode(self, mode):
        self.mode_auto = (mode == "AUTO")
        print(f"\n[Vision] Mode AUTO diatur ke: {self.mode_auto}")

    def set_inversion(self, is_inverted):
        self.is_inverted = bool(is_inverted)
        print(f"\n[Vision] Logika invers diatur ke: {self.is_inverted}")
        
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
        bar_left, bar_right = 50, w - 50
        relative_pos = np.clip((midpoint[0] - bar_left) / (bar_right - bar_left), 0, 1)
        return int(relative_pos * 180)

    def convert_degree_to_actuators(self, degree):
        error = degree - 90
        if self.is_inverted: error = -error
        actuator_config = self.config.get("actuators", {})
        servo_default = actuator_config.get("servo_default_angle", 90)
        servo_range = servo_default - actuator_config.get("servo_min_angle", 45)
        servo_angle = servo_default - (error / 90.0) * servo_range
        servo_angle = int(np.clip(servo_angle, 0, 180))
        motor_base = actuator_config.get("motor_pwm_auto_base", 1650)
        reduction = actuator_config.get("motor_pwm_auto_reduction", 100)
        motor_pwm = motor_base - (abs(error) / 90.0) * reduction
        return int(servo_angle), int(motor_pwm)