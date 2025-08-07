# gui/components/camera/detection_thread.py
# --- FINAL: Penamaan file sekuensial & pengiriman telemetri 500ms ---

import sys
import os
import cv2
import torch
import numpy as np
import time
from pathlib import Path
import requests 
import threading

# --- JARING PENGAMAN: Tambahkan patch ini di baris paling atas ---
if os.name == 'nt': # Hanya untuk Windows
    import pathlib
    pathlib.PosixPath = pathlib.WindowsPath

# Impor fungsi overlay dari file terpisah di folder yang sama
from .overlay_utils import create_overlay_from_html, apply_overlay

# Blok impor untuk YOLOv5
try:
    CURRENT_FILE_DIR = Path(os.path.abspath(__file__)).resolve()
    PROJECT_ROOT = CURRENT_FILE_DIR.parents[3]
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"Peringatan: Gagal mengimpor modul: {e}")
    YOLO_AVAILABLE = False

from PySide6.QtCore import QThread, Signal, Slot

# --- FUNGSI BARU UNTUK FIREBASE & SUPABASE ---

def send_telemetry_to_firebase(telemetry_data):
    """Mengirim data telemetri (dict) ke Firebase Realtime Database."""
    try:
        FIREBASE_URL = 'https://asv-2025-default-rtdb.asia-southeast1.firebasedatabase.app/monitor.json'
        
        data_to_send = {
            "gps": {
                "lat": telemetry_data.get('latitude', 0.0),
                "lng": telemetry_data.get('longitude', 0.0)
            },
            "hdg": telemetry_data.get('heading', 0.0),
            "cog": telemetry_data.get('cog', 0.0),
            "sog": telemetry_data.get('speed', 0.0),
            "jam": time.strftime("%H:%M:%S"),
        }

        response = requests.put(FIREBASE_URL, json=data_to_send, timeout=5)
        if response.ok:
            print(f"✅ Telemetri dikirim ke Firebase @ {data_to_send['jam']}")
        else:
            print(f"🔥 Gagal mengirim telemetri: {response.status_code}")
    except Exception as e:
        print(f"🔥 Error koneksi Firebase: {e}")

def upload_image_to_supabase(image_buffer, filename):
    """Mengunggah buffer gambar ke Supabase Storage."""
    try:
        BUCKET = 'navantara'
        ENDPOINT = f"https://ngmicoyombtgtlegdjzn.supabase.co/storage/v1/object/{BUCKET}/{filename}"
        TOKEN = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Im5nbWljb3lvbWJ0Z3RsZWdkanpuIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTM5NjkwOTMsImV4cCI6MjA2OTU0NTA5M30.tz7_mfVaX_5sWO3xtSQ1h85JAIRrL6iC4ZNm0TaUTdI'
        
        headers = {
            'Authorization': f'Bearer {TOKEN}',
            'Content-Type': 'image/jpeg',
            'x-upsert': 'true',
        }
        
        response = requests.post(ENDPOINT, headers=headers, data=image_buffer.tobytes(), timeout=10)
        
        if response.ok:
            print(f"✅ Gambar '{filename}' berhasil diunggah ke Supabase.")
        else:
            print(f"🔥 Gagal mengunggah gambar: {response.status_code} {response.text}")
    except Exception as e:
        print(f"🔥 Error koneksi Supabase: {e}")


class DetectionThread(QThread):
    frame_ready = Signal(np.ndarray)
    vision_command_status = Signal(dict)

    def __init__(self, source_idx, weights_path, parent=None):
        super().__init__(parent)
        self.source_idx, self.weights_path = source_idx, weights_path
        self.running, self.mode_auto, self.is_inverted = True, False, False
        
        self.snapshot_dir = PROJECT_ROOT / "snapshots"
        os.makedirs(self.snapshot_dir, exist_ok=True)
        print(f"Gambar akan disimpan di: {self.snapshot_dir}")
        
        self.latest_telemetry = {}

        self.firebase_thread = threading.Thread(target=self.firebase_updater, daemon=True)

        # --- PERUBAHAN DI SINI: Tambahkan counter untuk nama file ---
        self.surface_image_count = 1
        self.underwater_image_count = 1

    def firebase_updater(self):
        """Fungsi yang berjalan di thread terpisah untuk mengirim data setiap 500ms."""
        while self.running:
            if self.latest_telemetry:
                send_telemetry_to_firebase(self.latest_telemetry)
            time.sleep(0.5)

    @Slot(dict)
    def update_telemetry(self, data):
        self.latest_telemetry = data

    def get_score(self, ball, is_pair=False, other_ball=None):
        size = (ball['xyxy'][2] - ball['xyxy'][0]) * (ball['xyxy'][3] - ball['xyxy'][1])
        if is_pair and other_ball:
            other_size = (other_ball['xyxy'][2] - other_ball['xyxy'][0]) * (other_ball['xyxy'][3] - other_ball['xyxy'][1])
            avg_y = (ball['center'][1] + other_ball['center'][1]) / 2
            return 0.6 * (size + other_size) + 0.4 * avg_y
        else:
            y_pos = ball['center'][1]
            return 0.6 * size + 0.4 * y_pos
            
    def run(self):
        self.firebase_thread.start()

        cap = None
        try:
            print("Mempersiapkan model YOLOv5...")
            model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.weights_path, force_reload=True)
            model.conf = 0.4
            model.iou = 0.45
            
            from ultralytics.utils.plotting import Annotator

            print(f"Mencoba membuka kamera indeks: {self.source_idx}...")
            cap = cv2.VideoCapture(self.source_idx, cv2.CAP_DSHOW)
            if not cap.isOpened():
                print(f"ERROR: Tidak bisa membuka kamera {self.source_idx}.")
                return
            
            print("Kamera dan model berhasil dimuat. Memulai deteksi...")
            while self.running and cap.isOpened():
                ret, im0 = cap.read()
                if not ret: 
                    print("Peringatan: Gagal membaca frame dari kamera. Menghentikan thread.")
                    break

                results = model(im0)
                results.render()
                annotated_frame = results.ims[0]
                
                detected_red_buoys, detected_green_buoys = [], []
                detected_green_boxes, detected_blue_boxes = [], []

                df = results.pandas().xyxy[0]
                for _, row in df.iterrows():
                    class_name = row['name']
                    bbox_data = {
                        'xyxy': [row['xmin'], row['ymin'], row['xmax'], row['ymax']],
                        'center': (int((row['xmin'] + row['xmax']) / 2), int((row['ymin'] + row['ymax']) / 2)),
                        'class': class_name
                    }
                    if "red_buoy" in class_name: detected_red_buoys.append(bbox_data)
                    elif "green_buoy" in class_name: detected_green_buoys.append(bbox_data)
                    if "green_box" in class_name: detected_green_boxes.append(bbox_data)
                    elif "blue_box" in class_name: detected_blue_boxes.append(bbox_data)

                mission_name = None
                if detected_green_boxes: mission_name = "Surface Imaging"
                elif detected_blue_boxes: mission_name = "Underwater Imaging"
                
                if mission_name:
                    overlay_image = create_overlay_from_html(self.latest_telemetry, mission_type=mission_name)
                    snapshot_image = apply_overlay(im0.copy(), overlay_image)
                    
                    # --- PERUBAHAN DI SINI: Logika penamaan file sekuensial ---
                    supabase_filename = ""
                    if mission_name == "Surface Imaging":
                        supabase_filename = f"surface_{self.surface_image_count}.jpg"
                        self.surface_image_count += 1
                    elif mission_name == "Underwater Imaging":
                        supabase_filename = f"underwater_{self.underwater_image_count}.jpg"
                        self.underwater_image_count += 1

                    ret, buffer = cv2.imencode('.jpg', snapshot_image)
                    if ret:
                        upload_image_to_supabase(buffer, supabase_filename)
                    else:
                        print("🔥 Gagal meng-encode gambar ke JPEG.")

                if self.mode_auto:
                    target_object, target_midpoint, lebar_asli_cm = None, None, 0
                    best_pair = None
                    if detected_red_buoys and detected_green_buoys:
                        best_score = -1
                        for red in detected_red_buoys:
                            for green in detected_green_buoys:
                                score = self.get_score(red, is_pair=True, other_ball=green)
                                if score > best_score:
                                    best_score = score
                                    best_pair = (red, green)
                    if best_pair:
                        print("MODE: Dua Bola Terdeteksi.")
                        target_object, red_ball, green_ball = best_pair, best_pair[0], best_pair[1]
                        target_midpoint = ((red_ball['center'][0] + green_ball['center'][0]) // 2, (red_ball['center'][1] + green_ball['center'][1]) // 2)
                        lebar_asli_cm = 200.0
                    elif detected_red_buoys or detected_green_buoys:
                        print("MODE: Satu Bola Terdeteksi.")
                        all_balls = detected_red_buoys + detected_green_buoys
                        best_single_ball = max(all_balls, key=lambda b: self.get_score(b))
                        target_object, target_midpoint, lebar_asli_cm = (best_single_ball,), best_single_ball['center'], 10.0

                    if target_object:
                        distance_annotator = Annotator(annotated_frame, line_width=2, example="Jarak")
                        
                        if len(target_object) == 2:
                            x1, y1 = min(target_object[0]['xyxy'][0], target_object[1]['xyxy'][0]), min(target_object[0]['xyxy'][1], target_object[1]['xyxy'][1])
                            x2, y2 = max(target_object[0]['xyxy'][2], target_object[1]['xyxy'][2]), max(target_object[0]['xyxy'][3], target_object[1]['xyxy'][3])
                        else:
                            x1, y1, x2, y2 = target_object[0]['xyxy']
                        
                        PANJANG_FOKUS_PIKSEL, lebar_objek_piksel = 600, x2 - x1
                        jarak_estimasi_cm = (lebar_asli_cm * PANJANG_FOKUS_PIKSEL) / lebar_objek_piksel if lebar_objek_piksel > 0 else 0
                        distance_annotator.box_label([x1, y1, x2, y2], f"Jarak: {jarak_estimasi_cm:.1f} cm")
                        annotated_frame = distance_annotator.result()

                        batas_posisi_y, TARGET_JARAK_CM = im0.shape[0] * 0.5, 100.0
                        if jarak_estimasi_cm > 0 and jarak_estimasi_cm < TARGET_JARAK_CM and y1 > batas_posisi_y:
                            print(f"ZONA AKTIVASI TERPENUHI (Jarak: {jarak_estimasi_cm:.1f} cm). Mengikuti objek.")
                            degree = self.calculate_degree(im0, target_midpoint)
                            servo_angle, motor_pwm = self.convert_degree_to_actuators(degree)
                            print(f"   => Perintah Dihitung: Motor PWM={motor_pwm}, Servo Angle={servo_angle}")
                            self.vision_command_status.emit({'status': 'ACTIVE', 'command': f"S{motor_pwm};D{servo_angle}\n"})
                        else: self.vision_command_status.emit({'status': 'INACTIVE'})
                    else: self.vision_command_status.emit({'status': 'INACTIVE'})

                self.frame_ready.emit(annotated_frame)
                
        except Exception as e:
            print(f"Error di dalam thread deteksi: {e}")
        finally:
            self.stop()
            if cap and cap.isOpened(): cap.release()
            print("Pembersihan thread deteksi selesai.")

    def stop(self): 
        self.running = False
    
    def calculate_degree(self, frame, midpoint):
        h, w, _ = frame.shape
        bar_y, bar_left, bar_right = h - 50, 50, w - 50
        bar_width = bar_right - bar_left
        relative_pos = np.clip((midpoint[0] - bar_left) / bar_width, 0, 1)
        return int(relative_pos * 180)

    def convert_degree_to_actuators(self, degree):
        error = degree - 90
        if self.is_inverted: error = -error
        servo_angle = 90 - (error / 90.0) * 45
        servo_angle = max(0, min(180, int(servo_angle)))
        speed_reduction = abs(error) / 90.0
        motor_pwm = 1600 - (speed_reduction * 100)
        return int(servo_angle), int(motor_pwm)