# backend/services/vision_service.py
# --- VERSI FINAL: Menggabungkan logika deteksi, misi, dan kontrol ---

import cv2
import torch
import threading
import time
from pathlib import Path
import os
import pathlib
import requests
import numpy as np
import io
from contextlib import redirect_stderr

# Impor utilitas dan fungsi helper
from backend.vision.overlay_utils import create_overlay_from_html, apply_overlay

# Mengatasi masalah path di Windows
if os.name == 'nt':
    pathlib.PosixPath = pathlib.WindowsPath

# --- Fungsi Helper untuk Komunikasi Eksternal ---

def send_telemetry_to_firebase(telemetry_data, config):
    """Mengirim data telemetri ke Firebase Realtime Database."""
    try:
        FIREBASE_URL = config.get("api_urls", {}).get("firebase_url")
        if not FIREBASE_URL: return
        data_to_send = {
            "gps": {"lat": telemetry_data.get('latitude', 0.0), "lng": telemetry_data.get('longitude', 0.0)},
            "hdg": telemetry_data.get('heading', 0.0), "cog": telemetry_data.get('cog', 0.0),
            "sog": telemetry_data.get('speed', 0.0), "jam": time.strftime("%H:%M:%S"),
        }
        # Menggunakan timeout untuk mencegah thread macet
        requests.put(FIREBASE_URL, json=data_to_send, timeout=5)
    except Exception:
        # Abaikan error koneksi agar tidak mengganggu proses utama
        pass

def upload_image_to_supabase(image_buffer, filename, config):
    """Mengunggah buffer gambar ke Supabase Storage."""
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

# --- Kelas Utama Vision Service ---

class VisionService:
    def __init__(self, config, asv_handler):
        self.config = config
        self.asv_handler = asv_handler
        self.running = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.output_frame = None
        self.lock = threading.Lock()
        self.model = None
        self.is_inverted = False
        self.mode_auto = False
        self.camera_index = 0 # Default ke kamera utama
        self.restart_camera = False
        self.surface_image_count = 1
        self.underwater_image_count = 1

        self._initialize_model()

    def _initialize_model(self):
        """Mempersiapkan dan memuat model YOLOv5 dari file bobot."""
        weights_path = Path(__file__).parents[1] / "yolov5" / "besto.pt"
        try:
            print("[Vision] Mempersiapkan model YOLOv5...")
            # Mengalihkan output error sementara untuk menyembunyikan pesan yang tidak relevan
            with redirect_stderr(io.StringIO()):
                self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=str(weights_path), force_reload=True)
            self.model.conf = 0.4
            self.model.iou = 0.45
            print("[Vision] Model YOLOv5 berhasil dimuat.")
        except Exception as e:
            print(f"[Vision] KRITIS: Gagal memuat model YOLOv5. Error: {e}")
            self.model = None

    def start(self):
        """Memulai thread deteksi jika model berhasil dimuat."""
        if self.model is None:
            return
        self.running = True
        self.thread.start()
        
    def run(self):
        """Loop utama untuk deteksi objek yang berjalan di background."""
        from ultralytics.utils.plotting import Annotator # Impor di dalam thread
        
        while self.running:
            self.restart_camera = False
            cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
            if not cap.isOpened():
                print(f"[Vision] ERROR: Gagal membuka kamera indeks {self.camera_index}.")
                time.sleep(2)
                continue

            print(f"[Vision] Kamera indeks {self.camera_index} dibuka. Memulai deteksi...")
            
            while self.running and not self.restart_camera:
                ret, im0 = cap.read()
                if not ret:
                    time.sleep(0.01)
                    continue

                # --- INTI LOGIKA DETEKSI DIMULAI DI SINI ---
                
                results = self.model(im0)
                annotated_frame = results.render()[0].copy()
                
                # Parsing hasil deteksi
                detected_red_buoys, detected_green_buoys = [], []
                detected_green_boxes, detected_blue_boxes = [], []
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

                # Logika Misi Fotografi (Surface & Underwater)
                self.handle_photography_mission(im0, detected_green_boxes, detected_blue_boxes)

                # Logika Kontrol Otomatis berbasis Visi
                if self.mode_auto:
                    self.handle_auto_control(im0, annotated_frame, detected_red_buoys, detected_green_buoys)
                
                # --- AKHIR LOGIKA DETEKSI ---

                # Simpan frame hasil anotasi untuk di-stream ke GUI
                with self.lock:
                    self.output_frame = annotated_frame.copy()
            
            cap.release()
            if self.restart_camera:
                print(f"[Vision] Me-restart stream kamera...")
                with self.lock: self.output_frame = None
            else:
                print("[Vision] Thread layanan visi dihentikan.")

    def handle_photography_mission(self, original_frame, green_boxes, blue_boxes):
        """Mencetak foto dengan overlay jika box terdeteksi."""
        mission_name = "Surface Imaging" if green_boxes else "Underwater Imaging" if blue_boxes else None
        if mission_name:
            current_telemetry = self.asv_handler.get_current_state()
            send_telemetry_to_firebase(current_telemetry, self.config) # Update Firebase
            
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
                # Menjalankan upload di thread terpisah agar tidak memblokir
                upload_thread = threading.Thread(target=upload_image_to_supabase, args=(buffer, filename, self.config))
                upload_thread.start()


    def handle_auto_control(self, im0, annotated_frame, red_buoys, green_buoys):
        """Menjalankan logika navigasi otonom berbasis deteksi pelampung."""
        detection_config = self.config.get("camera_detection", {})
        TARGET_JARAK_CM = detection_config.get("target_activation_distance_cm", 100.0)
        
        target_midpoint = self.find_target_midpoint(im0, annotated_frame, red_buoys, green_buoys)

        if target_midpoint:
            # Kalkulasi jarak dan kondisi aktivasi
            # (Untuk simplifikasi, logika jarak bisa ditambahkan di sini jika diperlukan)
            
            # Jika target ditemukan, hitung perintah aktuator
            degree = self.calculate_degree(im0, target_midpoint)
            servo_angle, motor_pwm = self.convert_degree_to_actuators(degree)
            
            # Buat command string dan kirim ke AsvHandler
            command_string = f"S{motor_pwm};D{servo_angle}\n"
            vision_payload = {'status': 'ACTIVE', 'command': command_string}
            self.asv_handler.process_command("VISION_OVERRIDE", vision_payload)
        else:
            # Jika tidak ada target, nonaktifkan override
            self.asv_handler.process_command("VISION_OVERRIDE", {'status': 'INACTIVE'})


    def find_target_midpoint(self, im0, annotated_frame, red_buoys, green_buoys):
        """Menentukan titik target berdasarkan deteksi pelampung."""
        # Logika ini disederhanakan dari detection_thread.py
        if red_buoys and green_buoys:
            # Prioritas utama: pasangan pelampung
            best_pair = max(((r, g) for r in red_buoys for g in green_buoys), 
                            key=lambda pair: self.get_score(pair[0], is_pair=True, other_ball=pair[1]))
            red_ball, green_ball = best_pair
            return ((red_ball['center'][0] + green_ball['center'][0]) // 2, 
                    (red_ball['center'][1] + green_ball['center'][1]) // 2)
        elif red_buoys or green_buoys:
            # Jika hanya satu jenis pelampung, buat target virtual
            # (Logika target virtual bisa ditambahkan di sini jika diperlukan)
            best_single_ball = max(red_buoys + green_buoys, key=self.get_score)
            return best_single_ball['center']
        return None


    def stop(self):
        self.running = False

    def get_frame(self):
        """Menyediakan frame terbaru untuk API video stream."""
        with self.lock:
            frame_to_encode = self.output_frame
            if frame_to_encode is None:
                frame_to_encode = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame_to_encode, "Menunggu Kamera...", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode)
            return encodedImage.tobytes() if flag else None

    def list_available_cameras(self):
        """Memindai dan mengembalikan daftar indeks kamera yang tersedia."""
        arr = []
        for index in range(5): # Cek hingga 5 kamera
            cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
            if cap.isOpened():
                arr.append(index)
                cap.release()
        return arr

    def process_command(self, command, payload):
        """Memproses perintah yang datang dari GUI via endpoint."""
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

    # --- Fungsi Kalkulasi (diambil dari detection_thread.py) ---
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
        # Menghasilkan nilai antara -90 (kiri) dan 90 (kanan)
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

        # PWM motor melambat saat berbelok tajam
        motor_base = actuator_config.get("motor_pwm_auto_base", 1650)
        reduction = actuator_config.get("motor_pwm_auto_reduction", 100)
        motor_pwm = motor_base - (abs(error) / 90.0) * reduction
        
        return int(servo_angle), int(motor_pwm)