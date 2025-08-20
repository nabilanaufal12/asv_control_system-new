# backend/services/vision_service.py
# --- MODIFIKASI: Memperbaiki logika placeholder frame untuk mencegah error 500 ---

import cv2
import torch
import warnings
import threading
import time
from pathlib import Path
import os
import pathlib
import requests
import numpy as np
import sys
import io
from contextlib import redirect_stderr

from backend.vision.overlay_utils import create_overlay_from_html, apply_overlay

if os.name == 'nt':
    pathlib.PosixPath = pathlib.WindowsPath

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
    except Exception:
        pass

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
    except Exception:
        pass

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
        self.camera_index = 1 # Default ke kamera 1
        self.restart_camera = False

        self._initialize_model()

    def _initialize_model(self):
        weights_path = Path(__file__).parents[1] / "yolov5" / "besto.pt"
        try:
            print("[Vision] Mempersiapkan model YOLOv5...")
            with redirect_stderr(io.StringIO()):
                self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=str(weights_path), force_reload=True)
            self.model.conf = 0.4
            self.model.iou = 0.45
            print("[Vision] Model YOLOv5 berhasil dimuat.")
        except Exception as e:
            print(f"[Vision] KRITIS: Gagal memuat model YOLOv5. Error: {e}")
            self.model = None

    def start(self):
        if self.model is None:
            return
        self.running = True
        self.thread.start()
        
    def run(self):
        from ultralytics.utils.plotting import Annotator
        
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

                results = self.model(im0)
                annotated_frame = results.render()[0].copy()
                
                with self.lock:
                    self.output_frame = annotated_frame.copy()
            
            cap.release()
            if self.restart_camera:
                print(f"[Vision] Me-restart stream kamera...")
                with self.lock: self.output_frame = None
            else:
                print("[Vision] Thread layanan visi dihentikan.")

    def stop(self):
        self.running = False

    def get_frame(self):
        """Menyediakan frame terbaru untuk API video stream."""
        with self.lock:
            frame_to_encode = self.output_frame
            # --- PERBAIKAN DI SINI ---
            # Jika frame belum ada, buat placeholder di sini.
            if frame_to_encode is None:
                frame_to_encode = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame_to_encode, "Menunggu Kamera...", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode)
            if not flag:
                return None # Jika encoding gagal, jangan kirim apa-apa
        return encodedImage.tobytes()
        # --- AKHIR PERBAIKAN ---

    def list_available_cameras(self):
        index = 0
        arr = []
        while True:
            cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
            if not cap.isOpened(): break
            arr.append(index)
            cap.release()
            index += 1
            if index > 5:
                break
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
            self.mode_auto = bool(payload)
            print(f"[Vision] Mode AUTO diatur ke: {self.mode_auto}")

    def get_score(self, ball, is_pair=False, other_ball=None):
        size=(ball['xyxy'][2]-ball['xyxy'][0])*(ball['xyxy'][3]-ball['xyxy'][1])
        if is_pair and other_ball:
            other_size=(other_ball['xyxy'][2]-other_ball['xyxy'][0])*(other_ball['xyxy'][3]-other_ball['xyxy'][1])
            avg_y=(ball['center'][1]+other_ball['center'][1])/2;return 0.6*(size+other_size)+0.4*avg_y
        else:
            y_pos=ball['center'][1];return 0.6*size+0.4*y_pos

    def calculate_degree(self, frame, midpoint):
        h,w,_=frame.shape;bar_left,bar_right=50,w-50;relative_pos=np.clip((midpoint[0]-bar_left)/(bar_right-bar_left),0,1);return int(relative_pos*180)

    def convert_degree_to_actuators(self, degree):
        error=degree-90
        if self.is_inverted:error=-error
        servo_angle=max(0,min(180,int(90-(error/90.0)*45)));motor_pwm=1600-((abs(error)/90.0)*100);return int(servo_angle),int(motor_pwm)