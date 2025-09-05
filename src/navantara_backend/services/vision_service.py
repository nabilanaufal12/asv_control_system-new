# navantara_backend/services/vision_service.py
"""
File utama untuk Vision Service.
Bertindak sebagai orkestrator yang mengelola thread kamera, state,
dan mendelegasikan tugas-tugas kompleks ke modul lain seperti
InferenceEngine, PathPlanner, dan CloudUtils.
"""

# 1. Impor Pustaka Standar & Pihak Ketiga
import cv2
import numpy as np
import collections
import threading
import time
import pathlib
import os
import sys
import traceback
from PySide6.QtCore import QObject, Signal, Slot

# 2. Impor Modul Lokal yang Telah Dipecah
from navantara_backend.vision.inference_engine import InferenceEngine
from navantara_backend.vision.path_planner import dwa_path_planning
from navantara_backend.vision.cloud_utils import send_telemetry_to_firebase, upload_image_to_supabase
from navantara_backend.vision.overlay_utils import create_overlay_from_html, apply_overlay

# 3. Penyesuaian Path (Diperlukan oleh InferenceEngine)
if os.name == "nt": pathlib.PosixPath = pathlib.WindowsPath
backend_dir = Path(__file__).resolve().parents[1]
yolov5_path = backend_dir / "yolov5"
if str(yolov5_path) not in sys.path: sys.path.insert(0, str(yolov5_path))


class VisionService(QObject):
    frame_ready_cam1 = Signal(np.ndarray)
    frame_ready_cam2 = Signal(np.ndarray)

    def __init__(self, config, asv_handler):
        super().__init__()
        self.config = config
        self.asv_handler = asv_handler
        self.running = False
        
        # Inisialisasi engine inferensi dari modul terpisah
        self.inference_engine = InferenceEngine(config, yolov5_path)

        # State & Konfigurasi Service
        self.gui_is_listening = False
        self.is_inverted = False
        self.mode_auto = False
        self.surface_image_count = 1
        self.underwater_image_count = 1

        vision_cfg = self.config.get("vision", {})
        self.camera_index_1 = int(vision_cfg.get("camera_index_1", 0))
        self.camera_index_2 = int(vision_cfg.get("camera_index_2", 1))
        self.KNOWN_CLASSES = vision_cfg.get("known_classes", [])
        self.poi_confidence_threshold = vision_cfg.get("poi_confidence_threshold", 0.65)
        self.poi_validation_frames = vision_cfg.get("poi_validation_frames", 5)
        self.recent_detections = collections.deque(maxlen=self.poi_validation_frames)
        self.investigation_in_progress = False

    # =================================================================================
    # KONTROL UTAMA SERVICE & THREAD
    # =================================================================================

    @Slot()
    def start(self):
        """Memulai thread-thread untuk capture kamera."""
        if self.inference_engine.model is None:
            print("[Vision] Model tidak tersedia, layanan visi tidak dapat dimulai.")
            return
        
        self.running = True
        self.thread1 = threading.Thread(target=self._capture_loop, args=(self.camera_index_1, self.frame_ready_cam1, True), daemon=True)
        self.thread2 = threading.Thread(target=self._capture_loop, args=(self.camera_index_2, self.frame_ready_cam2, False), daemon=True)
        self.thread1.start()
        self.thread2.start()
        print("[Vision] Thread untuk kedua kamera telah dimulai.")

    def stop(self):
        """Menghentikan semua thread yang berjalan."""
        self.running = False
        print("\n[Vision] Perintah stop diterima. Menghentikan semua thread kamera.")

    def _capture_loop(self, cam_index, frame_signal, apply_detection):
        """Loop utama yang menangkap frame dari kamera secara terus-menerus."""
        cap = None
        while self.running:
            try:
                print(f"[Vision Cam-{cam_index}] Mencoba membuka kamera...")
                cap = cv2.VideoCapture(cam_index)
                if not cap.isOpened():
                    print(f"[Vision Cam-{cam_index}] ❌ Gagal membuka kamera. Mencoba lagi dalam 5 detik...")
                    time.sleep(5)
                    continue

                print(f"[Vision Cam-{cam_index}] Kamera berhasil dibuka. Memulai stream...")
                while self.running:
                    if not cap.isOpened(): break
                    ret, frame = cap.read()
                    if not ret:
                        print(f"[Vision Cam-{cam_-cam_index}] Peringatan: Gagal membaca frame.")
                        break
                    
                    processed_frame = self.process_and_control(frame) if apply_detection else frame
                    
                    if self.gui_is_listening:
                        frame_signal.emit(processed_frame)
                    
                    time.sleep(0.01) # Jeda singkat agar tidak membebani CPU
            
            except Exception as e:
                print(f"\n[Vision Cam-{cam_index}] ERROR di dalam loop deteksi: {e}")
                traceback.print_exc()
            finally:
                if cap and cap.isOpened(): cap.release()
                if self.running:
                    print(f"[Vision Cam-{cam_index}] Stream terputus, akan mencoba memulai ulang.")
                    time.sleep(2)

    # =================================================================================
    # ALUR KERJA PEMROSESAN & KONTROL
    # =================================================================================

    def process_and_control(self, frame):
        """
        Fungsi orkestrasi utama: inferensi, menjalankan misi, dan kontrol otonom.
        """
        # 1. Lakukan inferensi menggunakan engine
        detections, annotated_frame = self.inference_engine.infer(frame)

        # 2. Filter deteksi berdasarkan kelas untuk logika selanjutnya
        detected_red_buoys = [d for d in detections if "red_buoy" in d['class']]
        detected_green_buoys = [d for d in detections if "green_buoy" in d['class']]
        detected_green_boxes = [d for d in detections if "green_box" in d['class']]
        detected_blue_boxes = [d for d in detections if "blue_box" in d['class']]
        potential_pois = [d for d in detections if d['class'] not in self.KNOWN_CLASSES and d['confidence'] > self.poi_confidence_threshold]

        # 3. Jalankan logika misi berdasarkan deteksi
        if potential_pois:
            self.validate_and_trigger_investigation(potential_pois[0], frame)
        
        if self.mode_auto and (detected_green_boxes or detected_blue_boxes):
            self.handle_photography_mission(frame, detected_green_boxes, detected_blue_boxes)

        # 4. Jalankan logika kontrol otonom jika mode AUTO aktif
        if self.mode_auto and not self.investigation_in_progress:
            self.handle_auto_control_dwa(detected_red_buoys, detected_green_buoys)
        
        return annotated_frame

    def handle_auto_control_dwa(self, red_buoys, green_buoys):
        """Mengelola logika otonom menggunakan DWA Path Planner."""
        all_buoys = red_buoys + green_buoys
        target_midpoint = None

        # Tentukan target (contoh sederhana: buoy terbesar)
        if all_buoys:
            best_buoy = max(all_buoys, key=lambda b: (b['xyxy'][2] - b['xyxy'][0]) * (b['xyxy'][3] - b['xyxy'][1]))
            target_midpoint = best_buoy['center']

        if not target_midpoint:
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})
            return

        # Konversi piksel ke koordinat relatif (ini adalah placeholder, butuh kalibrasi)
        obstacles_relative = [self._convert_pixel_to_relative_coords(b['center']) for b in all_buoys]
        goal_relative = self._convert_pixel_to_relative_coords(target_midpoint)

        # Panggil DWA Planner dari modul terpisah
        current_state = self.asv_handler.get_current_state()
        v_opt, omega_opt = dwa_path_planning(current_state, obstacles_relative, goal_relative, self.config)

        # Konversi output DWA (v, ω) ke perintah aktuator (servo, motor)
        # ... (Logika konversi yang spesifik untuk ASV Anda) ...
        
        # Kirim perintah manuver terencana ke ASV Handler
        payload = {"active": True, "v_opt": v_opt, "omega_opt": omega_opt}
        self.asv_handler.process_command("PLANNED_MANEUVER", payload)

    # =================================================================================
    # LOGIKA MISI & HELPER
    # =================================================================================

    def handle_photography_mission(self, original_frame, green_boxes, blue_boxes):
        """Menangani misi fotografi saat box terdeteksi."""
        mission_name = "Surface Imaging" if green_boxes else "Underwater Imaging"
        current_telemetry = self.asv_handler.get_current_state()
        
        # Panggil fungsi dari cloud_utils
        send_telemetry_to_firebase(current_telemetry, self.config)
        
        overlay = create_overlay_from_html(current_telemetry, mission_type=mission_name)
        snapshot = apply_overlay(original_frame.copy(), overlay)
        
        filename = f"surface_{self.surface_image_count}.jpg" if green_boxes else f"underwater_{self.underwater_image_count}.jpg"
        if green_boxes: self.surface_image_count += 1
        else: self.underwater_image_count += 1

        _, buffer = cv2.imencode(".jpg", snapshot)
        if buffer is not None:
            # Panggil fungsi dari cloud_utils dalam thread baru
            threading.Thread(target=upload_image_to_supabase, args=(buffer, filename, self.config), daemon=True).start()

    def validate_and_trigger_investigation(self, poi_data, frame):
        """Memvalidasi objek anomali dan memulai misi investigasi."""
        # ... (Logika validasi tidak berubah)
        pass

    def _convert_pixel_to_relative_coords(self, pixel_coord):
        """PLACEHOLDER: Mengonversi piksel ke koordinat dunia nyata (meter)."""
        # Implementasi ini sangat bergantung pada kalibrasi kamera Anda.
        # Untuk sekarang, ini hanya mengembalikan nilai dummy.
        return [float(pixel_coord[0]), float(pixel_coord[1])]

    # =================================================================================
    # SLOT UNTUK KOMUNIKASI DENGAN GUI
    # =================================================================================

    @Slot(bool)
    def set_gui_listening(self, status: bool):
        self.gui_is_listening = status

    @Slot(str)
    def set_mode(self, mode):
        self.mode_auto = mode == "AUTO"
        print(f"\n[Vision] Mode AUTO diatur ke: {self.mode_auto}")
        if not self.mode_auto:
            self.investigation_in_progress = False
            self.recent_detections.clear()

    @Slot(bool)
    def set_inversion(self, is_inverted):
        if self.is_inverted != is_inverted:
            self.is_inverted = is_inverted
            print(f"\n[Vision] Inversi diubah secara manual menjadi: {self.is_inverted}")