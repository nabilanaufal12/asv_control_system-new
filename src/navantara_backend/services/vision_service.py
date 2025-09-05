# navantara_backend/services/vision_service.py
# --- VERSI FINAL DENGAN VALIDASI POI ---

import cv2
import numpy as np
import collections
import threading
import time
import traceback
from PySide6.QtCore import QObject, Signal, Slot

from navantara_backend.vision.inference_engine import InferenceEngine
from navantara_backend.vision.path_planner import dwa_path_planning
from navantara_backend.vision.cloud_utils import send_telemetry_to_firebase, upload_image_to_supabase
from navantara_backend.vision.overlay_utils import create_overlay_from_html, apply_overlay

class VisionService(QObject):
    frame_ready_cam1 = Signal(np.ndarray)
    frame_ready_cam2 = Signal(np.ndarray)

    def __init__(self, config, asv_handler):
        super().__init__()
        self.config = config
        self.asv_handler = asv_handler
        self.running = False
        self.inference_engine = InferenceEngine(config)
        self.settings_lock = threading.Lock()
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
                        print(f"[Vision Cam-{cam_index}] Peringatan: Gagal membaca frame. Stream mungkin terputus.")
                        break
                    
                    with self.settings_lock:
                        is_mode_auto = self.mode_auto
                        should_emit_frame = self.gui_is_listening

                    processed_frame = self.process_and_control(frame, is_mode_auto) if apply_detection else frame
                    
                    if should_emit_frame:
                        frame_signal.emit(processed_frame)
                    
                    time.sleep(0.01)
            
            except cv2.error as e:
                print(f"\n[Vision Cam-{cam_index}] KESALAHAN OPENCV: Terjadi masalah dengan perangkat keras kamera atau driver.")
                print(f"  > Detail: {e}")
                traceback.print_exc()
            except Exception as e:
                print(f"\n[Vision Cam-{cam_index}] KESALAHAN UMUM: Terjadi error tak terduga di dalam loop kamera.")
                print(f"  > Tipe Error: {type(e).__name__}, Pesan: {e}")
                traceback.print_exc()
            finally:
                if cap and cap.isOpened(): cap.release()
                if self.running:
                    print(f"[Vision Cam-{cam_index}] Stream terputus, akan mencoba memulai ulang dalam 5 detik.")
                    time.sleep(5)

    def process_and_control(self, frame, is_mode_auto):
        """
        Fungsi orkestrasi utama: inferensi, menjalankan misi, dan kontrol otonom.
        """
        # Sinkronisasi status: jika asv_handler selesai investigasi, reset flag di sini
        current_mission_phase = self.asv_handler.get_current_state().get("mission_phase")
        if self.investigation_in_progress and current_mission_phase != "INVESTIGATING":
            print("[Vision] Misi investigasi selesai. Kembali ke mode deteksi normal.")
            self.investigation_in_progress = False
            self.recent_detections.clear()

        detections, annotated_frame = self.inference_engine.infer(frame)

        # Hanya validasi POI jika mode AUTO dan tidak sedang investigasi
        if is_mode_auto and not self.investigation_in_progress:
            potential_pois = [d for d in detections if d['class'] not in self.KNOWN_CLASSES and d['confidence'] > self.poi_confidence_threshold]
            if potential_pois:
                best_poi = max(potential_pois, key=lambda p: p['confidence'])
                self.validate_and_trigger_investigation(best_poi, frame)
            else:
                self.recent_detections.clear() # Bersihkan history jika tidak ada POI

        # Logika misi fotografi dan DWA tetap berjalan seperti biasa
        if is_mode_auto:
            detected_green_boxes = [d for d in detections if "green_box" in d['class']]
            detected_blue_boxes = [d for d in detections if "blue_box" in d['class']]
            if detected_green_boxes or detected_blue_boxes:
                self.handle_photography_mission(frame, detected_green_boxes, detected_blue_boxes)

            if not self.investigation_in_progress:
                detected_red_buoys = [d for d in detections if "red_buoy" in d['class']]
                detected_green_buoys = [d for d in detections if "green_buoy" in d['class']]
                self.handle_auto_control_dwa(detected_red_buoys, detected_green_buoys)
        
        return annotated_frame

    def handle_auto_control_dwa(self, red_buoys, green_buoys):
        """Mengelola logika otonom menggunakan DWA Path Planner."""
        all_buoys = red_buoys + green_buoys
        target_detection = None

        if all_buoys:
            target_detection = max(all_buoys, key=lambda b: (b['xyxy'][2] - b['xyxy'][0]) * (b['xyxy'][3] - b['xyxy'][1]))
        
        if not target_detection:
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})
            return

        obstacles_relative = [self._convert_detection_to_relative_coords(b) for b in all_buoys]
        goal_relative = self._convert_detection_to_relative_coords(target_detection)
        
        obstacles_relative = [obs for obs in obstacles_relative if obs is not None]
        if not obstacles_relative or goal_relative is None: return

        current_state = self.asv_handler.get_current_state()
        v_opt, omega_opt = dwa_path_planning(current_state, obstacles_relative, goal_relative, self.config)
        
        payload = {"active": True, "v_opt": v_opt, "omega_opt": omega_opt}
        self.asv_handler.process_command("PLANNED_MANEUVER", payload)

    def handle_photography_mission(self, original_frame, green_boxes, blue_boxes):
        """Menangani misi fotografi saat box terdeteksi."""
        mission_name = "Surface Imaging" if green_boxes else "Underwater Imaging"
        current_telemetry = self.asv_handler.get_current_state()
        
        send_telemetry_to_firebase(current_telemetry, self.config)
        
        overlay = create_overlay_from_html(current_telemetry, mission_type=mission_name)
        snapshot = apply_overlay(original_frame.copy(), overlay)
        
        filename = f"surface_{self.surface_image_count}.jpg" if green_boxes else f"underwater_{self.underwater_image_count}.jpg"
        if green_boxes: self.surface_image_count += 1
        else: self.underwater_image_count += 1

        _, buffer = cv2.imencode(".jpg", snapshot)
        if buffer is not None:
            threading.Thread(target=upload_image_to_supabase, args=(buffer, filename, self.config), daemon=True).start()

    def validate_and_trigger_investigation(self, poi_data, frame):
        """
        Memvalidasi objek anomali dan memulai misi investigasi jika terdeteksi konsisten.
        """
        self.recent_detections.append(poi_data['class'])

        if len(self.recent_detections) < self.poi_validation_frames:
            return

        first_detection_class = self.recent_detections[0]
        is_consistent = all(cls == first_detection_class for cls in self.recent_detections)

        if is_consistent:
            print(f"[Vision] VALIDASI BERHASIL: Objek '{first_detection_class}' terdeteksi secara konsisten.")
            self.investigation_in_progress = True

            frame_center_x = frame.shape[1] / 2
            obj_center_x = poi_data['center'][0]
            fov_horizontal = 60 
            degrees_per_pixel = fov_horizontal / frame.shape[1]
            bearing_offset = (obj_center_x - frame_center_x) * degrees_per_pixel
            
            current_heading = self.asv_handler.get_current_state().get("heading", 0)
            absolute_bearing = (current_heading + bearing_offset) % 360

            command_payload = {
                "class_name": first_detection_class,
                "confidence": poi_data['confidence'],
                "bearing_deg": absolute_bearing
            }
            self.asv_handler.process_command("INVESTIGATE_POI", command_payload)
            
            self.recent_detections.clear()

    def _convert_detection_to_relative_coords(self, detection):
        """
        Mengestimasi koordinat dunia nyata (X, Y) dalam meter dari sebuah deteksi.
        """
        cam_cfg = self.config.get("camera_detection", {})
        F = cam_cfg.get("focal_length_pixels")
        real_widths = cam_cfg.get("object_real_widths_cm", {})

        obj_class = detection.get('class')
        if obj_class not in real_widths and "buoy" in obj_class:
            obj_class = "buoy_single"
        
        W = real_widths.get(obj_class)
        if not W: return None
        
        x1, _, x2, _ = detection['xyxy']
        P = x2 - x1
        if P == 0: return None

        dist_m = ((W / 100.0) * F) / P

        try:
            frame_width = self.inference_engine.model.input_shape[3]
        except (AttributeError, IndexError):
            frame_width = 640

        center_x_pixel = frame_width / 2
        obj_center_x_pixel = detection['center'][0]
        dx_pixel = obj_center_x_pixel - center_x_pixel
        
        x_m = (dx_pixel * dist_m) / F
        y_m = dist_m
        
        return [x_m, y_m]

    @Slot(bool)
    def set_gui_listening(self, status: bool):
        with self.settings_lock:
            self.gui_is_listening = status

    @Slot(str)
    def set_mode(self, mode):
        with self.settings_lock:
            new_mode_auto = (mode == "AUTO")
            if self.mode_auto != new_mode_auto:
                self.mode_auto = new_mode_auto
                print(f"\n[Vision] Mode AUTO diatur ke: {self.mode_auto}")
                if not self.mode_auto:
                    self.investigation_in_progress = False
                    self.recent_detections.clear()

    @Slot(bool)
    def set_inversion(self, is_inverted):
        with self.settings_lock:
            if self.is_inverted != is_inverted:
                self.is_inverted = is_inverted
                print(f"\n[Vision] Inversi diubah secara manual menjadi: {self.is_inverted}")