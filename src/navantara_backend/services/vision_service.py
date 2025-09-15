# src/navantara_backend/services/vision_service.py
import cv2
import numpy as np
import collections
import time
import traceback
import threading

from navantara_backend.vision.inference_engine import InferenceEngine
from navantara_backend.vision.path_planner import dwa_path_planning
from navantara_backend.vision.cloud_utils import (
    send_telemetry_to_firebase,
    upload_image_to_supabase,
)
from navantara_backend.vision.overlay_utils import (
    create_overlay_from_html,
    apply_overlay,
)


class VisionService:
    def __init__(self, config, asv_handler, socketio):
        self.config = config
        self.asv_handler = asv_handler
        self.socketio = socketio
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

    def run_capture_loops(self):
        if self.inference_engine.model is None:
            print("[Vision] Model tidak tersedia, layanan visi tidak dapat dimulai.")
            return

        self.running = True
        self.socketio.start_background_task(self._capture_loop, self.camera_index_1, 'frame_cam1', True)
        self.socketio.start_background_task(self._capture_loop, self.camera_index_2, 'frame_cam2', False)
        print("[Vision] Greenlet untuk kedua kamera telah dimulai.")

    def stop(self):
        self.running = False
        print("\n[Vision] Perintah stop diterima. Menghentikan semua greenlet kamera.")

    def _capture_loop(self, cam_index, event_name, apply_detection):
        cap = None
        while self.running:
            try:
                cap = cv2.VideoCapture(cam_index)
                if not cap.isOpened():
                    print(f"[Vision Cam-{cam_index}] ❌ Gagal membuka kamera. Mencoba lagi dalam 5 detik...")
                    self.socketio.sleep(5)
                    continue

                print(f"[Vision Cam-{cam_index}] Kamera berhasil dibuka. Memulai stream...")
                while self.running:
                    if not cap.isOpened(): break
                    ret, frame = cap.read()
                    if not ret: break

                    with self.settings_lock:
                        is_mode_auto = self.mode_auto
                        should_emit_frame = self.gui_is_listening
                    
                    if apply_detection:
                        # Proses frame bahkan jika tidak dikirim, agar logika otonom tetap berjalan
                        processed_frame = self.process_and_control(frame, is_mode_auto)
                    else:
                        processed_frame = frame
                    
                    if should_emit_frame:
                        ret_encode, buffer = cv2.imencode('.jpg', processed_frame)
                        if ret_encode:
                            self.socketio.emit(event_name, buffer.tobytes())

                    self.socketio.sleep(0.02)

            except Exception as e:
                print(f"\n[Vision Cam-{cam_index}] KESALAHAN UMUM: {e}")
                traceback.print_exc()
            finally:
                if cap and cap.isOpened():
                    cap.release()
                if self.running:
                    self.socketio.sleep(5)

    def process_and_control(self, frame, is_mode_auto):
        # --- PERBAIKAN DI SINI ---
        # Mengambil salinan state yang aman dari thread
        with self.asv_handler.state_lock:
            current_state = self.asv_handler.current_state.copy()

        current_mission_phase = current_state.get("mission_phase")
        if self.investigation_in_progress and current_mission_phase != "INVESTIGATING":
            print("[Vision] Misi investigasi selesai. Kembali ke mode deteksi normal.")
            self.investigation_in_progress = False
            self.recent_detections.clear()

        detections, annotated_frame = self.inference_engine.infer(frame)

        if is_mode_auto and not self.investigation_in_progress:
            potential_pois = [d for d in detections if d["class"] not in self.KNOWN_CLASSES and d["confidence"] > self.poi_confidence_threshold]
            if potential_pois:
                best_poi = max(potential_pois, key=lambda p: p["confidence"])
                self.validate_and_trigger_investigation(best_poi, frame, current_state)
            else:
                self.recent_detections.clear()

        if is_mode_auto:
            detected_green_boxes = [d for d in detections if "green_box" in d["class"]]
            detected_blue_boxes = [d for d in detections if "blue_box" in d["class"]]
            if detected_green_boxes or detected_blue_boxes:
                self.handle_photography_mission(frame, detected_green_boxes, detected_blue_boxes, current_state)

            if not self.investigation_in_progress:
                detected_red_buoys = [d for d in detections if "red_buoy" in d["class"]]
                detected_green_buoys = [d for d in detections if "green_buoy" in d["class"]]
                self.handle_auto_control_dwa(detected_red_buoys, detected_green_buoys, current_state)

        return annotated_frame

    def handle_auto_control_dwa(self, red_buoys, green_buoys, current_state):
        all_buoys = red_buoys + green_buoys
        if not all_buoys:
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})
            return

        target_detection = max(all_buoys, key=lambda b: (b["xyxy"][2] - b["xyxy"][0]) * (b["xyxy"][3] - b["xyxy"][1]))
        obstacles_relative = [self._convert_detection_to_relative_coords(b) for b in all_buoys if b is not None]
        goal_relative = self._convert_detection_to_relative_coords(target_detection)

        if not obstacles_relative or goal_relative is None: return

        v_opt, omega_opt = dwa_path_planning(current_state, obstacles_relative, goal_relative, self.config)
        self.asv_handler.process_command("PLANNED_MANEUVER", {"active": True, "v_opt": v_opt, "omega_opt": omega_opt})

    def handle_photography_mission(self, original_frame, green_boxes, blue_boxes, current_state):
        mission_name = "Surface Imaging" if green_boxes else "Underwater Imaging"
        send_telemetry_to_firebase(current_state, self.config)
        overlay = create_overlay_from_html(current_state, mission_type=mission_name)
        snapshot = apply_overlay(original_frame.copy(), overlay)
        filename = f"surface_{self.surface_image_count}.jpg" if green_boxes else f"underwater_{self.underwater_image_count}.jpg"
        
        if green_boxes: self.surface_image_count += 1
        else: self.underwater_image_count += 1

        _, buffer = cv2.imencode(".jpg", snapshot)
        if buffer is not None:
            self.socketio.start_background_task(upload_image_to_supabase, buffer, filename, self.config)

    def validate_and_trigger_investigation(self, poi_data, frame, current_state):
        self.recent_detections.append(poi_data["class"])
        if len(self.recent_detections) < self.poi_validation_frames: return

        if all(cls == self.recent_detections[0] for cls in self.recent_detections):
            first_detection_class = self.recent_detections[0]
            print(f"[Vision] VALIDASI BERHASIL: Objek '{first_detection_class}' terdeteksi.")
            self.investigation_in_progress = True

            frame_center_x = frame.shape[1] / 2
            obj_center_x = poi_data["center"][0]
            fov_horizontal = 60
            bearing_offset = (obj_center_x - frame_center_x) * (fov_horizontal / frame.shape[1])
            absolute_bearing = (current_state.get("heading", 0) + bearing_offset) % 360
            
            payload = {"class_name": first_detection_class, "confidence": poi_data["confidence"], "bearing_deg": absolute_bearing}
            self.asv_handler.process_command("INVESTIGATE_POI", payload)
            self.recent_detections.clear()

    def _convert_detection_to_relative_coords(self, detection):
        cam_cfg = self.config.get("camera_detection", {})
        F = cam_cfg.get("focal_length_pixels")
        real_widths = cam_cfg.get("object_real_widths_cm", {})
        obj_class = detection.get("class")
        if obj_class not in real_widths and "buoy" in obj_class: obj_class = "buoy_single"
        W = real_widths.get(obj_class)
        if not W: return None
        P = detection["xyxy"][2] - detection["xyxy"][0]
        if P == 0: return None
        dist_m = (W / 100.0 * F) / P
        center_x_pixel = (self.inference_engine.model.input_shape[3] if hasattr(self.inference_engine.model, 'input_shape') else 640) / 2
        dx_pixel = detection["center"][0] - center_x_pixel
        return [(dx_pixel * dist_m) / F, dist_m]

    def set_gui_listening(self, status: bool):
        with self.settings_lock:
            if self.gui_is_listening != status:
                print(f"[VisionService] Status pendengar GUI diatur ke: {status}")
                self.gui_is_listening = status

    def set_mode(self, mode: str):
        with self.settings_lock:
            new_mode_auto = mode == "AUTO"
            if self.mode_auto != new_mode_auto:
                self.mode_auto = new_mode_auto
                print(f"\n[Vision] Mode AUTO diatur ke: {self.mode_auto}")
                if not self.mode_auto:
                    self.investigation_in_progress = False
                    self.recent_detections.clear()

    def set_inversion(self, is_inverted: bool):
        with self.settings_lock:
            if self.is_inverted != is_inverted:
                self.is_inverted = is_inverted
                print(f"\n[Vision] Inversi diubah secara manual menjadi: {self.is_inverted}")