# src/navantara_backend/services/vision_service.py
# --- VERSI MODIFIKASI DENGAN LOGIKA POSISI LAYAR ---
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
        self.gate_activation_distance = vision_cfg.get("gate_activation_distance_cm", 200.0)
        self.obstacle_activation_distance = vision_cfg.get("obstacle_activation_distance_cm", 120.0)
        
        self.recent_detections = collections.deque(maxlen=self.poi_validation_frames)
        self.investigation_in_progress = False
        self.last_buoy_seen_time = 0
        self.obstacle_cooldown_period = 2.0

        cam_detect_cfg = self.config.get("camera_detection", {})
        self.FOCAL_LENGTH_PIXELS = cam_detect_cfg.get("focal_length_pixels", 600)
        self.OBJECT_REAL_WIDTHS_CM = cam_detect_cfg.get("object_real_widths_cm", {})

    def _estimate_distance(self, pixel_width, object_class):
        if object_class not in self.OBJECT_REAL_WIDTHS_CM or pixel_width == 0:
            return None
        real_width_cm = self.OBJECT_REAL_WIDTHS_CM[object_class]
        return (real_width_cm * self.FOCAL_LENGTH_PIXELS) / pixel_width

    def _draw_distance_info(self, frame, detections):
        buoy_detections = [d for d in detections if "buoy" in d["class"]]
        for det in buoy_detections:
            if "distance_cm" in det and det["distance_cm"] is not None:
                x1, y1, _, _ = map(int, det["xyxy"])
                distance_text = f"{det['distance_cm']:.1f} cm"
                cv2.putText(frame, distance_text, (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        red_buoy = next((d for d in buoy_detections if d["class"] == "red_buoy"), None)
        green_buoy = next((d for d in buoy_detections if d["class"] == "green_buoy"), None)
        if red_buoy and green_buoy:
            pass
        return frame

    def _validate_buoy_color(self, frame, detection):
        """
        Memvalidasi ulang warna buoy menggunakan analisis HSV di dalam bounding box.
        Mengembalikan nama kelas yang benar ('red_buoy' atau 'green_buoy').
        """
        # Hanya validasi jika kelasnya adalah salah satu dari buoy
        if "buoy" not in detection["class"]:
            return detection["class"]

        try:
            # 1. Crop area bounding box dari frame asli
            x1, y1, x2, y2 = map(int, detection["xyxy"])
            
            # Pastikan koordinat valid
            if x1 < 0 or y1 < 0 or x2 > frame.shape[1] or y2 > frame.shape[0] or x1 >= x2 or y1 >= y2:
                return detection["class"] # Kembalikan kelas asli jika bbox tidak valid
                
            roi = frame[y1:y2, x1:x2]

            # 2. Konversi area yang di-crop ke HSV
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # 3. Definisikan rentang warna (Hue) untuk MERAH dan HIJAU
            # Catatan: Merah melintasi batas 0/180 di HSV, jadi butuh dua rentang
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            
            lower_green = np.array([35, 100, 50]) # Hue untuk hijau lebih sempit
            upper_green = np.array([85, 255, 255])

            # 4. Buat mask untuk setiap warna
            mask_red1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv_roi, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            mask_green = cv2.inRange(hsv_roi, lower_green, upper_green)

            # 5. Hitung jumlah piksel non-nol untuk setiap warna
            red_pixel_count = cv2.countNonZero(mask_red)
            green_pixel_count = cv2.countNonZero(mask_green)

            # 6. Tentukan klasifikasi berdasarkan warna dominan
            # Jika piksel hijau jauh lebih dominan, koreksi menjadi 'green_buoy'
            if green_pixel_count > red_pixel_count * 1.5: # Tambahkan margin 1.5x
                # print(f"[Color Validation] Koreksi: {detection['class']} -> green_buoy")
                return "green_buoy"
            # Sebaliknya, jika piksel merah lebih dominan, koreksi (atau pertahankan) menjadi 'red_buoy'
            elif red_pixel_count > green_pixel_count * 1.5:
                # print(f"[Color Validation] Koreksi: {detection['class']} -> red_buoy")
                return "red_buoy"
                
            # Jika tidak ada warna yang dominan, percaya pada model
            return detection["class"]

        except Exception as e:
            # Jika terjadi error (misal, bounding box kosong), percaya pada model
            # print(f"[Color Validation] Error: {e}")
            return detection["class"]

    def run_capture_loops(self):
        if self.inference_engine.model is None: return
        self.running = True
        self.socketio.start_background_task(self._capture_loop, self.camera_index_1, "frame_cam1", True)
        self.socketio.start_background_task(self._capture_loop, self.camera_index_2, "frame_cam2", False)

    def stop(self):
        self.running = False
    
    def _capture_loop(self, cam_index, event_name, apply_detection):
        cap = None
        while self.running:
            try:
                cap = cv2.VideoCapture(cam_index)
                if not cap.isOpened(): self.socketio.sleep(5); continue
                while self.running:
                    if not cap.isOpened(): break
                    ret, frame = cap.read()
                    if not ret: break
                    with self.settings_lock:
                        should_emit_frame = self.gui_is_listening
                    processed_frame = frame
                    if apply_detection:
                        processed_frame = self.process_and_control(frame, True)
                    if should_emit_frame:
                        ret_encode, buffer = cv2.imencode(".jpg", processed_frame)
                        if ret_encode: self.socketio.emit(event_name, buffer.tobytes())
                    self.socketio.sleep(0.02)
            finally:
                if cap and cap.isOpened(): cap.release()
                if self.running: self.socketio.sleep(5)

    def process_and_control(self, frame, is_mode_auto):
        with self.asv_handler.state_lock:
            current_state = self.asv_handler.current_state.copy()
        
        detections, annotated_frame = self.inference_engine.infer(frame)

        buoy_classes = ["red_buoy", "green_buoy"]
        # Buat list baru untuk menyimpan deteksi yang sudah divalidasi
        validated_detections = []
        
        for det in detections:
            # Validasi ulang klasifikasi warna untuk buoy
            if det["class"] in buoy_classes:
                original_class = det["class"]
                # Panggil fungsi validasi baru kita, menggunakan frame asli (non-annotated)
                validated_class = self._validate_buoy_color(frame, det)
                if original_class != validated_class:
                    print(f"🎨 [Color Correction] YOLO class '{original_class}' dikoreksi menjadi '{validated_class}'")
                det["class"] = validated_class

            # Hitung jarak setelah kelasnya divalidasi
            if det["class"] in buoy_classes:
                pixel_width = det["xyxy"][2] - det["xyxy"][0]
                det["distance_cm"] = self._estimate_distance(pixel_width, det["class"])
            
            validated_detections.append(det)

        # Ganti 'detections' lama dengan yang sudah divalidasi
        detections = validated_detections

        if is_mode_auto:
            potential_pois = [d for d in detections if d["class"] not in self.KNOWN_CLASSES and d["confidence"] > self.poi_confidence_threshold]
            if potential_pois and not self.investigation_in_progress:
                best_poi = max(potential_pois, key=lambda p: p["confidence"])
                self.validate_and_trigger_investigation(best_poi, frame, current_state)
            else:
                self.recent_detections.clear()

            #detected_green_boxes = [d for d in detections if "green_box" in d["class"]]
            #detected_blue_boxes = [d for d in detections if "blue_box" in d["class"]]
            #if detected_green_boxes or detected_blue_boxes:
            #    self.handle_photography_mission(frame, detected_green_boxes, detected_blue_boxes, current_state)
            
            # Sekarang handle_autonomous_navigation akan menerima deteksi yang sudah dikoreksi warnanya
            self.handle_autonomous_navigation(detections, frame.shape[1], current_state)

        # Anotasi ulang frame dengan data yang sudah divalidasi
        final_frame = self.inference_engine._annotate_frame(frame.copy(), detections)
        final_frame = self._draw_distance_info(final_frame, detections)
        return final_frame

    # --- FUNGSI LOGIKA NAVIGASI (MODIFIKASI) ---
    def handle_autonomous_navigation(self, detections, frame_width, current_state):
        red_buoys = [d for d in detections if d["class"] == "red_buoy"]
        green_buoys = [d for d in detections if d["class"] == "green_buoy"]

        # === PRIORITAS 1: LOGIKA GERBANG ===
        # Cek jika KEDUA bola terdeteksi untuk membentuk sebuah gerbang.
        if red_buoys and green_buoys:
            red, green = red_buoys[0], green_buoys[0]
            
            # Hitung lebar gerbang dalam piksel untuk estimasi jarak.
            gate_pixel_width = max(red["xyxy"][2], green["xyxy"][2]) - min(red["xyxy"][0], green["xyxy"][0])
            gate_distance = self._estimate_distance(gate_pixel_width, "buoy_gate")

            # Jika gerbang cukup dekat, aktifkan mode melewati gerbang.
            if gate_distance is not None and gate_distance < self.gate_activation_distance:
                self.last_buoy_seen_time = time.time() # Perbarui timer untuk mencegah recovery prematur.
                
                # Tentukan titik tengah gerbang, yang menjadi target navigasi.
                gate_center_x = (red["center"][0] + green["center"][0]) / 2.0
                
                # Kirim perintah spesifik untuk melewati gerbang ke asv_handler.
                # Payload ini memberikan semua informasi yang dibutuhkan untuk manuver presisi.
                self.asv_handler.process_command(
                    "GATE_TRAVERSAL_COMMAND", {
                        "active": True,
                        "gate_center_x": gate_center_x,
                        "frame_width": frame_width
                    }
                )
                # Setelah perintah gerbang dikirim, hentikan pemrosesan lebih lanjut di frame ini.
                # Ini adalah implementasi dari "utamakan gerbang".
                return

        # === PRIORITAS 2: LOGIKA PENGHINDARAN RINTANGAN TUNGGAL ===
        # Blok ini hanya akan dieksekusi JIKA logika gerbang di atas tidak terpenuhi.
        all_buoys = red_buoys + green_buoys
        if all_buoys:
            closest_buoy = min(all_buoys, key=lambda b: b.get('distance_cm', float('inf')))
            distance_to_closest = closest_buoy.get('distance_cm', float('inf'))
            
            if distance_to_closest < self.obstacle_activation_distance:
                self.last_buoy_seen_time = time.time() # Perbarui timer.
                
                # Kirim perintah penghindaran rintangan generik ke asv_handler.
                self.asv_handler.process_command(
                    "VISION_TARGET_UPDATE", {
                        "active": True,
                        "obstacle_class": closest_buoy.get("class", "unknown"),
                        "object_center_x": closest_buoy["center"][0],
                        "frame_width": frame_width
                    }
                )
                return

        # === KONDISI DEFAULT: TIDAK ADA RINTANGAN ===
        # Jika tidak ada rintangan yang aktif dalam jangkauan selama periode cooldown,
        # nonaktifkan semua mode AI agar ASV bisa kembali ke navigasi waypoint.
        if (time.time() - self.last_buoy_seen_time) > self.obstacle_cooldown_period:
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})
            # Pastikan perintah gerbang juga dinonaktifkan untuk kebersihan state.
            self.asv_handler.process_command("GATE_TRAVERSAL_COMMAND", {"active": False})
    
    def handle_photography_mission(self, original_frame, green_boxes, blue_boxes, current_state):
        mission_name = "Surface Imaging" if green_boxes else "Underwater Imaging"
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
            bearing_offset = (obj_center_x - frame_center_x) * (60 / frame.shape[1])
            absolute_bearing = (current_state.get("heading", 0) + bearing_offset) % 360
            payload = {"class_name": first_detection_class, "confidence": poi_data["confidence"], "bearing_deg": absolute_bearing}
            self.asv_handler.process_command("INVESTIGATE_POI", payload)
            self.recent_detections.clear()

    def set_gui_listening(self, status: bool):
        with self.settings_lock:
            if self.gui_is_listening != status:
                self.gui_is_listening = status

    def set_mode(self, mode: str):
        with self.settings_lock:
            new_mode_auto = mode == "AUTO"
            if self.mode_auto != new_mode_auto:
                self.mode_auto = new_mode_auto

    def set_inversion(self, is_inverted: bool):
        with self.settings_lock:
            if self.is_inverted != is_inverted:
                self.is_inverted = is_inverted