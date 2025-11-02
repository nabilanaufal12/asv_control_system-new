# src/navantara_backend/services/vision_service.py
# --- VERSI FINAL DENGAN DUA KAMERA, PAYLOAD GERBANG, DAN PERBAIKAN IMPORT ---
import cv2
import numpy as np
import collections
import time
import traceback
import threading
import eventlet  # Pastikan eventlet diimpor
import os
import eventlet.tpool # <-- Diimpor
import logging # <-- [DEBUG] Dipertahankan

# Impor dari modul lain di backend Anda
from navantara_backend.vision.inference_engine import InferenceEngine

# Hapus import path_planner jika tidak digunakan di sini
# from navantara_backend.vision.path_planner import dwa_path_planning

# --- MODIFIKASI: Seluruh blok import cloud_utils di-comment ---
# from navantara_backend.vision.cloud_utils import (
#     # send_telemetry_to_firebase, # Firebase dipanggil dari asv_handler
#     # upload_image_to_supabase,
# )
# --- AKHIR MODIFIKASI ---

from navantara_backend.vision.overlay_utils import (
    create_overlay_from_html,
    apply_overlay,
)

# --- HAPUS IMPORT Slot ---
# from PySide6.QtCore import Slot # Hapus import ini
# --- AKHIR HAPUSAN ---


class VisionService:
    # --- Variabel Class untuk menyimpan frame terbaru & locks ---
    _latest_processed_frame_cam1 = None  # Frame CAM 1 (Hasil AI)
    _frame_lock_cam1 = threading.Lock()  # Lock untuk CAM 1
    _latest_raw_frame_cam2 = None  # Frame CAM 2 (Mentah)
    _frame_lock_cam2 = threading.Lock()  # Lock untuk CAM 2
    # Ganti nama variabel lama agar konsisten (opsional, bisa tetap _latest_processed_frame dll)
    _latest_processed_frame = _latest_processed_frame_cam1
    _frame_lock = _frame_lock_cam1
    # -----------------------------------------------------------

    def __init__(self, config, asv_handler, socketio):
        self.config = config
        self.asv_handler = asv_handler
        self.socketio = socketio
        self.running = False
        self.inference_engine = InferenceEngine(config)
        self.settings_lock = threading.Lock()  # Lock untuk pengaturan dari GUI

        # Pengaturan awal
        self.gui_is_listening = False
        self.is_inverted = False
        # --- [FIX] HAPUS STATE LOKAL YANG SUDAH USANG ---
        # self.mode_auto = False 
        # self.mode_auto = False 
        # Pengaturan dari config.json
        vision_cfg = self.config.get("vision", {})
        self.camera_index_1 = int(vision_cfg.get("camera_index_1", 0))
        self.camera_index_2 = int(vision_cfg.get("camera_index_2", 1))
        self.KNOWN_CLASSES = vision_cfg.get("known_classes", [])
        self.poi_confidence_threshold = vision_cfg.get("poi_confidence_threshold", 0.65)
        self.poi_validation_frames = vision_cfg.get("poi_validation_frames", 5)
        self.gate_activation_distance = vision_cfg.get(
            "gate_activation_distance_cm", 200.0
        )
        self.obstacle_activation_distance = vision_cfg.get(
            "obstacle_activation_distance_cm", 120.0
        )

        # State internal
        self.recent_detections = collections.deque(maxlen=self.poi_validation_frames)
        self.investigation_in_progress = False
        self.last_buoy_seen_time = 0
        self.obstacle_cooldown_period = vision_cfg.get(
            "obstacle_cooldown_period_sec", 2.0
        )  # Ambil dari config
        # self.show_local_feed = vision_cfg.get("show_local_video_feed", False)
        self.show_local_feed = False

        self.surface_image_count = 0
        self.underwater_image_count = 0

        # Pengaturan deteksi kamera
        cam_detect_cfg = self.config.get("camera_detection", {})
        self.FOCAL_LENGTH_PIXELS = cam_detect_cfg.get("focal_length_pixels", 600)
        self.OBJECT_REAL_WIDTHS_CM = cam_detect_cfg.get("object_real_widths_cm", {})

        print("[VisionService] Layanan Visi diinisialisasi.")

    # --- KODE LAMA ANDA UNTUK ESTIMASI JARAK, DLL (DIPERTAHANKAN) ---
    def _estimate_distance(self, pixel_width, object_class):
        real_width_cm = self.OBJECT_REAL_WIDTHS_CM.get(object_class)
        # Fallback ke 'buoy' jika kelas spesifik tidak ada
        if not real_width_cm:
            real_width_cm = self.OBJECT_REAL_WIDTHS_CM.get("buoy")

        if not real_width_cm or pixel_width <= 0:
            return None
        return (real_width_cm * self.FOCAL_LENGTH_PIXELS) / pixel_width

    def _draw_distance_info(self, frame, detections):
        buoy_detections = [d for d in detections if "buoy" in d.get("class", "")]
        for det in buoy_detections:
            distance_cm = det.get("distance_cm")
            if distance_cm is not None:
                x1, y1, _, _ = map(int, det.get("xyxy", [0, 0, 0, 0]))
                distance_text = f"{distance_cm:.1f} cm"
                try:
                    from navantara_backend.vision.overlay_utils import (
                        putText_with_shadow,
                    )

                    putText_with_shadow(
                        frame,
                        distance_text,
                        (x1, y1 - 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        thickness=2,
                    )
                except ImportError:
                    cv2.putText(
                        frame,
                        distance_text,
                        (x1, y1 - 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        2,
                    )
        red_buoy = next(
            (d for d in buoy_detections if d.get("class") == "red_buoy"), None
        )
        green_buoy = next(
            (d for d in buoy_detections if d.get("class") == "green_buoy"), None
        )
        if red_buoy and green_buoy:
            pass
        return frame

    def _validate_buoy_color(self, frame, detection):
        """Validates and determines the color of detected buoys with improved thresholds and logging."""
        if "buoy" not in detection.get("class", ""):
            return detection.get("class")
        try:
            x1, y1, x2, y2 = map(int, detection["xyxy"])
            h, w = frame.shape[:2]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x1 >= x2 or y1 >= y2:
                logging.warning("[Vision] Invalid bounding box dimensions")
                return detection.get("class")

            roi = frame[y1:y2, x1:x2]
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # Adjusted HSV thresholds for more reliable color detection
            lower_red1 = np.array([0, 100, 60])     # Slightly relaxed saturation & value
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 60])   # For the wrap-around hue of red
            upper_red2 = np.array([180, 255, 255])
            lower_green = np.array([35, 70, 35])    # Wider green range
            upper_green = np.array([85, 255, 255])
            
            mask_r = cv2.bitwise_or(
                cv2.inRange(hsv_roi, lower_red1, upper_red1),
                cv2.inRange(hsv_roi, lower_red2, upper_red2),
            )
            mask_g = cv2.inRange(hsv_roi, lower_green, upper_green)
            
            red_px = cv2.countNonZero(mask_r)
            green_px = cv2.countNonZero(mask_g)
            total_px = roi.shape[0] * roi.shape[1]
            
            # Calculate percentages for better decision making
            red_percentage = (red_px / total_px) * 100
            green_percentage = (green_px / total_px) * 100
            
            # Debug color detection
            logging.info(f"[Vision] Color Analysis - Red: {red_percentage:.1f}%, Green: {green_percentage:.1f}%")
            
            # More robust color classification with minimum thresholds
            if green_percentage > 15 and green_px > red_px * 1.2:  # Lowered ratio requirement
                result = "green_buoy"
            elif red_percentage > 15 and red_px > green_px * 1.2:  # Lowered ratio requirement
                result = "red_buoy"
            else:
                result = detection.get("class")
                
            logging.info(f"[Vision] Buoy classified as: {result}")
            return result
        except Exception as e:
            print(f"[ColorValidation] Error: {e}")
            return detection.get("class")

    # --- AKHIR KODE LAMA ANDA ---

    # --- MODIFIKASI: Memulai DUA greenlet capture ---
    def run_capture_loops(self):
        """Memulai greenlet terpisah untuk menangkap frame dari kedua kamera."""
        if not self.running:
            self.running = True
            print("[VisionService] Memulai loop penangkapan untuk kedua kamera...")

            eventlet.spawn(
                self._capture_loop,
                self.camera_index_1,  # Indeks Kamera 1
                VisionService._frame_lock_cam1,  # Lock untuk CAM 1
                "CAM1",  # Identifier logging
                "frame_cam1",  # Event WebSocket GUI CAM 1
                True,  # Terapkan AI
            )
            eventlet.spawn(
                self._capture_loop,
                self.camera_index_2,  # Indeks Kamera 2
                VisionService._frame_lock_cam2,  # Lock untuk CAM 2
                "CAM2",  # Identifier logging
                "frame_cam2",  # Event WebSocket GUI CAM 2
                False,  # JANGAN Terapkan AI
            )
            print("[VisionService] Kedua loop penangkapan telah dijadwalkan.")
        else:
            print("[VisionService] Loop penangkapan sudah berjalan.")

    def stop(self):
        """Menghentikan semua loop penangkapan."""
        print("[VisionService] Menghentikan loop penangkapan...")
        self.running = False

        if self.show_local_feed:
            cv2.destroyAllWindows()

    # --- MODIFIKASI: Fungsi _capture_loop generik untuk kedua kamera ---
    def _capture_loop(
        self, cam_index, frame_lock, cam_id_log, event_name, apply_detection
    ):
        """Loop utama untuk menangkap frame dari satu kamera."""
        cap = None
        print(
            f"[{cam_id_log}] Memulai loop untuk kamera index {cam_index} (AI: {apply_detection})..."
        )

        LOCAL_FEED_WIDTH = 320  # Ubah ukuran ini (misal: 320, 480)
        LOCAL_FEED_HEIGHT = 240  # Ubah ukuran ini (misal: 240, 320)

        while self.running:
            frame = None
            try:
                if cap is None or not cap.isOpened():
                    print(f"[{cam_id_log}] Mencoba membuka kamera index {cam_index}...")
                    backends = [cv2.CAP_ANY, cv2.CAP_V4L2, cv2.CAP_GSTREAMER]
                    for backend in backends:
                        cap = cv2.VideoCapture(cam_index, backend)
                        if cap.isOpened():
                            print(
                                f"[{cam_id_log}] Kamera berhasil dibuka (Backend: {backend})."
                            )
                            break
                    if cap is None or not cap.isOpened():
                        print(
                            f"[{cam_id_log}] Gagal membuka kamera. Mencoba lagi dalam 5 detik..."
                        )
                        eventlet.sleep(5)
                        continue

                ret, frame = cap.read()

                if not ret or frame is None:
                    print(
                        f"[{cam_id_log}] Gagal membaca frame. Menutup & mencoba membuka ulang..."
                    )
                    if cap.isOpened():
                        cap.release()
                    cap = None
                    eventlet.sleep(1)
                    continue

                # --- [PERBAIKAN ARSITEKTURAL UTAMA] ---
                # Baca state GUI dari 'settings_lock'
                with self.settings_lock:
                    should_emit_to_gui = self.gui_is_listening
                
                # Baca state KONTROL dari 'asv_handler.state_lock'
                # Ini adalah SUMBER KEBENARAN TUNGGAL (Single Source of Truth)
                with self.asv_handler.state_lock:
                    is_auto = self.asv_handler.current_state.get("control_mode") == "AUTO"
                # --- [AKHIR PERBAIKAN] ---


                if apply_detection:
                    
                    # --- [FIX] UBAH BAGIAN INI ---
                    # Hapus tpool.execute() dari sini. Buat panggilan langsung.
                    # tpool akan dipanggil DI DALAM process_and_control
                    try:
                        processed_frame_ai = self.process_and_control(frame, is_auto) # <-- 'is_auto' sekarang sudah benar
                    except Exception as e:
                        print(f"[{cam_id_log}] Error dalam process_and_control: {e}")
                        traceback.print_exc()
                        eventlet.sleep(1) 
                        continue
                    # --- [AKHIR FIX] ---

                    with frame_lock:  # _frame_lock_cam1
                        VisionService._latest_processed_frame_cam1 = (
                            processed_frame_ai.copy()
                        )
                        VisionService._latest_processed_frame = (
                            VisionService._latest_processed_frame_cam1
                        )  # Update var lama juga
                    frame_to_emit = processed_frame_ai
                else:
                    with frame_lock:  # _frame_lock_cam2
                        VisionService._latest_raw_frame_cam2 = frame.copy()
                    frame_to_emit = frame

                if should_emit_to_gui:
                    # --- OPTIMASI DI SINI ---
                    # Kecilkan frame sebelum dikirim ke GUI
                    try:
                        gui_frame = cv2.resize(frame_to_emit, (640, 480)) # atau (480, 320)
                    except:
                        gui_frame = frame_to_emit # Fallback

                    ret_encode, buffer = cv2.imencode(
                        ".jpg", gui_frame, [cv2.IMWRITE_JPEG_QUALITY, 60] # Kualitas 60
                    )
                    
                    # CATATAN: Anda memiliki dua 'imencode' berturut-turut. Ini mungkin tidak disengaja.
                    # Saya akan menghapus yang kedua yang berkualitas lebih rendah.
                    # ret_encode, buffer = cv2.imencode(
                    #     ".jpg", frame_to_emit, [cv2.IMWRITE_JPEG_QUALITY, 30]
                    # )
                    
                    if ret_encode:
                        self.socketio.emit(event_name, buffer.tobytes())

                if self.show_local_feed:
                    if apply_detection:  # HANYA thread CAM1 (AI) yang akan menampilkan
                        try:
                            # 1. Siapkan frame CAM1 (AI) - KIRI
                            frame_cam1_resized = cv2.resize(
                                frame_to_emit, (LOCAL_FEED_WIDTH, LOCAL_FEED_HEIGHT)
                            )

                            # 2. Ambil dan siapkan frame CAM2 (Raw) - KANAN
                            frame_cam2 = None
                            with VisionService._frame_lock_cam2:  # Kunci thread CAM2
                                if VisionService._latest_raw_frame_cam2 is not None:
                                    frame_cam2 = (
                                        VisionService._latest_raw_frame_cam2.copy()
                                    )

                            if frame_cam2 is not None:
                                frame_cam2_resized = cv2.resize(
                                    frame_cam2, (LOCAL_FEED_WIDTH, LOCAL_FEED_HEIGHT)
                                )
                            else:
                                # Buat frame hitam jika CAM2 belum siap
                                frame_cam2_resized = np.zeros(
                                    (LOCAL_FEED_HEIGHT, LOCAL_FEED_WIDTH, 3),
                                    dtype=np.uint8,
                                )
                                cv2.putText(
                                    frame_cam2_resized,
                                    "CAM 2 (Waiting)",
                                    (
                                        LOCAL_FEED_WIDTH // 2 - 60,
                                        LOCAL_FEED_HEIGHT // 2,
                                    ),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    (255, 255, 255),
                                    1,
                                )

                            # 4. Gabungkan (stack) kedua frame secara horizontal
                            # (CAM1 di kiri, CAM2 di kanan)
                            combined_frame = np.hstack(
                                (frame_cam1_resized, frame_cam2_resized)
                            )

                            # 5. Tampilkan frame gabungan dalam SATU jendela
                            cv2.imshow(
                                "NAVANTARA - Local Monitor (AI | Raw)", combined_frame
                            )

                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                self.stop()
                                break

                        except Exception as e:
                            print(
                                f"[{cam_id_log}] Error resizing/showing combined feed: {e}"
                            )

                    # Thread CAM2 (apply_detection == False) tidak melakukan apa-apa
                    # Ini penting agar hanya ada SATU thread yang mengontrol imshow

                eventlet.sleep(0.02)  # Target ~50 FPS

            except cv2.error as e:
                print(f"[{cam_id_log}] OpenCV Error: {e}. Mencoba membuka ulang.")
                if cap and cap.isOpened():
                    cap.release()
                    cap = None
                eventlet.sleep(5)
            except Exception as e:
                print(f"[{cam_id_log}] Error tidak terduga: {e}")
                traceback.print_exc()
                if cap and cap.isOpened():
                    cap.release()
                    cap = None
                eventlet.sleep(5)

        if cap and cap.isOpened():
            cap.release()
        print(f"[{cam_id_log}] Loop dihentikan.")

    def trigger_manual_capture(self, capture_type: str):
        """
        Memicu pengambilan gambar manual (Surface/Underwater)
        dengan overlay telemetri.
        Dipanggil oleh endpoint API.
        """
        print(f"[Capture] Menerima trigger manual untuk: {capture_type}")
        
        frame_to_use = None
        mission_name = None
        filename_prefix = None
        image_count = 0

        # 1. Dapatkan state telemetri saat ini
        try:
            with self.asv_handler.state_lock:
                current_state = self.asv_handler.current_state.copy()
        except Exception as e:
            print(f"[Capture] Gagal mendapatkan state ASV: {e}")
            return {"status": "error", "message": "Gagal mendapatkan state ASV."}

        # 2. Dapatkan frame kamera yang relevan
        if capture_type == 'surface':
            # Ambil frame CAM 1 (Surface)
            with VisionService._frame_lock_cam1:
                if VisionService._latest_processed_frame_cam1 is not None:
                    # Kita gunakan frame yang sudah diproses AI (dengan anotasi)
                    frame_to_use = VisionService._latest_processed_frame_cam1.copy()
                else:
                    print("[Capture] Gagal: Frame CAM 1 (Surface) tidak tersedia.")
                    return {"status": "error", "message": "Frame CAM 1 tidak tersedia."}
            
            mission_name = "Surface Imaging (Manual)"
            filename_prefix = "surface"
            image_count = self.surface_image_count
            self.surface_image_count += 1

        elif capture_type == 'underwater':
            # Ambil frame CAM 2 (Underwater)
            with VisionService._frame_lock_cam2:
                if VisionService._latest_raw_frame_cam2 is not None:
                    # Kita gunakan frame mentah CAM 2
                    frame_to_use = VisionService._latest_raw_frame_cam2.copy()
                else:
                    print("[Capture] Gagal: Frame CAM 2 (Underwater) tidak tersedia.")
                    return {"status": "error", "message": "Frame CAM 2 tidak tersedia."}

            mission_name = "Underwater Imaging (Manual)"
            filename_prefix = "underwater"
            image_count = self.underwater_image_count
            self.underwater_image_count += 1
        
        else:
            return {"status": "error", "message": "Tipe capture tidak valid."}


        # 3. Buat Overlay (seperti permintaan "ada tulisan nya e")
        try:
            overlay_data = create_overlay_from_html(
                current_state, mission_type=mission_name
            )
            snapshot = apply_overlay(frame_to_use, overlay_data)
        except Exception as e:
            print(f"[Capture] Gagal membuat overlay: {e}")
            snapshot = frame_to_use  # Fallback ke frame tanpa overlay

        # 4. Simpan file ke folder logs/captures/
        filename = f"{filename_prefix}_{image_count}.jpg"
        save_dir = os.path.join(os.getcwd(), 'logs', 'captures')
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, filename)

        try:
            # Gunakan kualitas 90 agar tulisan jelas
            ret, buffer = cv2.imencode(".jpg", snapshot, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ret:
                with open(save_path, 'wb') as f:
                    f.write(buffer)
                print(f"[Capture] Foto manual BERHASIL disimpan: {save_path}")
                return {"status": "success", "file": filename}
            else:
                raise Exception("cv2.imencode gagal")
                
        except Exception as e:
            print(f"[Capture] Gagal menyimpan file {filename}: {e}")
            # Rollback counter jika gagal
            if filename_prefix == "surface": self.surface_image_count -= 1
            else: self.underwater_image_count -= 1
            return {"status": "error", "message": f"Gagal menyimpan file: {e}"}

    # --- KODE LAMA ANDA UNTUK PROSES AI, NAVIGASI, FOTOGRAFI (DIPERTAHANKAN) ---
    def process_and_control(self, frame, is_mode_auto):
        """Processes frame with AI and triggers control logic if AUTO mode is active."""
        
        if not is_mode_auto:
            logging.info("[Vision] Skipping AI processing - Not in AUTO mode")
            return frame
            
        if frame is None:
            logging.error("[Vision] Cannot process None frame")
            return frame
            
        detections = []
        try:
            logging.info("[Vision] Starting inference on frame...")
            infer_result = eventlet.tpool.execute(
                self.inference_engine.infer, frame
            )
            
            # Handle different return formats from inference engine
            if isinstance(infer_result, tuple) and len(infer_result) == 2:
                detections, scores = infer_result
            else:
                detections = infer_result
                scores = None
                
            if not detections:
                detections = []
            
            logging.info(f"[Vision] Raw detections: {len(detections)} objects found")
            
            # More detailed logging for debugging
            if len(detections) > 0:
                for i, det in enumerate(detections):
                    # Resolve confidence robustly: prefer scores array, fall back to per-detection field
                    conf = 0.0
                    try:
                        if scores is not None:
                            raw = scores[i]
                            # numpy arrays or length-1 lists -> extract scalar
                            try:
                                import numpy as _np
                                if isinstance(raw, _np.ndarray):
                                    conf = float(_np.squeeze(raw))
                                else:
                                    # handle python lists/tuples
                                    if hasattr(raw, '__len__') and len(raw) == 1:
                                        conf = float(raw[0])
                                    else:
                                        conf = float(raw)
                            except Exception:
                                # last-resort conversion
                                conf = float(raw)
                        else:
                            conf = float(det.get('confidence', 0.0))
                    except Exception:
                        conf = 0.0

                    logging.info(f"[Vision] Detected {det.get('class', 'unknown')} with confidence {conf:.2f}")
                    logging.debug(f"[Vision] Detection {i+1} details: {det}")
            else:
                logging.debug("[Vision] No objects detected in this frame")
                    
        except Exception as e:
            logging.error(f"[Vision] Inference error: {str(e)}")
            logging.error(traceback.format_exc())
            # Keep running even with inference errors, just skip this frame
            return frame
        # --- [AKHIR FIX] ---

        # Sisa kode ini sekarang berjalan di greenlet utama (thread-safe)
        validated_detections = []
        green_boxes_detected = []  # List untuk menyimpan deteksi kotak hijau
        blue_boxes_detected = []  # List untuk menyimpan deteksi kotak biru
        
        # --- [DEBUG] TAMBAHKAN Logika Pengecekan ---
        if not detections:
             logging.warning("[Vision DEBUG] Daftar deteksi mentah KOSONG. Melompati validasi.")
        # --- [AKHIR DEBUG] ---
        
        for det in detections:
            original_class = det.get("class")
            if "buoy" in original_class:
                det["class"] = self._validate_buoy_color(frame, det)
                pixel_width = (
                    det.get("xyxy", [0, 0, 0, 0])[2] - det.get("xyxy", [0, 0, 0, 0])[0]
                )
                det["distance_cm"] = self._estimate_distance(
                    pixel_width, det.get("class")
                )
            validated_detections.append(det)

        annotated_frame = self.inference_engine._annotate_frame(
            frame.copy(), validated_detections
        )
        annotated_frame = self._draw_distance_info(
            annotated_frame, validated_detections
        )

        # --- [DEBUG] Log ini sekarang seharusnya tidak muncul jika mode AUTO ---
        if not is_mode_auto:
            logging.info("[Vision DEBUG] process_and_control: is_mode_auto = FALSE. AI dinonaktifkan.")
        # --- [AKHIR DEBUG] ---
            
        if is_mode_auto:
            with self.asv_handler.state_lock:
                current_state_nav = self.asv_handler.current_state.copy()
            # Panggilan ini SEKARANG aman
            self.handle_autonomous_navigation(
                validated_detections, frame.shape[1], current_state_nav
            )

        # Misi fotografi (jika ada)
        green_boxes_detected = [
            d for d in validated_detections if d.get("class") == "green_box"
        ]
        blue_boxes_detected = [
            d for d in validated_detections if d.get("class") == "blue_box"
        ]

        # Cek apakah ada kotak hijau atau biru terdeteksi
        if green_boxes_detected or blue_boxes_detected:
            # Ambil state ASV terbaru
            with self.asv_handler.state_lock:
                current_state_photo = self.asv_handler.current_state.copy()
            # Panggil fungsi fotografi dengan frame DARI KAMERA INI (CAM 1, sebelum anotasi)
            # Panggilan ini SEKARANG aman
            self.handle_photography_mission(
                frame, green_boxes_detected, blue_boxes_detected, current_state_photo
            )

        return annotated_frame

    def handle_autonomous_navigation(self, detections, frame_width, current_state):
        """Handles autonomous navigation based on object detections."""
        
        if not detections:
            logging.info("[Vision] No objects detected for navigation")
            return
            
        detected_classes = [d.get("class", "unknown") for d in detections]
        logging.info(f"[Vision] Processing navigation for objects: {detected_classes}")
        
        # Log control mode and ASV state
        logging.info(f"[Vision] Current control mode: {current_state.get('control_mode')}")
        if current_state.get('control_mode') != 'AUTO':
            logging.warning("[Vision] Not in AUTO mode, skipping navigation")
            return

        # Filter and validate buoy detections
        red_buoys = []
        green_buoys = []
        gate_distance = None
        
        for det in detections:
            cls = det.get("class", "")
            conf = det.get("confidence", 0.0)
            dist = det.get("distance_cm", float("inf"))
            
            if conf < self.poi_confidence_threshold:
                logging.debug(f"[Vision] Skipping low confidence detection: {cls} ({conf:.2f})")
                continue
                
            if cls == "red_buoy":
                red_buoys.append(det)
                logging.info(f"[Vision] Red buoy detected: conf={conf:.2f}, dist={dist:.1f}cm")
            elif cls == "green_buoy":
                green_buoys.append(det)
                logging.info(f"[Vision] Green buoy detected: conf={conf:.2f}, dist={dist:.1f}cm")

        if red_buoys and green_buoys:
            red = min(red_buoys, key=lambda d: d.get("distance_cm", float("inf")))
            try:
                green = min(green_buoys, key=lambda d: d.get("distance_cm", float("inf")))
                gate_pixel_width = max(red["xyxy"][2], green["xyxy"][2]) - min(
                    red["xyxy"][0], green["xyxy"][0]
                )
                
                # Validate detection quality
                if gate_pixel_width <= 0:
                    logging.warning("[Vision] Invalid gate width detected")
                    return
                    
                gate_distance = self._estimate_distance(gate_pixel_width, "buoy_gate")
                if gate_distance is None:
                    logging.warning("[Vision] Could not estimate gate distance")
                    return
                    
                logging.info(f"[Vision] Gate distance: {gate_distance:.1f}cm (activation: {self.gate_activation_distance}cm)")
                
                if gate_distance < self.gate_activation_distance:
                    self.last_buoy_seen_time = time.time()
                    gate_center_x = (red["center"][0] + green["center"][0]) / 2.0
                    
                    # Build and validate payload
                    payload = {
                        "active": True,
                        "gate_center_x": gate_center_x,
                        "red_buoy_x": red["center"][0],
                        "green_buoy_x": green["center"][0],
                        "frame_width": frame_width,
                        "gate_distance": gate_distance,  # Added for debugging
                        "timestamp": time.time(),
                    }
                    
                    logging.info(f"[Vision] Gate conditions met - Sending command")
                    logging.debug(f"[Vision] Gate payload: {payload}")
                    
                    self.asv_handler.process_command(
                        "GATE_TRAVERSAL_COMMAND",
                        payload
                    )
                    return
                    
            except Exception as e:
                logging.error(f"[Vision] Error processing gate detection: {str(e)}")
                logging.error(traceback.format_exc())
                return

        all_buoys = red_buoys + green_buoys
        if all_buoys:
            closest_buoy = min(
                all_buoys, key=lambda b: b.get("distance_cm", float("inf"))
            )
            distance_to_closest = closest_buoy.get("distance_cm", float("inf"))
            if distance_to_closest < self.obstacle_activation_distance:
                self.last_buoy_seen_time = time.time()
                
                # --- [DEBUG] DITAMBAHKAN ---
                payload_obs = {
                    "active": True,
                    "obstacle_class": closest_buoy.get("class", "unknown"),
                    "object_center_x": closest_buoy["center"][0],
                    "frame_width": frame_width,
                }
                logging.critical(f"[Vision DEBUG] KONDISI OBSTACLE TERPENUHI. HENDAK MEMANGGIL process_command. Payload: {payload_obs}")
                # --- [AKHIR DEBUG] ---
                
                self.asv_handler.process_command(
                    "VISION_TARGET_UPDATE",
                    payload_obs, # <-- [DEBUG] Menggunakan var payload
                )
                return

        # --- [DEBUG] DITAMBAHKAN ---
        # Log ini akan tereksekusi jika 'return' di atas tidak terpanggil
        logging.info(f"[Vision DEBUG] Nav handler selesai. Tidak ada perintah AI dikirim. Jarak Gerbang: {gate_distance} | Cooldown: {(time.time() - self.last_buoy_seen_time)}")
        # --- [AKHIR DEBUG] ---

        if (time.time() - self.last_buoy_seen_time) > self.obstacle_cooldown_period:
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})
            self.asv_handler.process_command(
                "GATE_TRAVERSAL_COMMAND", {"active": False}
            )

    # Di dalam class VisionService:
    def handle_photography_mission(
        self, frame_cam1, green_boxes, blue_boxes, current_state
    ):
        """Mengambil snapshot dengan overlay, menggunakan frame kamera yang sesuai."""

        frame_to_use = None
        mission_name = None
        filename_prefix = None
        image_count = 0

        if green_boxes:  # Jika terdeteksi kotak hijau (surface)
            frame_to_use = frame_cam1  # Gunakan frame dari kamera 1 (yang diproses AI)
            mission_name = "Surface Imaging"
            filename_prefix = "surface"
            image_count = self.surface_image_count
            self.surface_image_count += 1
            # print(f"? [CAPTURE] Deteksi Green Box. Mengambil gambar {filename_prefix}_{image_count} dari CAM 1.")

        elif blue_boxes:  # Jika terdeteksi kotak biru (underwater)
            # --- MODIFIKASI: Ambil frame CAM 2 ---
            # Ambil frame terbaru dari kamera 2 (mentah) menggunakan lock yang sesuai
            with VisionService._frame_lock_cam2:
                if VisionService._latest_raw_frame_cam2 is not None:
                    frame_to_use = VisionService._latest_raw_frame_cam2.copy()
                else:
                    # print("?? [CAPTURE] Deteksi Blue Box, tapi frame CAM 2 tidak tersedia.")
                    return  # Jangan lakukan apa-apa jika frame cam 2 tidak ada
            # --- AKHIR MODIFIKASI ---
            mission_name = "Underwater Imaging"
            filename_prefix = "underwater"
            image_count = self.underwater_image_count
            self.underwater_image_count += 1
            # print(f"? [CAPTURE] Deteksi Blue Box. Mengambil gambar {filename_prefix}_{image_count} dari CAM 2.")

        # Jika tidak ada frame yang dipilih
        if frame_to_use is None or mission_name is None:
            return

        # Buat overlay dan snapshot
        try:
            overlay_data = create_overlay_from_html(
                current_state, mission_type=mission_name
            )
            snapshot = apply_overlay(frame_to_use, overlay_data)  # Gunakan frame_to_use
        except Exception as e:
            print(f"[Photography] Gagal membuat overlay: {e}")
            snapshot = frame_to_use  # Fallback ke frame tanpa overlay

        # Encode dan unggah
        filename = f"{filename_prefix}_{image_count}.jpg"
        ret, buffer = cv2.imencode(
            ".jpg", snapshot, [cv2.IMWRITE_JPEG_QUALITY, 30]
        )  # Kualitas JPEG 90
        if ret and buffer is not None:
            # Jalankan upload di background task
            # --- MODIFIKASI: Panggilan Supabase di-comment ---
            # self.socketio.start_background_task(upload_image_to_supabase, buffer, filename, self.config)
            # --- AKHIR MODIFIKASI ---
            pass
        else:
            print(f"[Photography] Gagal encode gambar {filename}")
            # Reset counter jika gagal encode?
            if filename_prefix == "surface":
                self.surface_image_count -= 1
            else:
                self.underwater_image_count -= 1

    def validate_and_trigger_investigation(self, poi_data, frame, current_state):
        # Implementasi validasi POI Anda (dipertahankan)
        self.recent_detections.append(poi_data["class"])
        if len(self.recent_detections) < self.poi_validation_frames:
            return

        if len(set(self.recent_detections)) == 1:
            cls_name = self.recent_detections[0]
            print(f"[Vision] Validasi POI '{cls_name}' berhasil.")
            # ... (Logika trigger investigasi) ...
            self.recent_detections.clear()

    # --- Slot dari GUI (HAPUS DECORATOR @Slot) ---
    def set_gui_listening(self, status: bool):  # <-- Ubah argumen menjadi status: bool
        # Hapus baris 'payload.get()'
        with self.settings_lock:
            if self.gui_is_listening != status:
                print(f"[VisionService] GUI Listening: {status}")
                self.gui_is_listening = status

    # --- [FIX] HAPUS FUNGSI INI KARENA SUDAH TIDAK RELEVAN ---
    # def set_mode(self, payload: dict):  # Hapus @Slot
    #     mode = payload.get("mode", "MANUAL")
    #     with self.settings_lock:
    #         new_mode_auto = mode == "AUTO"
    #         if self.mode_auto != new_mode_auto:
    #             print(f"[VisionService] Mode set to: {mode}")
    #             self.mode_auto = new_mode_auto
    # ---------------------------------------------------------

    def set_inversion(self, payload: dict):  # Hapus @Slot
        is_inverted = payload.get("inverted", False)
        with self.settings_lock:
            if self.is_inverted != is_inverted:
                print(f"[VisionService] Inversion set to: {is_inverted}")
                self.is_inverted = is_inverted