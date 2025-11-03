# src/navantara_backend/services/vision_service.py
# --- VERSI FINAL DENGAN PEMUAT MODEL BARU (YOLOv5 LANGSUNG) ---
import cv2
import numpy as np
import collections
import time
import traceback
import threading
import eventlet
import os
import eventlet.tpool  # <-- Diimpor
import logging
import torch
import sys
from pathlib import Path
# --- [OPTIMASI 3] ---
from dataclasses import asdict
# --- [AKHIR OPTIMASI 3] ---

# --- HAPUS IMPORT LAMA ---
# from navantara_backend.vision.inference_engine import InferenceEngine

# --- HAPUS IMPORT CLOUD UTILS ---
# ... (impor cloud utils dihapus) ...

from navantara_backend.vision.overlay_utils import (
    create_overlay_from_html,
    apply_overlay,
)


class VisionService:
    # --- Variabel Class untuk menyimpan frame terbaru & locks ---
    _latest_processed_frame_cam1 = None
    _frame_lock_cam1 = threading.Lock()
    _latest_raw_frame_cam2 = None
    _frame_lock_cam2 = threading.Lock()
    _latest_processed_frame = _latest_processed_frame_cam1
    _frame_lock = _frame_lock_cam1
    # -----------------------------------------------------------

    def __init__(self, config, asv_handler, socketio):
        self.config = config
        self.asv_handler = asv_handler
        self.socketio = socketio
        self.running = False

        # --- [PERUBAHAN BESAR] Panggil pemuat model baru ---
        self.model = self._load_yolo_model(config)
        # --- [AKHIR PERUBAHAN] ---

        self.settings_lock = threading.Lock()

        # Pengaturan awal
        self.gui_is_listening = False
        self.is_inverted = False  # State ini sudah tidak dipakai di sini

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
        )
        self.show_local_feed = False

        self.surface_image_count = 0
        self.underwater_image_count = 0

        # --- [MODIFIKASI 2.1: Atribut Cooldown Misi Foto] ---
        self.photo_mission_cooldown_sec = 2.0
        self.last_auto_photo_time = 0
        # --- [AKHIR MODIFIKASI 2.1] ---

        # Pengaturan deteksi kamera
        cam_detect_cfg = self.config.get("camera_detection", {})
        self.FOCAL_LENGTH_PIXELS = cam_detect_cfg.get("focal_length_pixels", 600)
        self.OBJECT_REAL_WIDTHS_CM = cam_detect_cfg.get("object_real_widths_cm", {})

        print("[VisionService] Layanan Visi diinisialisasi.")

    # --- [FUNGSI BARU] Pemuat Model ---
    def _load_yolo_model(self, config):
        """Memuat model YOLOv5 (TensorRT atau PyTorch) secara langsung."""
        try:
            # Tentukan path ke folder yolov5 BARU
            yolo_dir = Path(__file__).parent.parent / "yolov5"

            # Tentukan path HANYA ke file .pt
            pt_path = yolo_dir / "besto.pt"

            # Tambahkan path yolov5 ke sys.path agar torch.hub bisa bekerja
            if str(yolo_dir) not in sys.path:
                sys.path.insert(0, str(yolo_dir))

            device = "cuda" if torch.cuda.is_available() else "cpu"

            # Kita HARUS memeriksa file .pt terlebih dahulu.
            if not pt_path.exists():
                print(
                    f"[VisionService] KRITIS: File 'besto.pt' tidak ditemukan di {yolo_dir}"
                )
                print(
                    f"[VisionService] Pastikan Anda sudah menyalin 'besto.pt' ke dalam folder 'yolov5' yang baru."
                )
                return None

            print(f"[VisionService] Memuat model dari {pt_path}.")
            print(
                f"[VisionService] (Akan otomatis mencari 'besto.engine' untuk optimasi)"
            )

            # Muat model menggunakan torch.hub.
            # KITA HANYA MEMBERIKAN .pt.
            # Logika internal yolov5 akan menemukan dan menggunakan .engine jika ada.
            model = torch.hub.load(
                str(yolo_dir),
                "custom",
                path=str(pt_path),  # <-- SELALU ARAHKAN KE .pt
                source="local",
                force_reload=True,
                trust_repo=True,
            )

            model.to(device)

            # Ambil conf/iou dari config
            vision_cfg = config.get("vision", {})
            model.conf = float(vision_cfg.get("conf_threshold", 0.25))
            model.iou = float(vision_cfg.get("iou_threshold", 0.45))

            print(f"[VisionService] Model YOLOv5 berhasil dimuat di device {device}.")
            return model

        except Exception as e:
            print(f"[VisionService] KRITIS: Gagal total memuat model YOLOv5: {e}")
            traceback.print_exc()
            return None

    # --- [AKHIR FUNGSI BARU] ---

    # --- (Fungsi _estimate_distance, _draw_distance_info, _validate_buoy_color tidak berubah) ---
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
            lower_red1 = np.array([0, 100, 60])  # Slightly relaxed saturation & value
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 60])  # For the wrap-around hue of red
            upper_red2 = np.array([180, 255, 255])
            lower_green = np.array([35, 70, 35])  # Wider green range
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
            logging.info(
                f"[Vision] Color Analysis - Red: {red_percentage:.1f}%, Green: {green_percentage:.1f}%"
            )

            # More robust color classification with minimum thresholds
            if (
                green_percentage > 15 and green_px > red_px * 1.2
            ):  # Lowered ratio requirement
                result = "green_buoy"
            elif (
                red_percentage > 15 and red_px > green_px * 1.2
            ):  # Lowered ratio requirement
                result = "red_buoy"
            else:
                result = detection.get("class")

            logging.info(f"[Vision] Buoy classified as: {result}")
            return result
        except Exception as e:
            print(f"[ColorValidation] Error: {e}")
            return detection.get("class")

    # --- (Fungsi run_capture_loops, stop, _capture_loop tidak berubah) ---
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

                with self.settings_lock:
                    should_emit_to_gui = self.gui_is_listening

                with self.asv_handler.state_lock:
                    # --- [OPTIMASI 3] ---
                    is_auto = (
                        self.asv_handler.current_state.control_mode == "AUTO"
                    )
                    # --- [AKHIR OPTIMASI 3] ---

                if apply_detection:

                    try:
                        processed_frame_ai = self.process_and_control(frame, is_auto)
                    except Exception as e:
                        print(f"[{cam_id_log}] Error dalam process_and_control: {e}")
                        traceback.print_exc()
                        eventlet.sleep(1)
                        continue

                    with frame_lock:  # _frame_lock_cam1
                        VisionService._latest_processed_frame_cam1 = (
                            processed_frame_ai.copy()
                        )
                        VisionService._latest_processed_frame = (
                            VisionService._latest_processed_frame_cam1
                        )
                    frame_to_emit = processed_frame_ai
                else:
                    with frame_lock:  # _frame_lock_cam2
                        VisionService._latest_raw_frame_cam2 = frame.copy()
                    frame_to_emit = frame

                if should_emit_to_gui:
                    try:
                        gui_frame = cv2.resize(frame_to_emit, (640, 480))
                    except:
                        gui_frame = frame_to_emit

                    ret_encode, buffer = cv2.imencode(
                        ".jpg", gui_frame, [cv2.IMWRITE_JPEG_QUALITY, 60]
                    )

                    if ret_encode:
                        self.socketio.emit(event_name, buffer.tobytes())

                if self.show_local_feed:
                    if apply_detection:
                        try:
                            frame_cam1_resized = cv2.resize(
                                frame_to_emit, (LOCAL_FEED_WIDTH, LOCAL_FEED_HEIGHT)
                            )

                            frame_cam2 = None
                            with VisionService._frame_lock_cam2:
                                if VisionService._latest_raw_frame_cam2 is not None:
                                    frame_cam2 = (
                                        VisionService._latest_raw_frame_cam2.copy()
                                    )

                            if frame_cam2 is not None:
                                frame_cam2_resized = cv2.resize(
                                    frame_cam2, (LOCAL_FEED_WIDTH, LOCAL_FEED_HEIGHT)
                                )
                            else:
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

                            combined_frame = np.hstack(
                                (frame_cam1_resized, frame_cam2_resized)
                            )

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

    # --- (Fungsi trigger_manual_capture tidak berubah) ---
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
                # --- [OPTIMASI 3] ---
                # Ubah objek dataclass menjadi dict untuk overlay
                current_state_dict = asdict(self.asv_handler.current_state)
                # --- [AKHIR OPTIMASI 3] ---
        except Exception as e:
            print(f"[Capture] Gagal mendapatkan state ASV: {e}")
            return {"status": "error", "message": "Gagal mendapatkan state ASV."}

        # 2. Dapatkan frame kamera yang relevan
        if capture_type == "surface":
            with VisionService._frame_lock_cam1:
                if VisionService._latest_processed_frame_cam1 is not None:
                    frame_to_use = VisionService._latest_processed_frame_cam1.copy()
                else:
                    print("[Capture] Gagal: Frame CAM 1 (Surface) tidak tersedia.")
                    return {"status": "error", "message": "Frame CAM 1 tidak tersedia."}

            mission_name = "Surface Imaging (Manual)"
            filename_prefix = "surface"
            image_count = self.surface_image_count
            self.surface_image_count += 1

        elif capture_type == "underwater":
            with VisionService._frame_lock_cam2:
                if VisionService._latest_raw_frame_cam2 is not None:
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

        try:
            # --- [OPTIMASI 3] ---
            # Kirim dict, bukan objek dataclass
            overlay_data = create_overlay_from_html(
                current_state_dict, mission_type=mission_name
            )
            # --- [AKHIR OPTIMASI 3] ---
            snapshot = apply_overlay(frame_to_use, overlay_data)
        except Exception as e:
            print(f"[Capture] Gagal membuat overlay: {e}")
            snapshot = frame_to_use  # Fallback ke frame tanpa overlay

        filename = f"{filename_prefix}_{image_count}.jpg"
        save_dir = os.path.join(os.getcwd(), "logs", "captures")
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, filename)

        try:
            ret, buffer = cv2.imencode(".jpg", snapshot, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ret:
                with open(save_path, "wb") as f:
                    f.write(buffer)
                print(f"[Capture] Foto manual BERHASIL disimpan: {save_path}")
                return {"status": "success", "file": filename}
            else:
                raise Exception("cv2.imencode gagal")

        except Exception as e:
            print(f"[Capture] Gagal menyimpan file {filename}: {e}")
            if filename_prefix == "surface":
                self.surface_image_count -= 1
            else:
                self.underwater_image_count -= 1
            return {"status": "error", "message": f"Gagal menyimpan file: {e}"}

    # --- [FUNGSI DIPERBARUI TOTAL] ---
    def process_and_control(self, frame, is_mode_auto):
        """Processes frame with AI and triggers control logic if AUTO mode is active."""

        if not is_mode_auto:
            # logging.info("[Vision] Skipping AI processing - Not in AUTO mode")
            return frame

        if self.model is None:
            logging.error("[Vision] Model tidak dimuat, inferensi dilompati.")
            return frame

        if frame is None:
            logging.error("[Vision] Cannot process None frame")
            return frame

        detections = []
        annotated_frame = frame.copy()  # Mulai dengan frame asli

        try:
            logging.info("[Vision] Starting inference on frame...")

            # --- [PERBAIKAI 1: RESIZE] ---
            # Ubah ukuran frame agar sesuai dengan input model (640x640)
            model_input_frame = cv2.resize(frame, (640, 640))

            # 1. Panggil inferensi (dijalankan di tpool oleh eventlet)
            results = eventlet.tpool.execute(self.model, model_input_frame)

            # 2. Konversi hasil pandas ke format dict standar kita
            df = results.pandas().xyxy[0]
            for _, row in df.iterrows():
                # --- [PERBAIKAN 2: KONVERSI KOORDINAT] ---
                # Kita harus mengkonversi koordinat dari 640x640 kembali ke ukuran frame asli (misal 480x640)
                orig_h, orig_w = frame.shape[:2]

                detections.append(
                    {
                        "xyxy": [
                            row["xmin"] * (orig_w / 640.0),  # xmin
                            row["ymin"] * (orig_h / 640.0),  # ymin
                            row["xmax"] * (orig_w / 640.0),  # xmax
                            row["ymax"] * (orig_h / 640.0),  # ymax
                        ],
                        "center": (
                            int((row["xmin"] + row["xmax"]) / 2 * (orig_w / 640.0)),
                            int((row["ymin"] + row["ymax"]) / 2 * (orig_h / 640.0)),
                        ),
                        "class": row["name"],
                        "confidence": row["confidence"],
                    }
                )

            logging.info(f"[Vision] Raw detections: {len(detections)} objects found")

            # 3. Buat frame anotasi (Renderer bawaan yolov5)
            # Kita render hasil di frame yang di-resize, lalu resize kembali
            annotated_resized = results.render()[0]
            annotated_frame = cv2.resize(
                annotated_resized, (frame.shape[1], frame.shape[0])
            )

        except Exception as e:
            logging.error(f"[Vision] Inference error: {str(e)}")
            logging.error(traceback.format_exc())
            return frame  # Kembalikan frame asli jika error
        # --- [AKHIR BLOK PENGGANTI] ---

        # Sisa kode ini sekarang berjalan di greenlet utama (thread-safe)
        validated_detections = []
        green_boxes_detected = []  # List untuk menyimpan deteksi kotak hijau
        blue_boxes_detected = []  # List untuk menyimpan deteksi kotak biru

        if not detections:
            logging.warning(
                "[Vision DEBUG] Daftar deteksi mentah KOSONG. Melompati validasi."
            )

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

        # HAPUS PANGGILAN LAMA KE _annotate_frame

        # Kita masih perlu _draw_distance_info
        annotated_frame = self._draw_distance_info(
            annotated_frame, validated_detections
        )

        if not is_mode_auto:
            logging.info(
                "[Vision DEBUG] process_and_control: is_mode_auto = FALSE. AI dinonaktifkan."
            )

        if is_mode_auto:
            with self.asv_handler.state_lock:
                # --- [OPTIMASI 3] ---
                # .copy() tidak lagi diperlukan, objek dataclass aman untuk dibaca
                # Namun, kita tetap butuh snapshot state, jadi kita gunakan copy()
                # TIDAK, copy() tidak berfungsi di dataclass, kita harus
                # menggunakan asdict() atau copy.deepcopy()
                # Mari kita asumsikan state tidak akan berubah *saat* fungsi ini berjalan
                # Ini adalah asumsi yang cukup aman karena state lock
                current_state_nav = self.asv_handler.current_state
                # --- [AKHIR OPTIMASI 3] ---
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

        # --- [MODIFIKASI 2.2: Logika Misi Foto Otomatis] ---
        if green_boxes_detected or blue_boxes_detected:
            current_time = time.time()
            # Ambil state misi DARI HANDLER (Otak)
            with self.asv_handler.state_lock:
                current_wp = self.asv_handler.current_state.nav_target_wp_index
                mission_target_wp = self.asv_handler.current_state.photo_mission_target_wp
                qty_taken = self.asv_handler.current_state.photo_mission_qty_taken
                qty_requested = self.asv_handler.current_state.photo_mission_qty_requested

                # Cek kondisi misi
                mission_active = (current_wp == mission_target_wp)
                photos_needed = (qty_taken < qty_requested)
                cooldown_ready = (current_time - self.last_auto_photo_time > self.photo_mission_cooldown_sec)

                if mission_active and photos_needed and cooldown_ready:
                    # KONDISI TERPENUHI: Ambil foto & update counter
                    print(f"[Vision] Misi Foto Otomatis: Mengambil foto {qty_taken + 1} / {qty_requested}")

                    # Ambil snapshot state untuk foto (di dalam lock)
                    current_state_photo = self.asv_handler.current_state 

                    # Panggil fungsi yang ada
                    self.handle_photography_mission(
                        frame, green_boxes_detected, blue_boxes_detected, current_state_photo
                    )

                    # Update counter & cooldown
                    self.asv_handler.current_state.photo_mission_qty_taken += 1
                    self.last_auto_photo_time = current_time
        # --- [AKHIR MODIFIKASI 2.2] ---

        return annotated_frame

    # --- [AKHIR FUNGSI DIPERBARUI] ---

    # --- (Sisa file: handle_autonomous_navigation, handle_photography_mission, dll tidak berubah) ---
    def handle_autonomous_navigation(self, detections, frame_width, current_state):
        """Handles autonomous navigation based on object detections."""

        if not detections:
            logging.info("[Vision] No objects detected for navigation")
            return

        detected_classes = [d.get("class", "unknown") for d in detections]
        logging.info(f"[Vision] Processing navigation for objects: {detected_classes}")

        # --- [OPTIMASI 3] ---
        # Ubah .get('key') menjadi .key
        logging.info(
            f"[Vision] Current control mode: {current_state.control_mode}"
        )
        if current_state.control_mode != "AUTO":
            logging.warning("[Vision] Not in AUTO mode, skipping navigation")
            return
        # --- [AKHIR OPTIMASI 3] ---

        red_buoys = []
        green_buoys = []
        gate_distance = None

        for det in detections:
            cls = det.get("class", "")
            conf = det.get("confidence", 0.0)
            dist = det.get("distance_cm", float("inf"))

            if conf < self.poi_confidence_threshold:
                logging.debug(
                    f"[Vision] Skipping low confidence detection: {cls} ({conf:.2f})"
                )
                continue

            if cls == "red_buoy":
                red_buoys.append(det)
                logging.info(
                    f"[Vision] Red buoy detected: conf={conf:.2f}, dist={dist:.1f}cm"
                )
            elif cls == "green_buoy":
                green_buoys.append(det)
                logging.info(
                    f"[Vision] Green buoy detected: conf={conf:.2f}, dist={dist:.1f}cm"
                )

        if red_buoys and green_buoys:
            red = min(red_buoys, key=lambda d: d.get("distance_cm", float("inf")))
            try:
                green = min(
                    green_buoys, key=lambda d: d.get("distance_cm", float("inf"))
                )
                gate_pixel_width = max(red["xyxy"][2], green["xyxy"][2]) - min(
                    red["xyxy"][0], green["xyxy"][0]
                )

                if gate_pixel_width <= 0:
                    logging.warning("[Vision] Invalid gate width detected")
                    return

                gate_distance = self._estimate_distance(gate_pixel_width, "buoy_gate")
                if gate_distance is None:
                    logging.warning("[Vision] Could not estimate gate distance")
                    return

                logging.info(
                    f"[Vision] Gate distance: {gate_distance:.1f}cm (activation: {self.gate_activation_distance}cm)"
                )

                if gate_distance < self.gate_activation_distance:
                    self.last_buoy_seen_time = time.time()
                    gate_center_x = (red["center"][0] + green["center"][0]) / 2.0

                    payload = {
                        "active": True,
                        "gate_center_x": gate_center_x,
                        "red_buoy_x": red["center"][0],
                        "green_buoy_x": green["center"][0],
                        "frame_width": frame_width,
                        "gate_distance": gate_distance,
                        "timestamp": time.time(),
                    }

                    logging.info(f"[Vision] Gate conditions met - Sending command")
                    logging.debug(f"[Vision] Gate payload: {payload}")

                    self.asv_handler.process_command("GATE_TRAVERSAL_COMMAND", payload)
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

                payload_obs = {
                    "active": True,
                    "obstacle_class": closest_buoy.get("class", "unknown"),
                    "object_center_x": closest_buoy["center"][0],
                    "frame_width": frame_width,
                }
                logging.critical(
                    f"[Vision DEBUG] KONDISI OBSTACLE TERPENUHI. HENDAK MEMANGGIL process_command. Payload: {payload_obs}"
                )

                self.asv_handler.process_command(
                    "VISION_TARGET_UPDATE",
                    payload_obs,
                )
                return

        logging.info(
            f"[Vision DEBUG] Nav handler selesai. Tidak ada perintah AI dikirim. Jarak Gerbang: {gate_distance} | Cooldown: {(time.time() - self.last_buoy_seen_time)}"
        )

        if (time.time() - self.last_buoy_seen_time) > self.obstacle_cooldown_period:
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})
            self.asv_handler.process_command(
                "GATE_TRAVERSAL_COMMAND", {"active": False}
            )

    def handle_photography_mission(
        self, frame_cam1, green_boxes, blue_boxes, current_state
    ):
        """Mengambil snapshot dengan overlay, menggunakan frame kamera yang sesuai."""

        frame_to_use = None
        mission_name = None
        filename_prefix = None
        image_count = 0

        if green_boxes:
            frame_to_use = frame_cam1
            mission_name = "Surface Imaging"
            filename_prefix = "surface"
            image_count = self.surface_image_count
            self.surface_image_count += 1
        elif blue_boxes:
            with VisionService._frame_lock_cam2:
                if VisionService._latest_raw_frame_cam2 is not None:
                    frame_to_use = VisionService._latest_raw_frame_cam2.copy()
                else:
                    return
            mission_name = "Underwater Imaging"
            filename_prefix = "underwater"
            image_count = self.underwater_image_count
            self.underwater_image_count += 1

        if frame_to_use is None or mission_name is None:
            return

        try:
            # --- [OPTIMASI 3] ---
            # Kirim dict ke fungsi eksternal, bukan objek dataclass
            overlay_data = create_overlay_from_html(
                asdict(current_state), mission_type=mission_name
            )
            # --- [AKHIR OPTIMASI 3] ---
            snapshot = apply_overlay(frame_to_use, overlay_data)
        except Exception as e:
            print(f"[Photography] Gagal membuat overlay: {e}")
            snapshot = frame_to_use

        filename = f"{filename_prefix}_{image_count}.jpg"
        
        # --- [MODIFIKASI: Tambahkan path penyimpanan lokal] ---
        save_dir = os.path.join(os.getcwd(), "logs", "captures")
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, filename)
        # --- [AKHIR MODIFIKASI] ---

        # (Kualitas disamakan dengan manual capture, yaitu 90)
        ret, buffer = cv2.imencode(".jpg", snapshot, [cv2.IMWRITE_JPEG_QUALITY, 90])
        
        if ret and buffer is not None:
            # --- [MODIFIKASI: Ganti 'pass' dengan logika penyimpanan file] ---
            try:
                with open(save_path, "wb") as f:
                    f.write(buffer)
                print(f"[Photography] Foto Misi Otomatis BERHASIL disimpan: {save_path}")
            except Exception as e:
                print(f"[Photography] Gagal menyimpan file {filename}: {e}")
                # Rollback counter jika gagal simpan
                if filename_prefix == "surface":
                    self.surface_image_count -= 1
                else:
                    self.underwater_image_count -= 1
            # --- [AKHIR MODIFIKASI] ---
        else:
            print(f"[Photography] Gagal encode gambar {filename}")
            if filename_prefix == "surface":
                self.surface_image_count -= 1
            else:
                self.underwater_image_count -= 1

    def validate_and_trigger_investigation(self, poi_data, frame, current_state):
        self.recent_detections.append(poi_data["class"])
        if len(self.recent_detections) < self.poi_validation_frames:
            return

        if len(set(self.recent_detections)) == 1:
            cls_name = self.recent_detections[0]
            print(f"[Vision] Validasi POI '{cls_name}' berhasil.")
            self.recent_detections.clear()

    def set_gui_listening(self, status: bool):
        with self.settings_lock:
            if self.gui_is_listening != status:
                print(f"[VisionService] GUI Listening: {status}")
                self.gui_is_listening = status

    def set_inversion(self, payload: dict):
        is_inverted = payload.get("inverted", False)
        with self.settings_lock:
            if self.is_inverted != is_inverted:
                print(f"[VisionService] Inversion set to: {is_inverted}")
                self.is_inverted = is_inverted