# src/navantara_backend/services/vision_service.py
import cv2
import numpy as np
import collections
import time
import traceback
import threading
import eventlet
import os
import eventlet.tpool
import logging
import base64
from pathlib import Path
from dataclasses import asdict

# --- [MIGRASI: Import Ultralytics] ---
from ultralytics import YOLO

# -------------------------------------

from navantara_backend.vision.overlay_utils import (
    create_overlay_from_html,
    apply_overlay,
)




class ThreadedCamera:
    def __init__(self, src=0):
        self.src = src
        # [MODIFIKASI] Log source untuk debugging
        print(f"[ThreadedCamera] Membuka source: {self.src}")

        self.capture = cv2.VideoCapture(src)
        # Set buffer size (opsional, tergantung driver)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.status = False
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()

        # Cek apakah kamera benar-benar terbuka saat init
        if self.capture.isOpened():
            self.status, self.frame = self.capture.read()
        else:
            self.status = False

        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        while not self.stopped:
            try:
                if not self.capture.isOpened():
                    # Jika capture tertutup, set status False dan tunggu
                    with self.lock:
                        self.status = False
                    time.sleep(0.5)
                    continue

                status, frame = self.capture.read()

                with self.lock:
                    if status and frame is not None and frame.size > 0:
                        self.status = True
                        self.frame = frame
                    else:
                        self.status = False

                # Sleep sangat kecil untuk yield CPU
                time.sleep(0.005)

            except Exception as e:
                print(f"[ThreadedCamera] Error in loop: {e}")
                with self.lock:
                    self.status = False
                # Jangan break loop, coba recover di iterasi berikutnya atau biarkan caller mereset
                time.sleep(0.1)

    def read(self):
        with self.lock:
            # Kembalikan None eksplisit jika status False
            if not self.status:
                return False, None
            return True, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

        if self.capture.isOpened():
            self.capture.release()



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

        # --- [CRITICAL: LABEL MAPPING] ---
        # Menjembatani perbedaan label Model Baru vs Logika asv_handler
        self.LABEL_MAP = {
            "Blue_Ball": "bola-biru",
            "Blue_Box": "kotak-biru",
            "Green_Ball": "bola-hijau",
            "Green_Box": "kotak-hijau",
            "Red_Ball": "bola-merah",
        }
        # ---------------------------------

        # Ambil threshold dari config
        vision_cfg = self.config.get("vision", {})
        self.conf_thres = float(vision_cfg.get("conf_threshold", 0.25))
        self.iou_thres = float(vision_cfg.get("iou_threshold", 0.45))

        # --- [MIGRASI: Panggil pemuat model Ultralytics] ---
        self.model = self._load_yolo_model(config)
        # ---------------------------------------------------

        self.settings_lock = threading.Lock()

        # Pengaturan awal
        self.gui_is_listening = False

        # Pengaturan dari config.json
        # [MODIFIKASI] Dukungan string path (udev) dengan fallback ke index lama
        self.camera_src_1 = vision_cfg.get("camera_src_1")
        if self.camera_src_1 is None:
            self.camera_src_1 = int(vision_cfg.get("camera_index_1", 0))

        self.camera_src_2 = vision_cfg.get("camera_src_2")
        if self.camera_src_2 is None:
            self.camera_src_2 = int(vision_cfg.get("camera_index_2", 1))

        print(f"[VisionService] Camera 1 Source: {self.camera_src_1}")
        print(f"[VisionService] Camera 2 Source: {self.camera_src_2}")
        self.KNOWN_CLASSES = vision_cfg.get("known_classes", [])
        self.poi_confidence_threshold = vision_cfg.get("poi_confidence_threshold", 0.65)
        self.poi_validation_frames = vision_cfg.get("poi_validation_frames", 5)
        self.gate_activation_distance = vision_cfg.get(
            "gate_activation_distance_cm", 0.0
        )
        self.obstacle_activation_distance = vision_cfg.get(
            "obstacle_activation_distance_cm", 160.0
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

        # Atribut Cooldown Misi Foto
        self.photo_mission_cooldown_sec = 2.0
        self.last_auto_photo_time_surface = 0
        self.last_auto_photo_time_underwater = 0

        # Pengaturan deteksi kamera
        cam_detect_cfg = self.config.get(
            "camera_detection", {"bola-merah": 20.0, "bola-hijau": 20.0, "bola-biru": 20.0}
        )
        self.FOCAL_LENGTH_PIXELS = cam_detect_cfg.get("focal_length_pixels", 600)
        self.OBJECT_REAL_WIDTHS_CM = cam_detect_cfg.get("object_real_widths_cm", {})

        print("[VisionService] Layanan Visi (YOLOv11 + TensorRT Ready) diinisialisasi.")

    # [FIX GRN-06] Dipindahkan dari module-level ke dalam class sebagai @staticmethod
    @staticmethod
    def _refine_blue_class(roi_frame, current_cls_id):
        """Mengkoreksi klasifikasi objek biru (bola vs kotak) menggunakan analisis bentuk HSV."""
        # Proteksi: Jika box terlalu kecil, percayai saja YOLO
        if roi_frame.shape[0] < 15 or roi_frame.shape[1] < 15:
            return current_cls_id

        # 1. Konversi ke HSV
        hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)

        # 2. HSV Masking Khusus Warna Biru
        lower_blue = np.array([90, 50, 40])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 3. Morfologi Ringan (Optimasi untuk Jetson)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 4. Cari Kontur
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return current_cls_id

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area < 50:
            return current_cls_id

        # 5. EVALUASI BENTUK (SHAPE ANALYSIS)
        perimeter = cv2.arcLength(largest_contour, True)
        epsilon = 0.04 * perimeter
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)
        vertices = len(approx)

        rect = cv2.minAreaRect(largest_contour)
        box_area = rect[1][0] * rect[1][1]
        extent = area / box_area if box_area > 0 else 0

        if vertices <= 6 and extent > 0.82:
            return 1  # 1 = 'Blue_Box'
        else:
            return 0  # 0 = 'Blue_Ball'

    # --- [REFACTOR: SMART MODEL LOADER - BERSIH] ---
    def _load_yolo_model(self, config):
        """
        Memuat model dengan urutan prioritas:
        1. best.engine (TensorRT - Tercepat)
        2. best.pt (PyTorch Standard)
        """
        try:
            # Path relatif ke folder src/navantara_backend/vision/
            vision_dir = Path(__file__).parent.parent / "vision"

            engine_path = vision_dir / "best.engine"
            pt_path = vision_dir / "best.pt"

            model_path = None
            task_msg = ""

            # 1. Cek TensorRT (.engine) - PRIORITAS UTAMA
            if engine_path.exists():
                print(f"[VisionService] MENEMUKAN MODEL TENSORRT: {engine_path}")
                print(
                    "[VisionService] Menggunakan akselerasi hardware Jetson (CUDA/TensorRT)."
                )
                model_path = str(engine_path)
                task_msg = "TensorRT Engine"

            # 2. Fallback ke PyTorch (.pt)
            elif pt_path.exists():
                print(
                    f"[VisionService] Warning: .engine tidak ditemukan. Fallback ke .pt: {pt_path}"
                )
                print(
                    "[VisionService] Performa mungkin lebih lambat dibandingkan TensorRT."
                )
                model_path = str(pt_path)
                task_msg = "PyTorch Model"

            else:
                print(
                    f"[VisionService] KRITIS: Tidak ada model 'best.engine' atau 'best.pt' di {vision_dir}"
                )
                return None

            # Muat Model menggunakan Ultralytics
            print(f"[VisionService] Memuat {task_msg}...")
            model = YOLO(model_path, task="detect")

            # Pemanasan awal (Warmup)
            print("[VisionService] Melakukan warmup model...")
            model.predict(
                source=np.zeros((640, 640, 3), dtype=np.uint8), verbose=False, device=0
            )

            print("[VisionService] Model berhasil dimuat dan siap.")
            return model

        except Exception as e:
            print(f"[VisionService] KRITIS: Gagal memuat model YOLOv11: {e}")
            traceback.print_exc()
            return None

    # -----------------------------------------

    def _estimate_distance(self, pixel_width, object_class):
        # Coba ambil ukuran dari file config.json terlebih dahulu
        real_width_cm = self.OBJECT_REAL_WIDTHS_CM.get(object_class)
        
        # --- [TAMBAHAN: FALLBACK UKURAN FISIK (DALAM CM)] ---
        # Jika nama Indonesia tidak ada di config, kita atur ukuran default di sini
        if not real_width_cm:
            if "bola" in object_class:
                real_width_cm = 20.0  # Asumsi diameter bola pelampung adalah 20 cm
            elif "kotak" in object_class:
                real_width_cm = 25.0  # Asumsi lebar kotak pelampung adalah 25 cm
            else:
                real_width_cm = 20.0  # Fallback default
        # ----------------------------------------------------

        # Proteksi pembagian nol
        if not real_width_cm or pixel_width <= 0:
            return None
            
        # Rumus Pinhole Camera: (Lebar Asli * Focal Length) / Lebar di Layar
        return (real_width_cm * self.FOCAL_LENGTH_PIXELS) / pixel_width

    def _draw_distance_info(self, frame, detections):
        all_detections = [d for d in detections if d.get("distance_cm") is not None]
        for det in all_detections:
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
        return frame


    def run_capture_loops(self):
        """Memulai greenlet terpisah untuk menangkap frame dari kedua kamera."""
        if not self.running:
            self.running = True
            print("[VisionService] Memulai loop penangkapan untuk kedua kamera...")

            eventlet.spawn(
                self._capture_loop,
                self.camera_src_1,  # [UBAH] Gunakan src
                VisionService._frame_lock_cam1,
                "CAM1",
                "frame_cam1",
                True,
            )
            eventlet.spawn(
                self._capture_loop,
                self.camera_src_2,  # [UBAH] Gunakan src
                VisionService._frame_lock_cam2,
                "CAM2",
                "frame_cam2",
                False,
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
        self, cam_src, frame_lock, cam_id_log, event_name, apply_detection
    ):
        """
        Loop capture dengan mekanisme SELF-HEALING (Auto Reconnect).
        Mendukung path string (udev rules) atau index integer.
        """
        print(f"[{cam_id_log}] Memulai loop kamera (Source: {cam_src})...")

        cap = None
        consecutive_failures = 0
        MAX_FAILURES = 10  # Batas toleransi kegagalan sebelum hard reset
        RECONNECT_DELAY = 2.0  # Waktu tunggu (detik) sebelum inisialisasi ulang

        # Fungsi helper untuk inisialisasi kamera
        def init_camera(source):
            try:
                # Coba buka sebentar untuk tes
                temp = cv2.VideoCapture(source)
                if not temp.isOpened():
                    print(f"[{cam_id_log}] Gagal membuka device {source} (Not Opened).")
                    return None
                temp.release()

                # Inisialisasi ThreadedCamera dengan source yang sesuai
                new_cap = ThreadedCamera(source)
                print(f"[{cam_id_log}] Kamera berhasil diinisialisasi (Re-init).")
                return new_cap
            except Exception as e:
                print(f"[{cam_id_log}] Exception saat init kamera: {e}")
                return None

        # Inisialisasi awal menggunakan cam_src (bisa string path atau int)
        cap = init_camera(cam_src)

        # Settingan GUI
        GUI_SKIP_RATE = 5
        frame_counter = 0

        while self.running:
            frame_valid = False
            frame_to_process = None

            # --- BLOK PEMBACAAN DENGAN WATCHDOG ---
            if cap:
                ret, frame = cap.read()
                if ret and frame is not None:
                    frame_valid = True
                    consecutive_failures = 0  # Reset counter jika sukses
                    frame_to_process = frame
                else:
                    frame_valid = False
            else:
                frame_valid = False

            # --- LOGIKA RECOVERY (SELF HEALING) ---
            if not frame_valid:
                consecutive_failures += 1
                if consecutive_failures % 10 == 0:
                    print(
                        f"[{cam_id_log}] Warning: Gagal membaca frame ({consecutive_failures}x)."
                    )

                # Jika kegagalan menembus ambang batas, lakukan HARD RESET
                if consecutive_failures > MAX_FAILURES:
                    print(
                        f"[{cam_id_log}] KRITIS: Mencoba HARD RESET koneksi kamera..."
                    )

                    # 1. Matikan instance lama
                    if cap:
                        try:
                            cap.stop()
                        except Exception:
                            pass
                        cap = None

                    # 2. Tunggu Kernel release resource (PENTING)
                    eventlet.sleep(RECONNECT_DELAY)

                    # 3. Coba buat instance baru dengan source yang sama
                    cap = init_camera(cam_src)

                    # Reset counter agar tidak spam reset jika kamera benar-benar mati
                    consecutive_failures = 0

                # Sleep sebentar agar tidak membebani CPU saat error
                eventlet.sleep(0.1)
                continue
            # --------------------------------------

            # === JIKA FRAME VALID, LANJUTKAN PROSES ===

            with self.settings_lock:
                should_emit_to_gui = self.gui_is_listening

            with self.asv_handler.state_lock:
                # Pastikan asv_handler sudah siap
                if hasattr(self.asv_handler, "current_state"):
                    is_auto = self.asv_handler.current_state.control_mode == "AUTO"
                else:
                    is_auto = False

            # 1. PROSES AI (Inference)
            if apply_detection:
                try:
                    processed_frame_ai = self.process_and_control(
                        frame_to_process, is_auto
                    )
                    with frame_lock:
                        VisionService._latest_processed_frame_cam1 = (
                            processed_frame_ai.copy()
                        )
                        VisionService._latest_processed_frame = (
                            VisionService._latest_processed_frame_cam1
                        )
                    frame_to_emit = processed_frame_ai
                except Exception as e:
                    print(f"[{cam_id_log}] Error AI: {e}")
                    traceback.print_exc()
                    eventlet.sleep(0.1)
                    continue
            else:
                # Kamera 2 (Tanpa AI)
                with frame_lock:
                    VisionService._latest_raw_frame_cam2 = frame_to_process.copy()
                frame_to_emit = frame_to_process

            # 2. KIRIM KE GUI (Rate Limited)
            frame_counter += 1
            if should_emit_to_gui and (frame_counter % GUI_SKIP_RATE == 0):
                try:
                    gui_frame = cv2.resize(frame_to_emit, (320, 240))
                    ret_encode, buffer = cv2.imencode(
                        ".jpg", gui_frame, [cv2.IMWRITE_JPEG_QUALITY, 35]
                    )
                    if ret_encode:
                        b64_string = base64.b64encode(buffer).decode("utf-8")
                        self.socketio.emit(event_name, b64_string)
                        eventlet.sleep(0)
                except Exception:
                    pass

            eventlet.sleep(0.001)

        # Cleanup saat stop total
        if cap:
            cap.stop()
        print(f"[{cam_id_log}] Loop berhenti.")

    # [MODIFIKASI] Menambahkan parameter raw_mode=False sebagai default
    def trigger_manual_capture(self, capture_type: str, raw_mode: bool = False):
        """
        Menangani trigger manual capture dengan dukungan mode RAW (Tanpa Overlay) vs Default (Dengan Overlay).
        Pastikan argumen raw_mode diteruskan dengan benar dari handler perintah.
        """
        mode_label = "RAW" if raw_mode else "OVERLAY"
        print(f"[Capture] Menerima trigger: {capture_type} | Mode: {mode_label}")

        frame_source = None
        mission_name = None
        filename_prefix = None
        image_count = 0

        # 1. Mengambil Frame yang Tepat (Thread-Safe)
        if capture_type == "surface":
            with VisionService._frame_lock_cam1:
                # Gunakan frame processed jika ada, tapi ini adalah gambar "bersih" dari output deteksi
                # sebelum digambar kotak-kotak visualisasi (jika arsitekturnya benar).
                # Jika ingin benar-benar RAW dari kamera, gunakan logika pengambilan frame mentah jika tersedia.
                # Untuk saat ini, kita gunakan _latest_processed_frame_cam1.
                if VisionService._latest_processed_frame_cam1 is not None:
                    frame_source = VisionService._latest_processed_frame_cam1.copy()
                else:
                    return {
                        "status": "error",
                        "message": "Kamera Depan tidak siap/kosong.",
                    }

            mission_name = "Surface Imaging"
            filename_prefix = "surface"
            image_count = self.surface_image_count
            self.surface_image_count += 1

        elif capture_type == "underwater":
            with VisionService._frame_lock_cam2:
                if VisionService._latest_raw_frame_cam2 is not None:
                    frame_source = VisionService._latest_raw_frame_cam2.copy()
                else:
                    return {
                        "status": "error",
                        "message": "Kamera Bawah tidak siap/kosong.",
                    }

            mission_name = "Underwater Imaging"
            filename_prefix = "underwater"
            image_count = self.underwater_image_count
            self.underwater_image_count += 1

        else:
            return {
                "status": "error",
                "message": f"Tipe capture '{capture_type}' tidak valid.",
            }

        # 2. Proses Snapshot (Overlay vs RAW)
        snapshot = frame_source

        if raw_mode:
            # --- JALUR RAW ---
            # 2.1. Terapkan penanda nama file _raw
            filename_prefix = f"{filename_prefix}_raw"
            # 2.2. snapshot sudah diset ke frame_source, overlay dilewati.
            print("[Capture] Mode RAW: Overlay dilewati.")
        else:
            # --- JALUR OVERLAY ---
            # 2.1. Coba terapkan overlay telemetri
            try:
                with self.asv_handler.state_lock:
                    current_state_dict = asdict(self.asv_handler.current_state)

                # Buat dan terapkan overlay
                overlay_data = create_overlay_from_html(
                    current_state_dict, mission_type=mission_name
                )
                snapshot = apply_overlay(frame_source, overlay_data)
                print("[Capture] Mode OVERLAY: Telemetri diterapkan.")
            except Exception as e:
                # 2.2. Jika overlay gagal, snapshot tetap frame_source.
                logging.error(
                    f"[Capture] Gagal membuat overlay: {e}. Fallback ke gambar asli."
                )
                # snapshot sudah frame_source, tidak perlu assignment lagi.

        # 3. Penyimpanan File
        filename = f"{filename_prefix}_{image_count}.jpg"
        save_dir = os.path.join(os.getcwd(), "logs", "captures")
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, filename)

        try:
            # Gunakan kualitas tinggi untuk RAW
            quality = 98 if raw_mode else 90
            ret, buffer = cv2.imencode(
                ".jpg", snapshot, [cv2.IMWRITE_JPEG_QUALITY, quality]
            )

            if ret:
                with open(save_path, "wb") as f:
                    f.write(buffer)
                print(f"[Capture] Berhasil disimpan: {filename}")
                return {
                    "status": "success",
                    "file": filename,
                    "mode": mode_label,
                    "path": save_path,
                }
            else:
                return {"status": "error", "message": "Gagal encode gambar."}

        except Exception as e:
            logging.error(f"[Capture] Exception saat menyimpan: {e}")
            return {"status": "error", "message": str(e)}

    def set_obstacle_distance(self, payload):
        """
        Mengubah jarak aktivasi penghindaran rintangan secara real-time.
        Payload: {'distance': float}
        """
        try:
            new_dist = float(payload.get("distance", 160.0))

            # Batasi nilai agar masuk akal (misal 0 - 1000 cm)
            new_dist = max(0.0, min(1000.0, new_dist))

            with self.settings_lock:
                self.obstacle_activation_distance = new_dist

            print(f"[VisionService] Jarak pemicu obstacle diupdate ke: {new_dist} cm")

        except ValueError:
            print(
                f"[VisionService] Error: Nilai jarak tidak valid di payload: {payload}"
            )

    # --- [REFACTOR: ULTRALYTICS INFERENCE + LABEL MAPPING] ---
    def process_and_control(self, frame, is_mode_auto):
        """
        OPTIMIZED: Navigasi diprioritaskan sebelum Visualisasi.
        """
        if not is_mode_auto:
            return frame

        if self.model is None or frame is None:
            return frame

        detections = []
        annotated_frame = frame.copy()
        orig_h, orig_w = frame.shape[:2]
        target_size = 640  # Ukuran input model YOLOv11

        try:
            # 1. INFERENCE (AI Processing)
            frame_resized = cv2.resize(frame, (target_size, target_size))
            scale_x = orig_w / target_size
            scale_y = orig_h / target_size

            results = eventlet.tpool.execute(
                self.model.predict,
                source=frame_resized,
                imgsz=target_size,
                conf=self.conf_thres,
                iou=self.iou_thres,
                verbose=False,
            )
            result = results[0]

            # 2. DATA EXTRACTION (Cepat)
            # Hanya ekstrak koordinat dan class, JANGAN GAMBAR DULU.
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    coords_small = box.xyxy[0].cpu().numpy().tolist()
                    x1 = int(coords_small[0] * scale_x)
                    y1 = int(coords_small[1] * scale_y)
                    x2 = int(coords_small[2] * scale_x)
                    y2 = int(coords_small[3] * scale_y)

                    cls_id = int(box.cls[0].cpu().numpy())
                    conf = float(box.conf[0].cpu().numpy())

                    # --- [PANGGIL FUNGSI KOREKSI DI SINI] ---
                    # Jika YOLO menebak kelas 0 (Blue_Ball) atau 1 (Blue_Box)
                    if cls_id == 0 or cls_id == 1:
                        # Pastikan koordinat crop tidak melenceng keluar dari batas frame asli
                        x1_roi, y1_roi = max(0, x1), max(0, y1)
                        x2_roi, y2_roi = min(orig_w, x2), min(orig_h, y2)
                        
                        # Potong frame menjadi ROI (Region of Interest)
                        if (x2_roi > x1_roi) and (y2_roi > y1_roi):
                            roi = frame[y1_roi:y2_roi, x1_roi:x2_roi]
                            # Timpa cls_id lama dengan hasil koreksi bentuk/HSV
                            cls_id = self._refine_blue_class(roi, cls_id)
                    # ----------------------------------------

                    # Tarik nama kelas SETELAH proses koreksi selesai
                    raw_cls_name = result.names[cls_id]
                    final_cls_name = self.LABEL_MAP.get(raw_cls_name, raw_cls_name)

                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    detections.append(
                        {
                            "xyxy": [x1, y1, x2, y2],
                            "center": (center_x, center_y),
                            "class": final_cls_name,
                            "confidence": conf,
                            "original_class": raw_cls_name,
                        }
                    )
            # =========================================================

            # 3. CONTROL LOGIC (PRIORITAS UTAMA)
            # Eksekusi keputusan navigasi SEKARANG JUGA sebelum CPU sibuk menggambar.
            # Validasi warna buoy juga sebaiknya dilakukan di sini jika mempengaruhi keputusan
            validated_detections_for_nav = []
            for det in detections:
                cls_name = det.get("class", "")
                
                # Hitung jarak jika objek adalah bola atau kotak
                if "bola" in cls_name or "kotak" in cls_name:
                    pixel_width = det["xyxy"][2] - det["xyxy"][0]
                    det["distance_cm"] = self._estimate_distance(pixel_width, cls_name)
                    
                validated_detections_for_nav.append(det)

            if is_mode_auto:
                # [FIX RED-07] Salin hanya value yang dibutuhkan ke variabel lokal,
                # bukan menyimpan referensi ke objek state utuh yang bisa berubah
                # dari thread lain setelah lock dilepas.
                with self.asv_handler.state_lock:
                    nav_control_mode = self.asv_handler.current_state.control_mode

                # [ZERO LATENCY TRIGGER]
                # Kirim perintah ke ESP32
                self.handle_autonomous_navigation(
                    validated_detections_for_nav, orig_w, nav_control_mode
                )

            # 4. VISUALIZATION (PRIORITAS RENDAH - UI ONLY)
            # Lakukan penggambaran setelah perintah kontrol dikirim
            for det in validated_detections_for_nav:
                x1, y1, x2, y2 = det["xyxy"]
                cls_name = det["class"]
                conf = det["confidence"]
                sudut = det.get("jumlah_sudut", 0)
                
                # --- [TAMBAHAN] Ambil Nilai Jarak ---
                jarak = det.get("distance_cm")
                jarak_teks = f"{jarak:.1f}cm" if jarak is not None else "? cm"

                # Penentuan warna Bounding Box
                color = (0, 255, 0) # Default: Hijau
                if "merah" in cls_name:
                    color = (0, 0, 255)
                elif "biru" in cls_name: 
                    color = (255, 0, 0)

                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                
                # --- Teks Bounding Box (Digabung: Nama, Conf, Sudut, Jarak) ---
                label = f"{cls_name} {conf:.2f} (Sdt:{sudut}) | {jarak_teks}"
                cv2.putText(
                    annotated_frame,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2,
                )

        except Exception as e:
            logging.error(f"[Vision] Inference error: {str(e)}")
            logging.error(traceback.format_exc())
            return frame



        # --- [MULAI LOGIKA BARU: MISI FOTO SEGMEN] ---
        # Langsung masuk ke logika segmen tanpa mendefinisikan green_boxes/blue_boxes
        current_time = time.time()

        with self.asv_handler.state_lock:
            # Ambil State Navigasi
            current_wp = self.asv_handler.current_state.nav_target_wp_index

            # Ambil Konfigurasi Segmen & Kuota
            start_wp = self.asv_handler.current_state.photo_mission_target_wp1
            stop_wp = self.asv_handler.current_state.photo_mission_target_wp2
            qty_req = self.asv_handler.current_state.photo_mission_qty_requested

            # Ambil Counter Saat Ini
            taken_1 = (
                self.asv_handler.current_state.photo_mission_qty_taken_1
            )  # Surface
            taken_2 = (
                self.asv_handler.current_state.photo_mission_qty_taken_2
            )  # Underwater

            # Kondisi Dasar: Misi Aktif & Dalam Segmen
            mission_active = start_wp != -1 and stop_wp != -1
            in_segment = (start_wp <= current_wp) and (current_wp < stop_wp)

            current_state_photo = self.asv_handler.current_state

        if mission_active and in_segment:
            # --- CEK 1: TRIGGER SURFACE (CAM1) ---
            cooldown_surf_ok = (
                current_time - self.last_auto_photo_time_surface
                > self.photo_mission_cooldown_sec
            )

            if (taken_1 < qty_req) and cooldown_surf_ok:
                # Trigger Capture Mode Surface
                self.handle_photography_mission(current_state_photo, mode="surface")

                # Update State
                with self.asv_handler.state_lock:
                    self.asv_handler.current_state.photo_mission_qty_taken_1 += 1

                self.last_auto_photo_time_surface = current_time
                print(
                    f"[Mission] Auto Surface ({taken_1 + 1}/{qty_req}) di WP {current_wp}"
                )

            # --- CEK 2: TRIGGER UNDERWATER (CAM2) ---
            # Berjalan independen dari Cek 1
            cooldown_under_ok = (
                current_time - self.last_auto_photo_time_underwater
                > self.photo_mission_cooldown_sec
            )

            if (taken_2 < qty_req) and cooldown_under_ok:
                # Trigger Capture Mode Underwater
                self.handle_photography_mission(current_state_photo, mode="underwater")

                # Update State
                with self.asv_handler.state_lock:
                    self.asv_handler.current_state.photo_mission_qty_taken_2 += 1

                self.last_auto_photo_time_underwater = current_time
                print(
                    f"[Mission] Auto Underwater ({taken_2 + 1}/{qty_req}) di WP {current_wp}"
                )

        return annotated_frame

    # -----------------------------------------

    def handle_autonomous_navigation(self, detections, frame_width, control_mode):
        """Handles autonomous navigation based on object detections."""

        # 1. CEK COOLDOWN TERLEBIH DAHULU
        if (time.time() - self.last_buoy_seen_time) > self.obstacle_cooldown_period:
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})

        # 2. Baru cek apakah ada deteksi
        if not detections:
            return

        # 3. Cek Mode Navigasi (PENTING: Kapal harus dalam mode AUTO)
        # [FIX RED-07] Gunakan variabel lokal control_mode, bukan current_state.control_mode
        if control_mode != "AUTO":
            return

        valid_buoys = []

        for det in detections:
            cls = det.get("class", "")
            conf = det.get("confidence", 0.0)

            # Abaikan jika tingkat kepercayaan AI di bawah threshold
            if conf < self.poi_confidence_threshold:
                continue

            # --- [PERBAIKAN] Kumpulkan semua 5 kelas baru ---
            if cls in ["bola-merah", "bola-hijau", "kotak-hijau", "kotak-biru", "bola-biru"]:
                valid_buoys.append(det)

        if valid_buoys:
            # Cari objek yang paling dekat dengan kapal
            closest_buoy = min(
                valid_buoys, 
                key=lambda b: b.get("distance_cm") if b.get("distance_cm") is not None else float("inf")
            )
            
            distance_to_closest = closest_buoy.get("distance_cm")
            if distance_to_closest is None:
                distance_to_closest = float("inf")

            # Jika objek berada dalam jarak aktivasi penghindaran
            if distance_to_closest < self.obstacle_activation_distance:
                self.last_buoy_seen_time = time.time()

                payload_obs = {
                    "active": True,
                    "obstacle_class": closest_buoy.get("class", "unknown"),
                    "object_center_x": closest_buoy["center"][0],
                    "frame_width": frame_width,
                }

                logging.info(
                    f"[Vision] Target manuver aktif ({closest_buoy.get('class')}) pada jarak {distance_to_closest:.1f}cm."
                )

                # --- MENGIRIM DATA KE ASV HANDLER ---
                self.asv_handler.process_command(
                    "VISION_TARGET_UPDATE",
                    payload_obs,
                )
                return
        # -------------------------------------

    def handle_photography_mission(self, current_state, mode="surface"):
        """
        Menangani pengambilan foto otomatis berdasarkan mode yang diminta.
        Mode: "surface" (CAM1) atau "underwater" (CAM2).
        """
        frame_to_use = None
        mission_name = None
        filename_prefix = None
        image_count = 0

        # --- SELEKSI FRAME & LOCK BERDASARKAN MODE ---
        if mode == "surface":
            # Ambil Frame CAM 1
            with VisionService._frame_lock_cam1:
                if VisionService._latest_processed_frame_cam1 is not None:
                    frame_to_use = VisionService._latest_processed_frame_cam1.copy()

            mission_name = "Surface Imaging"
            filename_prefix = "surface_auto"
            image_count = self.surface_image_count
            self.surface_image_count += 1

        elif mode == "underwater":
            # Ambil Frame CAM 2
            with VisionService._frame_lock_cam2:
                if VisionService._latest_raw_frame_cam2 is not None:
                    frame_to_use = VisionService._latest_raw_frame_cam2.copy()

            if frame_to_use is None:
                print("[Mission] Gagal Auto-Capture Underwater: Frame CAM 2 None.")
                return

            mission_name = "Underwater Imaging"
            filename_prefix = "underwater_auto"
            image_count = self.underwater_image_count
            self.underwater_image_count += 1

        else:
            print(f"[Mission] Mode tidak dikenal: {mode}")
            return

        if frame_to_use is None:
            return

        # --- PROSES OVERLAY & SIMPAN ---
        try:
            overlay_data = create_overlay_from_html(
                asdict(current_state), mission_type=mission_name
            )
            snapshot = apply_overlay(frame_to_use, overlay_data)
        except Exception as e:
            print(f"[Mission] Gagal Overlay Auto: {e}")
            snapshot = frame_to_use

        filename = f"{filename_prefix}_{image_count}.jpg"
        save_dir = os.path.join(os.getcwd(), "logs", "captures")
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, filename)

        cv2.imwrite(save_path, snapshot)
        print(f"[Photography] Auto-Capture ({mode}) Disimpan: {save_path}")

    def set_gui_listening(self, status: bool):
        with self.settings_lock:
            if self.gui_is_listening != status:
                self.gui_is_listening = status
