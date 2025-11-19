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
import sys
from pathlib import Path
from dataclasses import asdict

# --- [MIGRASI 1: Import Ultralytics] ---
from ultralytics import YOLO
# ---------------------------------------

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

        # Ambil threshold dari config untuk digunakan saat inferensi nanti
        vision_cfg = self.config.get("vision", {})
        self.conf_thres = float(vision_cfg.get("conf_threshold", 0.25))
        self.iou_thres = float(vision_cfg.get("iou_threshold", 0.45))

        # --- [MIGRASI 2: Panggil pemuat model Ultralytics] ---
        self.model = self._load_yolo_model(config)
        # -----------------------------------------------------

        self.settings_lock = threading.Lock()

        # Pengaturan awal
        self.gui_is_listening = False
        self.is_inverted = False

        # Pengaturan dari config.json
        self.camera_index_1 = int(vision_cfg.get("camera_index_1", 0))
        self.camera_index_2 = int(vision_cfg.get("camera_index_2", 1))
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
        self.last_auto_photo_time = 0

        # Pengaturan deteksi kamera
        cam_detect_cfg = self.config.get("camera_detection", {
            "red_buoy": 20.0,   
            "green_buoy": 20.0, 
            "buoy": 20.0        
        })
        self.FOCAL_LENGTH_PIXELS = cam_detect_cfg.get("focal_length_pixels", 600)
        self.OBJECT_REAL_WIDTHS_CM = cam_detect_cfg.get("object_real_widths_cm", {})

        print("[VisionService] Layanan Visi (YOLOv11 + TensorRT Ready) diinisialisasi.")

    # --- [FUNGSI BARU: ULTRALYTICS LOADER] ---
    def _load_yolo_model(self, config):
        """Memuat model YOLOv11 dengan prioritas TensorRT (.engine) lalu PyTorch (.pt)."""
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
                print("[VisionService] Menggunakan akselerasi hardware Jetson (CUDA/TensorRT).")
                model_path = str(engine_path)
                task_msg = "TensorRT Engine"
            
            # 2. Fallback ke PyTorch (.pt)
            elif pt_path.exists():
                print(f"[VisionService] Warning: .engine tidak ditemukan. Fallback ke .pt: {pt_path}")
                print("[VisionService] Performa mungkin lebih lambat dibandingkan TensorRT.")
                model_path = str(pt_path)
                task_msg = "PyTorch Model"
            
            else:
                print(f"[VisionService] KRITIS: Tidak ada model 'best.engine' atau 'best.pt' di {vision_dir}")
                return None

            # Muat Model menggunakan Ultralytics
            print(f"[VisionService] Memuat {task_msg}...")
            model = YOLO(model_path, task='detect')
            
            # Pemanasan awal (Warmup) agar inferensi pertama tidak lag
            # Gunakan ukuran gambar dummy standar 640x640
            print("[VisionService] Melakukan warmup model...")
            model.predict(source=np.zeros((640, 640, 3), dtype=np.uint8), verbose=False, device=0)
            
            print(f"[VisionService] Model berhasil dimuat dan siap.")
            return model

        except Exception as e:
            print(f"[VisionService] KRITIS: Gagal memuat model YOLOv11: {e}")
            traceback.print_exc()
            return None
    # -----------------------------------------

    def _estimate_distance(self, pixel_width, object_class):
        real_width_cm = self.OBJECT_REAL_WIDTHS_CM.get(object_class)
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

            # Adjusted HSV thresholds
            lower_red1 = np.array([0, 100, 60])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 60])
            upper_red2 = np.array([180, 255, 255])
            lower_green = np.array([35, 70, 35])
            upper_green = np.array([85, 255, 255])

            mask_r = cv2.bitwise_or(
                cv2.inRange(hsv_roi, lower_red1, upper_red1),
                cv2.inRange(hsv_roi, lower_red2, upper_red2),
            )
            mask_g = cv2.inRange(hsv_roi, lower_green, upper_green)

            red_px = cv2.countNonZero(mask_r)
            green_px = cv2.countNonZero(mask_g)
            total_px = roi.shape[0] * roi.shape[1]

            red_percentage = (red_px / total_px) * 100
            green_percentage = (green_px / total_px) * 100

            logging.info(
                f"[Vision] Color Analysis - Red: {red_percentage:.1f}%, Green: {green_percentage:.1f}%"
            )

            if green_percentage > 15 and green_px > red_px * 1.2:
                result = "green_buoy"
            elif red_percentage > 15 and red_px > green_px * 1.2:
                result = "red_buoy"
            else:
                result = detection.get("class")

            logging.info(f"[Vision] Buoy classified as: {result}")
            return result
        except Exception as e:
            print(f"[ColorValidation] Error: {e}")
            return detection.get("class")

    def run_capture_loops(self):
        """Memulai greenlet terpisah untuk menangkap frame dari kedua kamera."""
        if not self.running:
            self.running = True
            print("[VisionService] Memulai loop penangkapan untuk kedua kamera...")

            eventlet.spawn(
                self._capture_loop,
                self.camera_index_1,
                VisionService._frame_lock_cam1,
                "CAM1",
                "frame_cam1",
                True,
            )
            eventlet.spawn(
                self._capture_loop,
                self.camera_index_2,
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
        self, cam_index, frame_lock, cam_id_log, event_name, apply_detection
    ):
        """Loop utama untuk menangkap frame dari satu kamera."""
        cap = None
        print(
            f"[{cam_id_log}] Memulai loop untuk kamera index {cam_index} (AI: {apply_detection})..."
        )

        LOCAL_FEED_WIDTH = 320
        LOCAL_FEED_HEIGHT = 240

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
                    is_auto = (
                        self.asv_handler.current_state.control_mode == "AUTO"
                    )

                if apply_detection:
                    try:
                        processed_frame_ai = self.process_and_control(frame, is_auto)
                    except Exception as e:
                        print(f"[{cam_id_log}] Error dalam process_and_control: {e}")
                        traceback.print_exc()
                        eventlet.sleep(1)
                        continue

                    with frame_lock:
                        VisionService._latest_processed_frame_cam1 = (
                            processed_frame_ai.copy()
                        )
                        VisionService._latest_processed_frame = (
                            VisionService._latest_processed_frame_cam1
                        )
                    frame_to_emit = processed_frame_ai
                else:
                    with frame_lock:
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

                eventlet.sleep(0.02)

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
        Memicu pengambilan gambar (Surface/Underwater)
        dengan overlay telemetri.
        """
        print(f"[Capture] Menerima trigger untuk: {capture_type}")

        frame_to_use = None
        mission_name = None
        filename_prefix = None
        image_count = 0

        # 1. Dapatkan state telemetri saat ini
        try:
            with self.asv_handler.state_lock:
                current_state_dict = asdict(self.asv_handler.current_state)
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
            
            mission_name = "Surface Imaging" 
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

            mission_name = "Underwater Imaging"
            filename_prefix = "underwater"
            image_count = self.underwater_image_count
            self.underwater_image_count += 1

        else:
            return {"status": "error", "message": "Tipe capture tidak valid."}

        try:
            overlay_data = create_overlay_from_html(
                current_state_dict, mission_type=mission_name
            )
            snapshot = apply_overlay(frame_to_use, overlay_data)
        except Exception as e:
            print(f"[Capture] Gagal membuat overlay: {e}")
            snapshot = frame_to_use

        filename = f"{filename_prefix}_{image_count}.jpg"
        save_dir = os.path.join(os.getcwd(), "logs", "captures")
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, filename)

        try:
            ret, buffer = cv2.imencode(".jpg", snapshot, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ret:
                with open(save_path, "wb") as f:
                    f.write(buffer)
                print(f"[Capture] Foto BERHASIL disimpan: {save_path}")
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

    # --- [MIGRASI 3: INFERENSI BARU (ULTRALYTICS)] ---
    def process_and_control(self, frame, is_mode_auto):
        """
        Memproses frame menggunakan YOLOv11 (Ultralytics).
        Menggunakan 'results.boxes' untuk mengekstrak data deteksi
        secara manual tanpa Pandas, demi kompatibilitas format.
        """

        if not is_mode_auto:
            return frame

        if self.model is None:
            return frame

        if frame is None:
            logging.error("[Vision] Cannot process None frame")
            return frame

        detections = []
        annotated_frame = frame.copy()

        try:
            # logging.info("[Vision] Starting inference on frame...")

            # Ultralytics menangani resizing (imgsz=640) secara internal.
            # Kita cukup mengirimkan frame asli.
            # Gunakan eventlet.tpool jika ingin non-blocking di server context
            # conf dan iou diset saat prediksi.
            results = eventlet.tpool.execute(
                self.model.predict, 
                source=frame, 
                imgsz=640, 
                conf=self.conf_thres, 
                iou=self.iou_thres,
                verbose=False
            )
            
            result = results[0]  # Ambil hasil frame pertama

            # Parsing Manual Results (Pengganti Pandas)
            # Kita butuh format list of dicts:
            # [{'xyxy': [x1,y1,x2,y2], 'center': (cx,cy), 'class': 'name', 'confidence': 0.9}, ...]
            
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Koordinat [x1, y1, x2, y2] (sudah dalam skala gambar asli)
                    coords = box.xyxy[0].cpu().numpy().tolist()
                    x1, y1, x2, y2 = coords
                    
                    # Confidence Score
                    conf = float(box.conf[0].cpu().numpy())
                    
                    # Class ID & Name
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = result.names[cls_id]

                    # Hitung Center
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    detections.append({
                        "xyxy": [x1, y1, x2, y2],
                        "center": (center_x, center_y),
                        "class": cls_name,
                        "confidence": conf
                    })

            # logging.info(f"[Vision] Raw detections: {len(detections)} objects found")

            # Gambar bounding box (Annotated Frame)
            # result.plot() mengembalikan numpy array BGR yang sudah digambar
            annotated_frame = result.plot()

        except Exception as e:
            logging.error(f"[Vision] Inference error: {str(e)}")
            logging.error(traceback.format_exc())
            return frame

        # --- Bagian di bawah ini TIDAK DIUBAH (Logika Navigasi & Misi) ---
        validated_detections = []
        green_boxes_detected = []
        blue_boxes_detected = []

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

        annotated_frame = self._draw_distance_info(
            annotated_frame, validated_detections
        )

        if is_mode_auto:
            with self.asv_handler.state_lock:
                current_state_nav = self.asv_handler.current_state
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

        if green_boxes_detected or blue_boxes_detected:
            current_time = time.time()
            
            with self.asv_handler.state_lock:
                current_wp = self.asv_handler.current_state.nav_target_wp_index
                
                target1 = self.asv_handler.current_state.photo_mission_target_wp1
                target2 = self.asv_handler.current_state.photo_mission_target_wp2
                qty_req = self.asv_handler.current_state.photo_mission_qty_requested
                
                taken1 = self.asv_handler.current_state.photo_mission_qty_taken_1
                taken2 = self.asv_handler.current_state.photo_mission_qty_taken_2

                cooldown_ready = (current_time - self.last_auto_photo_time > self.photo_mission_cooldown_sec)

                active_target = None
                if target1 != -1 and current_wp == target1 and taken1 < qty_req:
                    active_target = 1
                elif target2 != -1 and current_wp == target2 and taken2 < qty_req:
                    active_target = 2

                if active_target and cooldown_ready:
                    taken_now = taken1 if active_target == 1 else taken2
                    print(f"[Vision] Misi Foto Otomatis (Target {active_target}): Mengambil foto {taken_now + 1} / {qty_req}")

                    current_state_photo = self.asv_handler.current_state 

                    self.handle_photography_mission(
                        frame, green_boxes_detected, blue_boxes_detected, current_state_photo
                    )

                    if active_target == 1:
                        self.asv_handler.current_state.photo_mission_qty_taken_1 += 1
                    else:
                        self.asv_handler.current_state.photo_mission_qty_taken_2 += 1
                        
                    self.last_auto_photo_time = current_time

        return annotated_frame
    # -----------------------------------------

    def handle_autonomous_navigation(self, detections, frame_width, current_state):
        """Handles autonomous navigation based on object detections."""

        if not detections:
            # logging.info("[Vision] No objects detected for navigation")
            return

        detected_classes = [d.get("class", "unknown") for d in detections]
        # logging.info(f"[Vision] Processing navigation for objects: {detected_classes}")

        if current_state.control_mode != "AUTO":
            return

        red_buoys = []
        green_buoys = []

        for det in detections:
            cls = det.get("class", "")
            conf = det.get("confidence", 0.0)
            dist = det.get("distance_cm", float("inf"))

            if conf < self.poi_confidence_threshold:
                continue

            if cls == "red_buoy":
                red_buoys.append(det)
            elif cls == "green_buoy":
                green_buoys.append(det)

        # Simple Avoidance (Statis)
        all_buoys = red_buoys + green_buoys
        
        if all_buoys:
            # Cari buoy terdekat
            closest_buoy = min(
                all_buoys, key=lambda b: b.get("distance_cm", float("inf"))
            )
            distance_to_closest = closest_buoy.get("distance_cm", float("inf"))
            
            # Jika jarak < threshold aktivasi, kirim target ke Handler
            if distance_to_closest < self.obstacle_activation_distance:
                self.last_buoy_seen_time = time.time()

                # Payload disederhanakan untuk Simple Avoidance
                payload_obs = {
                    "active": True,
                    "obstacle_class": closest_buoy.get("class", "unknown"),
                    "object_center_x": closest_buoy["center"][0],
                    "frame_width": frame_width,
                }

                logging.info(f"[Vision] Obstacle detected ({closest_buoy.get('class')}) at {distance_to_closest:.1f}cm. Sending update.")

                self.asv_handler.process_command(
                    "VISION_TARGET_UPDATE",
                    payload_obs,
                )
                return

        # Cleanup / Cooldown
        if (time.time() - self.last_buoy_seen_time) > self.obstacle_cooldown_period:
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})

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
            overlay_data = create_overlay_from_html(
                asdict(current_state), mission_type=mission_name
            )
            snapshot = apply_overlay(frame_to_use, overlay_data)
        except Exception as e:
            print(f"[Photography] Gagal membuat overlay: {e}")
            snapshot = frame_to_use

        filename = f"{filename_prefix}_{image_count}.jpg"
        
        save_dir = os.path.join(os.getcwd(), "logs", "captures")
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, filename)

        ret, buffer = cv2.imencode(".jpg", snapshot, [cv2.IMWRITE_JPEG_QUALITY, 90])
        
        if ret and buffer is not None:
            try:
                with open(save_path, "wb") as f:
                    f.write(buffer)
                print(f"[Photography] Foto Misi Otomatis BERHASIL disimpan: {save_path}")
            except Exception as e:
                print(f"[Photography] Gagal menyimpan file {filename}: {e}")
                if filename_prefix == "surface":
                    self.surface_image_count -= 1
                else:
                    self.underwater_image_count -= 1
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