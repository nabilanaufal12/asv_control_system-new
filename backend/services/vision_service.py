# backend/services/vision_service.py
# --- VERSI FINAL: Dengan Kesadaran Spasial Otomatis ---

# 1. Impor Pustaka Standar
import sys
from pathlib import Path
import os
import traceback
import pathlib
import collections
import threading
import time

# 2. Impor Pustaka Pihak Ketiga (Third-party)
import cv2
import torch
import requests
import numpy as np
from PySide6.QtCore import QObject, Signal, Slot

# 3. Logika Penyesuaian Path (Harus sebelum impor lokal)
# Patch untuk masalah PosixPath di Windows
if os.name == "nt":
    pathlib.PosixPath = pathlib.WindowsPath

# Menambahkan path YOLOv5 ke sistem SEBELUM impor lokal
backend_dir = Path(__file__).resolve().parents[1]
yolov5_path = backend_dir / "yolov5"
if str(yolov5_path) not in sys.path:
    sys.path.insert(0, str(yolov5_path))

# 4. Impor Pustaka Lokal (dari proyek Anda dan YOLOv5)
from utils.plots import Annotator
from backend.vision.overlay_utils import create_overlay_from_html, apply_overlay


def send_telemetry_to_firebase(telemetry_data, config):
    try:
        FIREBASE_URL = config.get("api_urls", {}).get("firebase_url")
        if not FIREBASE_URL:
            return
        data_to_send = {
            "gps": {
                "lat": telemetry_data.get("latitude", 0.0),
                "lng": telemetry_data.get("longitude", 0.0),
            },
            "hdg": telemetry_data.get("heading", 0.0),
            "cog": telemetry_data.get("cog", 0.0),
            "sog": telemetry_data.get("speed", 0.0),
            "jam": time.strftime("%H:%M:%S"),
        }
        requests.put(FIREBASE_URL, json=data_to_send, timeout=5)
    except Exception:
        pass


def upload_image_to_supabase(image_buffer, filename, config):
    try:
        api_config = config.get("api_urls", {})
        ENDPOINT_TEMPLATE, TOKEN = api_config.get("supabase_endpoint"), api_config.get(
            "supabase_token"
        )
        if not ENDPOINT_TEMPLATE or not TOKEN:
            return
        ENDPOINT = ENDPOINT_TEMPLATE.replace("{filename}", filename)
        headers = {
            "Authorization": f"Bearer {TOKEN}",
            "Content-Type": "image/jpeg",
            "x-upsert": "true",
        }
        response = requests.post(
            ENDPOINT, headers=headers, data=image_buffer.tobytes(), timeout=10
        )
        if response.ok:
            print(f"\n[Vision] ✅ Gambar '{filename}' berhasil diunggah ke Supabase.")
        else:
            print(
                f"\n[Vision] 🔥 Gagal mengunggah gambar: {response.status_code} {response.text}"
            )
    except Exception as e:
        print(f"\n[Vision] 🔥 Error koneksi Supabase: {e}")


class VisionService(QObject):
    frame_ready = Signal(np.ndarray)

    def __init__(self, config, asv_handler):
        super().__init__()
        self.config = config
        self.asv_handler = asv_handler
        self.running = False
        self.model = None
        self.is_inverted = False
        self.mode_auto = False
        self.restart_camera = False
        self.surface_image_count = 1
        self.underwater_image_count = 1

        vision_cfg = self.config.get("vision", {})
        self.conf_thresh = float(vision_cfg.get("conf_threshold", 0.4))
        self.iou_thresh = float(vision_cfg.get("iou_threshold", 0.45))
        self.device = str(
            vision_cfg.get("device", "cuda" if torch.cuda.is_available() else "cpu")
        )
        self.camera_index = int(vision_cfg.get("camera_index", 0))

        self.KNOWN_CLASSES = ["red_buoy", "green_buoy", "blue_box", "green_box"]
        self.poi_confidence_threshold = 0.65
        self.poi_validation_frames = 5
        self.recent_detections = collections.deque(maxlen=self.poi_validation_frames)
        self.investigation_in_progress = False

        self._initialize_model()

    def _initialize_model(self):
        try:
            weights_path = yolov5_path / "besto.pt"
            if not yolov5_path.exists() or not weights_path.exists():
                print(
                    f"[Vision] KRITIS: Direktori '{yolov5_path}' atau file bobot '{weights_path}' tidak ditemukan."
                )
                self.model = None
                return
            print("[Vision] Mempersiapkan model YOLOv5 dari sumber lokal...")
            self.model = torch.hub.load(
                str(yolov5_path),
                "custom",
                path=str(weights_path),
                source="local",
                force_reload=True,
                trust_repo=True,
            )
            self.model.conf = self.conf_thresh
            self.model.iou = self.iou_thresh
            print(f"[Vision] Model YOLOv5 berhasil dimuat di device {self.device}.")
        except Exception:
            print("[Vision] KRITIS: Gagal memuat model YOLOv5.")
            traceback.print_exc()
            self.model = None

    @Slot()
    def run(self):
        if self.model is None:
            print("[Vision] Model tidak tersedia, layanan visi tidak dapat dimulai.")
            return

        self.running = True
        cap = None

        while self.running:
            self.restart_camera = False
            print(
                f"[Vision] Mencoba membuka kamera indeks {self.camera_index} dengan backend DSHOW..."
            )
            cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)

            if not cap.isOpened():
                print(
                    f"[Vision] ERROR: Gagal membuka kamera indeks {self.camera_index}. Mencari kamera lain..."
                )
                found_cam_index = self.find_working_camera()
                if found_cam_index is not None:
                    self.camera_index = found_cam_index
                    cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
                else:
                    print(
                        "[Vision] ❌ Tidak ada kamera yang bisa dibuka. Mencoba lagi dalam 2 detik..."
                    )
                    time.sleep(2)
                    continue

            print(
                f"[Vision] Kamera {self.camera_index} berhasil dibuka. Memulai deteksi..."
            )

            while self.running and not self.restart_camera:
                try:
                    if not cap.isOpened():
                        break
                    ret, im0 = cap.read()
                    if not ret:
                        break

                    annotated_frame = self.process_frame(im0)
                    self.frame_ready.emit(annotated_frame)
                    time.sleep(0.01)

                except Exception as e:
                    print(f"\n[Vision] ERROR di dalam loop deteksi: {e}")
                    traceback.print_exc()
                    break

            if cap and cap.isOpened():
                cap.release()

            if self.restart_camera:
                print("\n[Vision] Me-restart stream kamera...")

        print("\n[Vision] Thread layanan visi telah dihentikan.")

    def process_frame(self, im0):
        results = self.model(im0)
        annotated_frame = results.render()[0].copy()

        detected_red_buoys, detected_green_buoys = [], []
        detected_green_boxes, detected_blue_boxes = [], []

        potential_pois = []
        df = results.pandas().xyxy[0]

        for _, row in df.iterrows():
            class_name = row["name"]
            confidence = row["confidence"]

            bbox_data = {
                "xyxy": [row["xmin"], row["ymin"], row["xmax"], row["ymax"]],
                "center": (
                    int((row["xmin"] + row["xmax"]) / 2),
                    int((row["ymin"] + row["ymax"]) / 2),
                ),
                "class": class_name,
            }

            if class_name in self.KNOWN_CLASSES:
                if "red_buoy" in class_name:
                    detected_red_buoys.append(bbox_data)
                elif "green_buoy" in class_name:
                    detected_green_buoys.append(bbox_data)
                if "green_box" in class_name:
                    detected_green_boxes.append(bbox_data)
                elif "blue_box" in class_name:
                    detected_blue_boxes.append(bbox_data)
            elif confidence > self.poi_confidence_threshold:
                potential_pois.append(bbox_data)

        if potential_pois:
            self.validate_and_trigger_investigation(potential_pois[0], im0)

        # --- MODIFIKASI DIMULAI DI SINI ---
        if self.mode_auto and (detected_green_boxes or detected_blue_boxes):
            self.handle_photography_mission(
                im0, detected_green_boxes, detected_blue_boxes
            )
        # --- AKHIR MODIFIKASI ---

        if self.mode_auto and not self.investigation_in_progress:
            annotated_frame = self.handle_auto_control(
                im0, annotated_frame, detected_red_buoys, detected_green_buoys
            )

        return annotated_frame

    def validate_and_trigger_investigation(self, poi_data, im0):
        """Memvalidasi POI selama beberapa frame dan mengirim perintah investigasi."""
        if (
            self.asv_handler.mission_phase == "PATROLLING"
            and not self.investigation_in_progress
        ):
            self.recent_detections.append(poi_data["class"])

            if (
                len(self.recent_detections) == self.poi_validation_frames
                and len(set(self.recent_detections)) == 1
            ):

                print(
                    f"\n[Vision] 🕵️‍♂️ Anomali terdeteksi & tervalidasi: '{poi_data['class']}'. Memulai investigasi!"
                )
                self.investigation_in_progress = True
                self.recent_detections.clear()

                degree = self.calculate_degree(im0, poi_data["center"])

                self.asv_handler.process_command(
                    "INVESTIGATE_POI",
                    {"class_name": poi_data["class"], "bearing_deg": degree},
                )

    def handle_photography_mission(self, original_frame, green_boxes, blue_boxes):
        mission_name = (
            "Surface Imaging"
            if green_boxes
            else "Underwater Imaging" if blue_boxes else None
        )
        if not mission_name:
            return

        self.asv_handler.process_command(
            "VISION_TARGET_UPDATE", {"active": True, "degree": 90}
        )
        time.sleep(0.5)

        current_telemetry = self.asv_handler.get_current_state()
        send_telemetry_to_firebase(current_telemetry, self.config)
        overlay_image = create_overlay_from_html(
            current_telemetry, mission_type=mission_name
        )
        snapshot_image = apply_overlay(original_frame.copy(), overlay_image)

        filename = (
            f"surface_{self.surface_image_count}.jpg"
            if mission_name == "Surface Imaging"
            else f"underwater_{self.underwater_image_count}.jpg"
        )
        if mission_name == "Surface Imaging":
            self.surface_image_count += 1
        else:
            self.underwater_image_count += 1

        ret, buffer = cv2.imencode(".jpg", snapshot_image)
        if ret:
            threading.Thread(
                target=upload_image_to_supabase,
                args=(buffer, filename, self.config),
                daemon=True,
            ).start()

        self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})

    def handle_auto_control(
        self, im0, annotated_frame, detected_red_buoys, detected_green_buoys
    ):
        """
        Logika utama untuk avoidance otomatis dengan kesadaran spasial.
        """
        detection_config = self.config.get("camera_detection", {})
        focal_length_px = detection_config.get("focal_length_pixels", 600)
        activation_dist_cm = detection_config.get(
            "target_activation_distance_cm", 100.0
        )
        activation_y_percent = detection_config.get("activation_zone_y_percent", 0.5)
        min_buoy_area_px = detection_config.get("min_buoy_area_px", 500)

        real_width_single_cm = detection_config.get("object_real_widths_cm", {}).get(
            "buoy_single", 10.0
        )
        real_width_gate_cm = detection_config.get("object_real_widths_cm", {}).get(
            "buoy_gate", 200.0
        )
        virtual_target_offset_cm = detection_config.get(
            "virtual_target_offset_cm", 40.0
        )

        h, w, _ = im0.shape
        activation_y_px = int(h * activation_y_percent)

        # --- Filter Awal ---
        def is_valid(buoy):
            box = buoy["xyxy"]
            area = (box[2] - box[0]) * (box[3] - box[1])
            return buoy["xyxy"][3] > activation_y_px and area > min_buoy_area_px

        red_buoys_filtered = [b for b in detected_red_buoys if is_valid(b)]
        green_buoys_filtered = [b for b in detected_green_buoys if is_valid(b)]

        target_object = None
        target_midpoint = None
        target_real_width_cm = 0

        # --- Logika Prioritas ---
        # 1. Prioritas #1: Deteksi Gerbang & Kesadaran Spasial
        if red_buoys_filtered and green_buoys_filtered:
            best_pair = max(
                ((r, g) for r in red_buoys_filtered for g in green_buoys_filtered),
                key=lambda p: self.get_score(p[0], is_pair=True, other_ball=p[1]),
            )

            # --- ANALISIS KONFIGURASI GERBANG ---
            red_x = best_pair[0]["center"][0]
            green_x = best_pair[1]["center"][0]

            is_currently_inverted = red_x > green_x
            if is_currently_inverted != self.is_inverted:
                self.is_inverted = is_currently_inverted
                print(
                    f"\n[VISION] 🧠 Kesadaran Spasial: Konfigurasi gerbang terdeteksi sebagai {'INVERTED' if self.is_inverted else 'NORMAL'}. Status diadaptasi."
                )

            target_object = best_pair
            target_midpoint = (
                (best_pair[0]["center"][0] + best_pair[1]["center"][0]) // 2,
                (best_pair[0]["center"][1] + best_pair[1]["center"][1]) // 2,
            )
            target_real_width_cm = real_width_gate_cm

        # 2. Prioritas #2: Deteksi Bola Tunggal (menggunakan status is_inverted terbaru)
        elif red_buoys_filtered or green_buoys_filtered:
            all_buoys = red_buoys_filtered + green_buoys_filtered
            best_buoy = max(all_buoys, key=self.get_score)

            is_red = "red_buoy" in best_buoy["class"]
            is_left_turn = (is_red and not self.is_inverted) or (
                not is_red and self.is_inverted
            )

            w_px = best_buoy["xyxy"][2] - best_buoy["xyxy"][0]
            dist_cm = (real_width_single_cm * focal_length_px) / w_px if w_px > 0 else 0
            offset_px = (
                (virtual_target_offset_cm * focal_length_px) / dist_cm
                if dist_cm > 0
                else 0
            )

            cx, cy = best_buoy["center"]
            v_cx = cx + int(offset_px) if is_left_turn else cx - int(offset_px)

            target_object = (best_buoy,)
            target_midpoint = (v_cx, cy)
            target_real_width_cm = real_width_single_cm

            cv2.circle(annotated_frame, target_midpoint, 10, (255, 255, 0), -1)
            cv2.line(
                annotated_frame, best_buoy["center"], target_midpoint, (255, 255, 0), 2
            )

        # --- Aktivasi Kontrol ---
        if target_midpoint:
            x_coords = [t["xyxy"][0] for t in target_object] + [
                t["xyxy"][2] for t in target_object
            ]
            lebar_px = max(x_coords) - min(x_coords)

            jarak_cm = (
                (target_real_width_cm * focal_length_px) / lebar_px
                if lebar_px > 0
                else 0
            )

            annotator = Annotator(annotated_frame, line_width=2)
            annotator.box_label(
                (
                    min(x_coords),
                    target_object[0]["xyxy"][1],
                    max(x_coords),
                    target_object[0]["xyxy"][3],
                ),
                f"Jarak: {jarak_cm:.1f} cm",
            )
            annotated_frame = annotator.result()

            if 0 < jarak_cm < activation_dist_cm:
                degree = self.calculate_degree(im0, target_midpoint)
                servo_angle, motor_pwm = self.convert_degree_to_actuators(degree)

                print(f"\n[VISION] ZONA AKTIVASI TERPENUHI (Jarak: {jarak_cm:.1f} cm).")
                print(
                    f"   => Perintah Dihitung: Motor PWM={motor_pwm}, Servo Angle={servo_angle} (Derajat Target: {degree})"
                )

                payload = {
                    "active": True,
                    "degree": degree,
                    "is_inverted": self.is_inverted,
                }
                self.asv_handler.process_command("VISION_TARGET_UPDATE", payload)
            else:
                print(
                    f"\r[VISION] STATUS: PASIF | Target di luar jangkauan ({jarak_cm:.1f} cm)",
                    end="",
                )
                self.asv_handler.process_command(
                    "VISION_TARGET_UPDATE", {"active": False}
                )
        else:
            print(
                "\r[VISION] STATUS: PASIF | Tidak ada target valid, mengikuti waypoint...",
                end="",
            )
            self.asv_handler.process_command("VISION_TARGET_UPDATE", {"active": False})

        return annotated_frame

    def stop(self):
        self.running = False
        print("\n[Vision] Perintah stop diterima.")

    def find_working_camera(self, max_indices=5):
        print("[Vision] Memindai kamera dengan backend DSHOW...")
        for idx in range(max_indices):
            test_cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
            if test_cap and test_cap.isOpened():
                print(f"[Vision]   -> Kamera ditemukan di indeks {idx}")
                test_cap.release()
                return idx
        return None

    @Slot(str)
    def set_mode(self, mode):
        self.mode_auto = mode == "AUTO"
        print(f"\n[Vision] Mode AUTO diatur ke: {self.mode_auto}")
        if not self.mode_auto:
            self.investigation_in_progress = False
            self.recent_detections.clear()

    @Slot(bool)
    def set_inversion(self, is_inverted):
        # Tombol manual masih bisa override, tapi sistem akan mengoreksi lagi jika melihat gerbang
        if self.is_inverted != is_inverted:
            self.is_inverted = is_inverted
            print(
                f"\n[VISION] ⚙️ Inversi diubah secara MANUAL menjadi: {self.is_inverted}"
            )

    def get_score(self, ball, is_pair=False, other_ball=None):
        """Menghitung skor untuk objek terdeteksi."""
        size = (ball["xyxy"][2] - ball["xyxy"][0]) * (ball["xyxy"][3] - ball["xyxy"][1])
        if is_pair and other_ball:
            other_size = (other_ball["xyxy"][2] - other_ball["xyxy"][0]) * (
                other_ball["xyxy"][3] - other_ball["xyxy"][1]
            )
            avg_y = (ball["center"][1] + other_ball["center"][1]) / 2
            return 0.6 * (size + other_size) + 0.4 * avg_y
        return 0.6 * size + 0.4 * ball["center"][1]

    def calculate_degree(self, frame, midpoint):
        """Mengubah posisi piksel horizontal menjadi sudut 0-180."""
        h, w, _ = frame.shape
        bar_left, bar_right = w * 0.1, w * 0.9
        relative_pos = np.clip((midpoint[0] - bar_left) / (bar_right - bar_left), 0, 1)
        return int(relative_pos * 180)

    def convert_degree_to_actuators(self, degree):
        """Mengonversi sudut target menjadi perintah PWM motor dan sudut servo."""
        vision_params = self.config.get("vision", {}).get("vision_control_params", {})
        motor_base = vision_params.get("motor_base_pwm", 1600)
        motor_reduction = vision_params.get("motor_reduction_factor", 100)
        servo_range = vision_params.get("servo_range_deg", 45)

        actuators_config = self.config.get("actuators", {})
        servo_default = actuators_config.get("servo_default_angle", 90)

        error = degree - 90
        if self.is_inverted:
            error = -error

        servo_angle = servo_default - (error / 90.0) * servo_range
        servo_angle = max(0, min(180, int(servo_angle)))

        speed_reduction_factor = abs(error) / 90.0
        motor_pwm = motor_base - (speed_reduction_factor * motor_reduction)

        return int(servo_angle), int(motor_pwm)
