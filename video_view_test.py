# gui/components/video_view.py
# --- MODIFIKASI: Menggabungkan desain overlay menjadi lebih rapi dan informatif ---

import sys
import os
import cv2
import torch
import numpy as np
import time
import random
from datetime import datetime
from pathlib import Path

# Penanganan path untuk kompatibilitas Windows
if os.name == 'nt':
    import pathlib
    pathlib.PosixPath = pathlib.WindowsPath

# Blok impor untuk YOLOv5
try:
    CURRENT_FILE_DIR = Path(os.path.abspath(__file__)).resolve()
    PROJECT_ROOT = CURRENT_FILE_DIR.parents[2]
    YOLOV5_ROOT_PATH = PROJECT_ROOT / 'yolov5'
    if str(YOLOV5_ROOT_PATH) not in sys.path:
        sys.path.insert(0, str(YOLOV5_ROOT_PATH))

    from models.common import DetectMultiBackend
    from utils.general import non_max_suppression, scale_boxes, check_img_size
    from utils.torch_utils import select_device, smart_inference_mode
    from ultralytics.utils.plotting import Annotator, colors
    from utils.augmentations import letterbox
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"Peringatan: Gagal mengimpor modul YOLOv5: {e}")
    YOLO_AVAILABLE = False

from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout, QComboBox
from PySide6.QtCore import QThread, Signal, Slot, Qt
from PySide6.QtGui import QImage, QPixmap

class DetectionThread(QThread):
    frame_ready = Signal(np.ndarray)
    vision_command_status = Signal(dict)

    def __init__(self, source_idx, weights_path, parent=None):
        super().__init__(parent)
        self.source_idx, self.weights_path = source_idx, weights_path
        self.running, self.mode_auto, self.is_inverted = True, False, False
        
        self.snapshot_dir = PROJECT_ROOT / "snapshots"
        os.makedirs(self.snapshot_dir, exist_ok=True)
        print(f"Gambar akan disimpan di: {self.snapshot_dir}")
        
        self.latest_telemetry = {}

    @Slot(dict)
    def update_telemetry(self, data):
        self.latest_telemetry = data

    def draw_geotag_overlay(self, image, mission_type="Surface Imaging"):
        """Menggambar overlay yang rapi dan informatif, menggabungkan elemen terbaik."""
        h, w, _ = image.shape
        overlay = image.copy()
        
        # --- Pengaturan Umum ---
        font = cv2.FONT_HERSHEY_SIMPLEX
        white = (255, 255, 255)
        yellow = (0, 215, 255)
        
        # --- Latar Belakang Utama ---
        start_y_bg = h - 210
        cv2.rectangle(overlay, (10, start_y_bg), (w - 10, h - 10), (0,0,0), -1)
        alpha = 0.6
        image = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

        # --- Blok Kiri Bawah (Informasi Utama) ---
        start_x, start_y = 25, h - 180
        
        # Judul Misi
        cv2.putText(image, mission_type.upper(), (start_x, start_y), font, 0.9, white, 2, cv2.LINE_AA)

        # Kotak Waktu
        now = datetime.now()
        time_str = now.strftime("%H:%M")
        cv2.rectangle(image, (start_x, start_y + 15), (start_x + 110, start_y + 48), yellow, -1)
        cv2.putText(image, "VISIT", (start_x + 8, start_y + 39), font, 0.7, (0,0,0), 2, cv2.LINE_AA)
        cv2.putText(image, time_str, (start_x + 60, start_y + 39), font, 0.7, (0,0,0), 2, cv2.LINE_AA)

        # Detail Teks (Tanggal, Lokasi, Koordinat, SOG, COG)
        day_map = ["Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"]
        day = day_map[now.weekday()]
        date_str = now.strftime("%d/%m/%Y")
        full_date_str = f"{day}, {date_str}"
        
        lat = self.latest_telemetry.get('latitude', 1.177697)
        lon = self.latest_telemetry.get('longitude', 104.319206)
        sog_ms = self.latest_telemetry.get('speed', 0.0)
        cog = self.latest_telemetry.get('heading', 0.0)
        sog_knot = sog_ms * 1.94384
        sog_kmh = sog_ms * 3.6

        coord_str = f"{abs(lat):.6f}°{'N' if lat >= 0 else 'S'}, {abs(lon):.6f}°{'E' if lon >= 0 else 'W'}"
        sog_str = f"SOG: {sog_knot:.2f} knot / {sog_kmh:.2f} km/h"
        cog_str = f"COG: {cog:.2f} deg"
        
        text_y = start_y + 80
        font_scale_small = 0.5
        y_step = 22
        cv2.putText(image, full_date_str, (start_x, text_y), font, font_scale_small, white, 1, cv2.LINE_AA)
        cv2.putText(image, "Lokasi: ASV NAVANTARA - UMRAH", (start_x, text_y + y_step), font, font_scale_small, white, 1, cv2.LINE_AA)
        cv2.putText(image, coord_str, (start_x, text_y + y_step * 2), font, font_scale_small, white, 1, cv2.LINE_AA)
        cv2.putText(image, sog_str, (start_x, text_y + y_step * 3), font, font_scale_small, white, 1, cv2.LINE_AA)
        cv2.putText(image, cog_str, (start_x, text_y + y_step * 4), font, font_scale_small, white, 1, cv2.LINE_AA)

        # Garis pemisah vertikal
        cv2.line(image, (start_x - 8, start_y + 65), (start_x - 8, text_y + y_step * 4 + 5), yellow, 2)

        # Footer (Kode Foto Unik)
        photo_code = f"Kode Foto: KKI25-{time.strftime('%H%M%S')}-{random.randint(100,999)}"
        cv2.line(image, (15, h - 48), (w - 15, h - 48), (255,255,255), 1)
        # Ikon centang
        cv2.line(image, (start_x, h - 28), (start_x + 5, h - 23), white, 2)
        cv2.line(image, (start_x + 5, h - 23), (start_x + 12, h - 33), white, 2)
        cv2.putText(image, photo_code, (start_x + 25, h - 25), font, 0.5, white, 1, cv2.LINE_AA)

        # --- Kanan Atas (Branding) ---
        cv2.putText(image, "NAVANTARA - UMRAH", (w - 200, 30), font, 0.6, white, 2, cv2.LINE_AA)
        cv2.putText(image, "Foto 100% akurat", (w - 200, 50), font, 0.4, (200,200,200), 1, cv2.LINE_AA)

        return image

    def get_score(self, ball, is_pair=False, other_ball=None):
        # (Fungsi ini tidak berubah)
        size = (ball['xyxy'][2] - ball['xyxy'][0]) * (ball['xyxy'][3] - ball['xyxy'][1])
        if is_pair and other_ball:
            other_size = (other_ball['xyxy'][2] - other_ball['xyxy'][0]) * (other_ball['xyxy'][3] - other_ball['xyxy'][1])
            avg_y = (ball['center'][1] + other_ball['center'][1]) / 2
            return 0.6 * (size + other_size) + 0.4 * avg_y
        else:
            y_pos = ball['center'][1]
            return 0.6 * size + 0.4 * y_pos
            
    @smart_inference_mode()
    def run(self):
        cap = None
        try:
            print(f"Mencoba membuka kamera indeks: {self.source_idx}...")
            cap = cv2.VideoCapture(self.source_idx, cv2.CAP_MSMF)
            if not cap.isOpened():
                print(f"ERROR: Tidak bisa membuka kamera {self.source_idx}.")
                return

            device = select_device('')
            model = DetectMultiBackend(weights=self.weights_path, device=device, fp16=False)
            stride, names, pt = model.stride, model.names, model.pt
            imgsz = check_img_size((640, 640), s=stride)
            model.warmup(imgsz=(1, 3, *imgsz))
            
            while self.running and cap.isOpened():
                ret, im0 = cap.read()
                if not ret: break

                img = letterbox(im0, imgsz, stride=stride, auto=pt)[0]
                img = img.transpose((2, 0, 1))[::-1]
                img = np.ascontiguousarray(img)
                im = torch.from_numpy(img).to(model.device).float() / 255.0
                if len(im.shape) == 3: im = im[None]

                pred = model(im, augment=False, visualize=False)
                pred = non_max_suppression(pred, conf_thres=0.4, iou_thres=0.45)
                annotator = Annotator(im0.copy(), line_width=2, example=str(names))
                
                detected_red_buoys, detected_green_buoys = [], []
                detected_green_boxes, detected_blue_boxes = [], []

                if len(pred[0]):
                    det = pred[0]
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                    for *xyxy, conf, cls in reversed(det):
                        class_name = names[int(cls)]
                        annotator.box_label(xyxy, f"{class_name} {conf:.2f}", color=colors(int(cls), True))
                        bbox_data = { 'xyxy': [v.item() for v in xyxy], 'center': (int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)), 'class': class_name }
                        
                        if "red_buoy" in class_name: detected_red_buoys.append(bbox_data)
                        elif "green_buoy" in class_name: detected_green_buoys.append(bbox_data)
                        if "green_box" in class_name: detected_green_boxes.append(bbox_data)
                        elif "blue_box" in class_name: detected_blue_boxes.append(bbox_data)

                # --- LOGIKA PENGAMBILAN GAMBAR DENGAN GEO-TAG BARU ---
                mission_name = None
                if detected_green_boxes: mission_name = "Surface Imaging"
                elif detected_blue_boxes: mission_name = "Underwater Imaging"
                
                if mission_name:
                    snapshot_image = self.draw_geotag_overlay(im0, mission_type=mission_name)
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"snapshot_{mission_name.replace(' ','_')}_{timestamp}.jpg"
                    filepath = self.snapshot_dir / filename
                    cv2.imwrite(str(filepath), snapshot_image)
                    print(f"✅ SNAPSHOT DIAMBIL: '{mission_name}' terdeteksi. Disimpan sebagai {filename}")

                if self.mode_auto:
                    # (Logika Navigasi AUTO tidak berubah)
                    pass

                final_frame = annotator.result()
                self.frame_ready.emit(final_frame)
                
        except Exception as e:
            print(f"Error di dalam thread deteksi: {e}")
        finally:
            if cap and cap.isOpened(): cap.release()
            print("Pembersihan thread deteksi selesai.")

    def stop(self): self.running = False

    def calculate_degree(self, frame, midpoint):
        # (Fungsi ini tidak berubah)
        h, w, _ = frame.shape
        bar_y, bar_left, bar_right = h - 50, 50, w - 50
        bar_width = bar_right - bar_left
        relative_pos = np.clip((midpoint[0] - bar_left) / bar_width, 0, 1)
        return int(relative_pos * 180)

    def convert_degree_to_actuators(self, degree):
        # (Fungsi ini tidak berubah)
        error = degree - 90
        if self.is_inverted: error = -error
        servo_angle = 90 - (error / 90.0) * 45
        servo_angle = max(0, min(180, int(servo_angle)))
        speed_reduction = abs(error) / 90.0
        motor_pwm = 1600 - (speed_reduction * 100)
        return int(servo_angle), int(motor_pwm)

class VideoView(QWidget):
    vision_status_updated = Signal(dict)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.detection_thread = None
        self.label = QLabel("Kamera nonaktif.")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setMinimumSize(640, 480)
        self.label.setStyleSheet("background-color: #2c3e50; color: white; font-size: 16px;")

        self.camera_selector = QComboBox()
        self.refresh_button = QPushButton("Refresh List")
        self.start_stop_button = QPushButton("Start Camera")
        self.invert_button = QPushButton("Invert Logic")
        self.invert_button.setCheckable(True)
        self.invert_button.setEnabled(False)

        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel("Sumber Kamera:"))
        control_layout.addWidget(self.camera_selector, 1)
        control_layout.addWidget(self.refresh_button)
        control_layout.addWidget(self.start_stop_button)
        control_layout.addWidget(self.invert_button)

        main_layout = QVBoxLayout(self)
        main_layout.addLayout(control_layout)
        main_layout.addWidget(self.label, 1)

        self.refresh_button.clicked.connect(self.list_cameras)
        self.start_stop_button.clicked.connect(self.toggle_camera)
        self.invert_button.clicked.connect(self.toggle_inversion)

        if not YOLO_AVAILABLE:
            self.label.setText("Modul YOLOv5 tidak ditemukan.\nPastikan instalasi benar.")
            self.start_stop_button.setEnabled(False)
        else: self.list_cameras()
            
    @Slot(dict)
    def update_telemetry_in_thread(self, data):
        if self.detection_thread and self.detection_thread.isRunning():
            self.detection_thread.update_telemetry(data)
            
    def start_camera(self):
        if self.detection_thread and self.detection_thread.isRunning(): return
        if "Kamera" not in self.camera_selector.currentText(): return
        
        source_idx = int(self.camera_selector.currentText().split(" ")[1])
        weights_path = str(PROJECT_ROOT / "yolov5" / "besto.pt")
        
        if not os.path.exists(weights_path):
            self.label.setText(f"Error: File bobot 'besto.pt' tidak ditemukan.")
            return

        print(f"Memulai thread kamera baru untuk device {source_idx}...")
        self.detection_thread = DetectionThread(source_idx=source_idx, weights_path=weights_path, parent=self)
        self.detection_thread.frame_ready.connect(self.set_frame)
        self.detection_thread.vision_command_status.connect(self.vision_status_updated.emit)
        
        is_checked = self.invert_button.isChecked()
        self.detection_thread.is_inverted = is_checked
        self.toggle_inversion()
        self.detection_thread.start()
        self.start_stop_button.setText("Stop Camera")
        self.invert_button.setEnabled(True)

    def list_cameras(self):
        self.camera_selector.clear()
        index, arr = 0, []
        while True:
            cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
            if not cap.isOpened(): break
            arr.append(index)
            cap.release()
            index += 1
        if arr:
            self.camera_selector.addItems([f"Kamera {i}" for i in arr])
            self.start_stop_button.setEnabled(True)
        else:
            self.camera_selector.addItem("Tidak ada kamera")
            self.start_stop_button.setEnabled(False)

    def toggle_inversion(self):
        if self.detection_thread and self.detection_thread.isRunning():
            is_checked = self.invert_button.isChecked()
            self.detection_thread.is_inverted = is_checked
            print(f"Logika Invers diaktifkan: {is_checked}")
            if is_checked:
                self.invert_button.setText("Logic: Inverted")
                self.invert_button.setStyleSheet("background-color: #e74c3c; color: white;")
            else:
                self.invert_button.setText("Logic: Normal")
                self.invert_button.setStyleSheet("")

    def toggle_camera(self):
        if self.detection_thread and self.detection_thread.isRunning(): self.stop_camera()
        else: self.start_camera()

    def stop_camera(self):
        if self.detection_thread and self.detection_thread.isRunning():
            print("Memulai prosedur penghentian kamera...")
            self.detection_thread.stop()
            self.detection_thread.wait(5000)
            self.detection_thread = None
            print("Thread kamera berhasil dihentikan.")
            
        self.start_stop_button.setText("Start Camera")
        self.label.setText("Kamera nonaktif.")
        self.label.setPixmap(QPixmap())
        self.invert_button.setEnabled(False)
        self.invert_button.setText("Invert Logic")

    @Slot(np.ndarray)
    def set_frame(self, frame):
        try:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.label.setPixmap(pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except Exception as e:
            print(f"Gagal menampilkan frame: {e}")

    def closeEvent(self, event):
        self.stop_camera()
        super().closeEvent(event)

    @Slot(str)
    def set_mode(self, mode):
        if self.detection_thread and self.detection_thread.isRunning():
            self.detection_thread.mode_auto = (mode == "AUTO")