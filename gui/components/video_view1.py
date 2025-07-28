# gui/components/video_view.py
# --- MODIFIKASI: Memperbaiki NameError dengan menambahkan impor 'Slot' ---

import sys
import os
import cv2
import torch
import numpy as np
from pathlib import Path

# Penyesuaian path untuk Windows dan import YOLOv5
if os.name == 'nt':
    import pathlib
    pathlib.PosixPath = pathlib.WindowsPath

try:
    CURRENT_FILE_DIR = Path(os.path.abspath(__file__)).resolve()
    PROJECT_ROOT = CURRENT_FILE_DIR.parents[2]
    YOLOV5_ROOT_PATH = PROJECT_ROOT / 'yolov5'
    if str(YOLOV5_ROOT_PATH) not in sys.path:
        sys.path.insert(0, str(YOLOV5_ROOT_PATH))

    from models.common import DetectMultiBackend
    from utils.dataloaders import LoadStreams
    from utils.general import non_max_suppression, scale_boxes, check_img_size
    from utils.torch_utils import select_device
    from ultralytics.utils.plotting import Annotator, colors
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"Peringatan: Gagal mengimpor modul YOLOv5: {e}")
    YOLO_AVAILABLE = False

from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout, QComboBox
# --- PERBAIKAN UTAMA: Tambahkan 'Slot' ke baris impor ini ---
from PySide6.QtCore import QTimer, Qt, Signal, QThread, Slot 
from PySide6.QtGui import QImage, QPixmap, QFont

# --- Worker Thread untuk Deteksi (Tidak Berubah) ---
class DetectionThread(QThread):
    frame_ready = Signal(np.ndarray)
    vision_data_updated = Signal(dict)

    def __init__(self, source, weights_path, parent=None):
        super().__init__(parent)
        self.source = source
        self.weights_path = weights_path
        self.running = True

    def run(self):
        try:
            device = select_device('')
            model = DetectMultiBackend(weights=self.weights_path, device=device, fp16=False)
            stride, names, pt = model.stride, model.names, model.pt
            imgsz = check_img_size((640, 640), s=stride)
            model.warmup(imgsz=(1, 3, *imgsz))
            dataset = LoadStreams(self.source, img_size=imgsz, stride=stride, auto=pt)
            
            for path, im, im0s, vid_cap, s in dataset:
                if not self.running:
                    break
                im = torch.from_numpy(im).to(model.device).float() / 255.0
                if len(im.shape) == 3:
                    im = im[None]
                pred = model(im, augment=False, visualize=False)
                pred = non_max_suppression(pred, conf_thres=0.4, iou_thres=0.45)
                for i, det in enumerate(pred):
                    im0 = im0s[i].copy()
                    annotator = Annotator(im0, line_width=2, example=str(names))
                    red_centers, green_centers = [], []
                    if len(det):
                        det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                        for *xyxy, conf, cls in reversed(det):
                            class_name = names[int(cls)]
                            annotator.box_label(xyxy, f"{class_name} {conf:.2f}", color=colors(int(cls), True))
                            cx = int((xyxy[0] + xyxy[2]) / 2)
                            cy = int((xyxy[1] + xyxy[3]) / 2)
                            if "Red_Ball" in class_name: red_centers.append((cx, cy))
                            elif "Green_Ball" in class_name: green_centers.append((cx, cy))
                    if red_centers and green_centers:
                        red_centers.sort(key=lambda pt: -pt[1])
                        green_centers.sort(key=lambda pt: -pt[1])
                        red_nearest = red_centers[0]
                        green_nearest = green_centers[0]
                        midpoint = ((red_nearest[0] + green_nearest[0]) // 2, (red_nearest[1] + green_nearest[1]) // 2)
                        cv2.line(im0, red_nearest, green_nearest, (200, 200, 200), 2)
                        cv2.circle(im0, midpoint, 6, (255, 0, 255), -1)
                        degree = self.calculate_degree(im0, midpoint)
                        self.vision_data_updated.emit({'detected': True, 'degree': degree})
                    else:
                        self.vision_data_updated.emit({'detected': False, 'degree': None})
                    self.frame_ready.emit(annotator.result())
        except Exception as e:
            print(f"Error di dalam thread deteksi: {e}")
        print("Thread deteksi dihentikan.")

    def calculate_degree(self, frame, midpoint):
        h, w, _ = frame.shape
        bar_y = h - 50
        bar_left, bar_right = 50, w - 50
        bar_width = bar_right - bar_left
        cv2.rectangle(frame, (bar_left, bar_y), (bar_right, bar_y + 20), (0, 140, 255), 2)
        relative_pos = np.clip((midpoint[0] - bar_left) / bar_width, 0, 1)
        degree = int(relative_pos * 180)
        indicator_x = int(bar_left + relative_pos * bar_width)
        cv2.rectangle(frame, (indicator_x - 15, bar_y), (indicator_x + 15, bar_y + 20), (0, 0, 255), -1)
        cv2.putText(frame, f"{degree}°", (indicator_x - 15, bar_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return degree

    def stop(self):
        self.running = False

# --- Kelas VideoView (Tidak Berubah, kecuali impor di atas) ---
class VideoView(QWidget):
    vision_data_updated = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.detection_thread = None
        self.label = QLabel("Kamera nonaktif. Pilih sumber dan klik 'Start Camera'.")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setMinimumSize(640, 480)
        self.label.setStyleSheet("background-color: #2c3e50; color: white;")
        self.camera_selector = QComboBox()
        self.refresh_button = QPushButton("Refresh List")
        self.start_stop_button = QPushButton("Start Camera")
        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel("Sumber Kamera:"))
        control_layout.addWidget(self.camera_selector, 1)
        control_layout.addWidget(self.refresh_button)
        control_layout.addWidget(self.start_stop_button)
        main_layout = QVBoxLayout(self)
        main_layout.addLayout(control_layout)
        main_layout.addWidget(self.label, 1)
        self.refresh_button.clicked.connect(self.list_cameras)
        self.start_stop_button.clicked.connect(self.toggle_camera)
        if not YOLO_AVAILABLE:
            self.label.setText("Modul YOLOv5 tidak ditemukan. Deteksi objek dinonaktifkan.")
            self.start_stop_button.setEnabled(False)
        else:
            self.list_cameras()

    def list_cameras(self):
        self.camera_selector.clear()
        indices = [i for i in range(10) if cv2.VideoCapture(i).isOpened()]
        if indices:
            self.camera_selector.addItems([f"Kamera {i}" for i in indices])
            self.start_stop_button.setEnabled(True)
        else:
            self.camera_selector.addItem("Tidak ada kamera")
            self.start_stop_button.setEnabled(False)

    def toggle_camera(self):
        if self.detection_thread and self.detection_thread.isRunning():
            self.stop_camera()
        else:
            self.start_camera()

    def start_camera(self):
        if "Kamera" not in self.camera_selector.currentText(): return
        source = self.camera_selector.currentText().split(" ")[1]
        weights_path = str(YOLOV5_ROOT_PATH / "best.pt")
        if not os.path.exists(weights_path):
            self.label.setText(f"Error: File model 'best.pt' tidak ditemukan.")
            return
        self.detection_thread = DetectionThread(source=source, weights_path=weights_path, parent=self)
        self.detection_thread.frame_ready.connect(self.set_frame)
        self.detection_thread.vision_data_updated.connect(self.vision_data_updated.emit)
        self.detection_thread.start()
        self.start_stop_button.setText("Stop Camera")

    def stop_camera(self):
        if self.detection_thread:
            self.detection_thread.stop()
            self.detection_thread.wait()
            self.detection_thread = None
        self.start_stop_button.setText("Start Camera")
        self.label.setText("Kamera nonaktif.")

    @Slot(np.ndarray)
    def set_frame(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image).scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.stop_camera()
        super().closeEvent(event)
