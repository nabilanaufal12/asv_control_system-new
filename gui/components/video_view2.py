# gui/components/video_view.py
# --- MODIFIKASI: Mengubah sinyal agar lebih informatif ---

# (Impor tidak berubah)
import sys
import os
import cv2
import torch
import numpy as np
from pathlib import Path
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
    from utils.general import non_max_suppression, scale_boxes
    from utils.torch_utils import select_device, smart_inference_mode
    from ultralytics.utils.plotting import Annotator, colors
    YOLO_AVAILABLE = True
except ImportError as e:
    YOLO_AVAILABLE = False
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout, QComboBox
from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtGui import QImage, QPixmap, QFont


class VideoView(QWidget):
    # --- PERUBAHAN 1: Sinyal baru yang lebih deskriptif ---
    vision_data_updated = Signal(dict)

    def __init__(self, parent=None):
        # (Kode __init__ tidak berubah)
        super().__init__(parent)
        self.is_camera_active = False
        self.yolo_loaded = False
        self.cap = None
        self.timer = QTimer(self)
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
        self.timer.timeout.connect(self.update_frame)
        self.load_yolo_model()
        self.list_cameras()
    
    # (Metode lain seperti load_yolo_model, list_cameras, start/stop_camera tidak berubah)
    def load_yolo_model(self):
        if not YOLO_AVAILABLE:
            self.label.setText("Modul YOLOv5 tidak ditemukan.")
            return
        try:
            self.device = select_device('')
            model_path = str(YOLOV5_ROOT_PATH / "best.pt")
            self.model = DetectMultiBackend(model_path, device=self.device, dnn=False, fp16=False)
            self.model.warmup(imgsz=(1, 3, 640, 640))
            self.names = self.model.names
            self.yolo_loaded = True
            print(f"Model YOLOv5 berhasil dimuat dari: {model_path}")
        except Exception as e:
            self.yolo_loaded = False
            self.label.setText(f"Gagal memuat model 'best.pt':\n{e}")
    def list_cameras(self):
        self.camera_selector.clear()
        indices = [i for i in range(10) if cv2.VideoCapture(i, cv2.CAP_DSHOW).isOpened()]
        if indices:
            self.camera_selector.addItems([f"Kamera {i}" for i in indices])
            self.start_stop_button.setEnabled(True)
        else:
            self.camera_selector.addItem("Tidak ada kamera")
            self.start_stop_button.setEnabled(False)
    def toggle_camera(self):
        if self.is_camera_active: self.stop_camera()
        else: self.start_camera()
    def start_camera(self):
        if "Kamera" not in self.camera_selector.currentText(): return
        idx = int(self.camera_selector.currentText().split(" ")[1])
        self.cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if not self.cap.isOpened(): return
        self.is_camera_active = True
        self.start_stop_button.setText("Stop Camera")
        self.timer.start(30)
    def stop_camera(self):
        if not self.is_camera_active: return
        self.timer.stop()
        if self.cap: self.cap.release()
        self.is_camera_active = False
        self.start_stop_button.setText("Start Camera")
        self.label.setText("Kamera nonaktif.")
    def update_frame(self):
        if not self.is_camera_active or self.cap is None: return
        ret, frame = self.cap.read()
        if not ret:
            self.stop_camera()
            return
        if self.yolo_loaded:
            frame = self.process_frame_with_yolo(frame)
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image).scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.label.setPixmap(pixmap)


    @smart_inference_mode()
    def process_frame_with_yolo(self, frame):
        # ... (logika deteksi tidak berubah)
        img_resized = cv2.resize(frame, (640, 640))
        img_tensor = torch.from_numpy(img_resized).to(self.device).permute(2, 0, 1).float() / 255.0
        img_tensor = img_tensor.unsqueeze(0)
        pred = self.model(img_tensor, augment=False, visualize=False)
        pred = non_max_suppression(pred, conf_thres=0.4, iou_thres=0.45)
        annotator = Annotator(frame, line_width=2, example=str(self.names))
        red_centers, green_centers = [], []
        for det in pred:
            if len(det):
                det[:, :4] = scale_boxes(img_tensor.shape[2:], det[:, :4], frame.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    class_name = self.names[int(cls)]
                    annotator.box_label(xyxy, f"{class_name} {conf:.2f}", color=colors(int(cls), True))
                    cx = int((xyxy[0] + xyxy[2]) / 2)
                    cy = int((xyxy[1] + xyxy[3]) / 2)
                    if "Red_Ball" in class_name: red_centers.append((cx, cy))
                    elif "Green_Ball" in class_name: green_centers.append((cx, cy))
        
        red_centers.sort(key=lambda pt: -pt[1])
        green_centers.sort(key=lambda pt: -pt[1])

        # --- PERUBAHAN 2: Pancarkan sinyal dengan status deteksi ---
        if red_centers and green_centers:
            red_nearest = red_centers[0]
            green_nearest = green_centers[0]
            midpoint = ((red_nearest[0] + green_nearest[0]) // 2, (red_nearest[1] + green_nearest[1]) // 2)
            cv2.line(frame, red_nearest, green_nearest, (200, 200, 200), 2)
            cv2.circle(frame, midpoint, 6, (255, 0, 255), -1)
            
            degree = self.calculate_degree(frame, midpoint)
            self.vision_data_updated.emit({'detected': True, 'degree': degree})
        else:
            # Jika objek tidak terdeteksi, kirim sinyal juga
            self.vision_data_updated.emit({'detected': False, 'degree': None})

        return annotator.result()

    def calculate_degree(self, frame, midpoint):
        """Menggambar bilah dan mengembalikan nilai derajat."""
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

    def closeEvent(self, event):
        self.stop_camera()
        super().closeEvent(event)
