# gui/components/camera/video_view.py
# --- PERBAIKAN: Menambahkan impor 'numpy' yang hilang ---

import os
from pathlib import Path
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout, QComboBox
from PySide6.QtCore import Signal, Slot, Qt
from PySide6.QtGui import QImage, QPixmap
import cv2
import numpy as np # <-- PENAMBAHAN BARIS INI

# Impor kelas DetectionThread dari file terpisah di folder yang sama
from .detection_thread import DetectionThread, YOLO_AVAILABLE

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
        else:
            self.list_cameras()
            
    @Slot(dict)
    def update_telemetry_in_thread(self, data):
        """Meneruskan data telemetri ke thread deteksi jika sedang berjalan."""
        if self.detection_thread and self.detection_thread.isRunning():
            self.detection_thread.update_telemetry(data)
            
    def start_camera(self):
        if self.detection_thread and self.detection_thread.isRunning(): return
        if "Kamera" not in self.camera_selector.currentText(): return
        
        # Path ke file bobot sekarang ditentukan dari lokasi file ini
        CURRENT_FILE_DIR = Path(os.path.abspath(__file__)).resolve()
        PROJECT_ROOT = CURRENT_FILE_DIR.parents[3] # Naik 3 tingkat dari camera/
        weights_path = str(PROJECT_ROOT / "yolov5" / "besto.pt")
        
        if not os.path.exists(weights_path):
            self.label.setText(f"Error: File bobot 'besto.pt' tidak ditemukan.")
            return
            
        source_idx = int(self.camera_selector.currentText().split(" ")[1])
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