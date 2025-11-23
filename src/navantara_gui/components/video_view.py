# src/navantara_gui/components/video_view.py
import cv2
import numpy as np
import os
import base64
from PySide6.QtWidgets import (
    QWidget,
    QLabel,
    QVBoxLayout,
    QPushButton,
    QHBoxLayout,
    QSizePolicy,
    QSplitter,
)
from PySide6.QtCore import Signal, Slot, Qt
from PySide6.QtGui import QImage, QPixmap, QIcon


class VideoView(QWidget):
    """
    Widget video yang 'Smart': Bisa menerima data berupa Base64 String,
    Raw Bytes, ataupun NumPy Array (OpenCV Image).
    """

    toggle_camera_requested = Signal(bool)
    inversion_changed = Signal(bool)

    def __init__(self, config, parent=None):
        super().__init__(parent)

        self.is_camera_running = False

        # --- UI SETUP ---
        self.start_stop_button = QPushButton("Start Stream")
        self.invert_button = QPushButton("Invert Logic")
        self.invert_button.setCheckable(True)
        self.invert_button.setEnabled(False)

        try:
            assets_path = os.path.join(os.path.dirname(__file__), "..", "assets")
            self.play_icon = QIcon(os.path.join(assets_path, "play-circle.svg"))
            self.pause_icon = QIcon(os.path.join(assets_path, "pause-circle.svg"))
            self.start_stop_button.setIcon(self.play_icon)
        except Exception:
            pass

        control_layout = QHBoxLayout()
        control_layout.addStretch()
        control_layout.addWidget(self.invert_button)
        control_layout.addWidget(self.start_stop_button)

        self.label_video_1 = QLabel("Stream nonaktif.")
        self._style_label(self.label_video_1)

        self.label_video_2 = QLabel("Stream nonaktif.")
        self._style_label(self.label_video_2)

        video_splitter = QSplitter(Qt.Horizontal)
        video_splitter.addWidget(self.label_video_1)
        video_splitter.addWidget(self.label_video_2)

        layout = QVBoxLayout(self)
        layout.addLayout(control_layout)
        layout.addWidget(video_splitter, 1)
        self.setLayout(layout)

        self.start_stop_button.clicked.connect(self.toggle_camera_stream)
        self.invert_button.clicked.connect(self.on_inversion_toggled)

    def _style_label(self, label):
        label.setAlignment(Qt.AlignCenter)
        label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        label.setStyleSheet("background-color: black; color: white; font-size: 16px;")

    @Slot(object)
    def update_frame_1(self, frame_data):
        self._display_frame(frame_data, self.label_video_1)

    @Slot(object)
    def update_frame_2(self, frame_data):
        self._display_frame(frame_data, self.label_video_2)

    def _display_frame(self, frame_data, label_widget):
        """
        Fungsi pintar untuk menampilkan frame dari berbagai format data.
        """
        if frame_data is None:
            return

        try:
            q_img = None

            # --- KASUS 1: Data adalah NumPy Array (OpenCV Image) ---
            # Ini yang dikirim oleh api_client.py Anda sekarang
            if isinstance(frame_data, np.ndarray):
                if frame_data.size == 0:
                    return

                # Convert BGR (OpenCV) ke RGB (Qt)
                rgb_image = cv2.cvtColor(frame_data, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w

                # Buat QImage dari data pixel raw
                q_img = QImage(
                    rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888
                )

            # --- KASUS 2: Data adalah String (Base64) ---
            elif isinstance(frame_data, str):
                img_bytes = base64.b64decode(frame_data)
                q_img = QImage.fromData(img_bytes)

            # --- KASUS 3: Data adalah Bytes (Raw JPEG) ---
            elif isinstance(frame_data, (bytes, bytearray)):
                q_img = QImage.fromData(frame_data)

            # --- TAMPILKAN KE LABEL ---
            if q_img and not q_img.isNull():
                # Scaling gambar agar pas di label tanpa merusak rasio
                pixmap = QPixmap.fromImage(q_img)
                scaled_pix = pixmap.scaled(
                    label_widget.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
                label_widget.setPixmap(scaled_pix)

        except Exception:
            # Supaya log tidak penuh spam error, kita print sekali saja jika perlu
            # print(f"GUI Error (_display_frame): {e}")
            pass

    def toggle_camera_stream(self):
        self.is_camera_running = not self.is_camera_running
        self.toggle_camera_requested.emit(self.is_camera_running)
        self.update_ui_controls(self.is_camera_running)

        if not self.is_camera_running:
            self.label_video_1.setText("Stream dijeda.")
            self.label_video_1.setPixmap(QPixmap())
            self.label_video_2.setText("Stream dijeda.")
            self.label_video_2.setPixmap(QPixmap())

    def on_inversion_toggled(self):
        self.inversion_changed.emit(self.invert_button.isChecked())

    def update_ui_controls(self, is_running):
        self.start_stop_button.setText("Stop Stream" if is_running else "Start Stream")
        self.start_stop_button.setIcon(
            self.pause_icon if is_running else self.play_icon
        )
        self.invert_button.setEnabled(is_running)
