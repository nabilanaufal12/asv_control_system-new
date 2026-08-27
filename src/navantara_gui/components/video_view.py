# src/navantara_gui/components/video_view.py
import cv2
import numpy as np
import base64
from PySide6.QtWidgets import (
    QWidget,
    QLabel,
    QVBoxLayout,
    QPushButton,
    QSizePolicy,
    QSplitter,
)
from PySide6.QtCore import Signal, Slot, Qt
from PySide6.QtGui import QImage, QPixmap


class VideoView(QWidget):
    """
    Widget video yang 'Smart': Bisa menerima data berupa Base64 String,
    Raw Bytes, ataupun NumPy Array (OpenCV Image).
    """

    toggle_camera_requested = Signal(bool)
    inversion_changed = Signal(bool)

    def __init__(self, config, parent=None):
        super().__init__(parent)

        self.is_camera_running = True

        # --- UI SETUP ---
        self.start_stop_button = QPushButton("Stop Stream")
        self.start_stop_button.setStyleSheet(
            "background-color: #F44336; color: white; font-weight: bold; padding: 6px 14px; border-radius: 4px;"
        )

        self.invert_button = QPushButton("Swap Camera (Surface ↔ UW)")
        self.invert_button.setCheckable(True)
        self.invert_button.setEnabled(True)
        self.invert_button.setStyleSheet(
            "background-color: #607D8B; color: white; font-weight: bold; padding: 6px 14px; border-radius: 4px;"
        )

        self.label_video_1 = QLabel("SURFACE CAMERA (CAM 1)\n\n[ OFFLINE / NO SIGNAL ]")
        self._style_label(self.label_video_1)

        self.label_video_2 = QLabel(
            "UNDERWATER CAMERA (CAM 2)\n\n[ OFFLINE / NO SIGNAL ]"
        )
        self._style_label(self.label_video_2)

        self.video_splitter = QSplitter(Qt.Horizontal)
        self.video_splitter.addWidget(self.label_video_1)
        self.video_splitter.addWidget(self.label_video_2)
        self.video_splitter.setChildrenCollapsible(False)

        # Paksa rasio splitter agar 50:50 merata
        self.video_splitter.setStretchFactor(0, 1)
        self.video_splitter.setStretchFactor(1, 1)
        self.video_splitter.setSizes([1000, 1000])

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.video_splitter, 1)
        self.setLayout(layout)

        self.start_stop_button.clicked.connect(self.toggle_camera_stream)
        self.invert_button.clicked.connect(self.on_inversion_toggled)

    def _style_label(self, label):
        label.setAlignment(Qt.AlignCenter)
        label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        label.setMinimumSize(1, 1)
        label.setStyleSheet(
            "background-color: #000000;"
            "color: #e74c3c;"
            "font-family: 'Courier New', Courier, monospace;"
            "font-size: 15px;"
            "font-weight: bold;"
            "border: 1px solid #333333;"
            "border-radius: 4px;"
        )

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
        if not self.is_camera_running or frame_data is None:
            return

        try:
            q_img = None

            # --- KASUS 1: Data adalah NumPy Array (OpenCV Image) ---
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
                target_size = label_widget.size()
                if target_size.width() > 10 and target_size.height() > 10:
                    scaled_pix = QPixmap.fromImage(q_img).scaled(
                        target_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
                    )
                    label_widget.setPixmap(scaled_pix)

        except Exception:
            pass

    def toggle_camera_stream(self):
        self.is_camera_running = not self.is_camera_running
        self.update_ui_controls(self.is_camera_running)
        self.toggle_camera_requested.emit(self.is_camera_running)

        if not self.is_camera_running:
            self.label_video_1.setText("SURFACE CAMERA (CAM 1)\n\n[ STREAM DIJEDA ]")
            self.label_video_1.setPixmap(QPixmap())
            self.label_video_2.setText("UNDERWATER CAMERA (CAM 2)\n\n[ STREAM DIJEDA ]")
            self.label_video_2.setPixmap(QPixmap())

    def on_inversion_toggled(self):
        self.inversion_changed.emit(self.invert_button.isChecked())

    def update_ui_controls(self, is_running):
        if is_running:
            self.start_stop_button.setText("Stop Stream")
            self.start_stop_button.setStyleSheet(
                "background-color: #F44336; color: white; font-weight: bold; padding: 6px 14px; border-radius: 4px;"
            )
        else:
            self.start_stop_button.setText("Play Stream")
            self.start_stop_button.setStyleSheet(
                "background-color: #4CAF50; color: white; font-weight: bold; padding: 6px 14px; border-radius: 4px;"
            )
