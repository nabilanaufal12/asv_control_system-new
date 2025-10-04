# src/navantara_gui/components/video_view.py
import cv2
import numpy as np
import os
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
    Widget yang menampilkan dua feed video dan mengontrol HANYA
    pengiriman stream ke GUI, bukan mematikan kamera di backend.
    """
    # Sinyal untuk memberitahu MainWindow untuk memulai/menghentikan PENGIRIMAN stream
    toggle_camera_requested = Signal(bool)
    inversion_changed = Signal(bool)

    def __init__(self, config, parent=None):
        super().__init__(parent)

        self.is_camera_running = False # Awalnya, GUI tidak menerima stream

        # --- PERUBAHAN TEKS 1: Tombol sekarang mengontrol "Stream" ---
        self.start_stop_button = QPushButton("Start Stream")
        self.invert_button = QPushButton("Invert Logic")
        self.invert_button.setCheckable(True)
        self.invert_button.setEnabled(False)

        # Coba muat ikon
        try:
            assets_path = os.path.join(os.path.dirname(__file__), "..", "assets")
            self.play_icon = QIcon(os.path.join(assets_path, "play-circle.svg"))
            self.pause_icon = QIcon(os.path.join(assets_path, "pause-circle.svg"))
            self.start_stop_button.setIcon(self.play_icon)
        except Exception as e:
            print(f"Peringatan: Gagal memuat ikon play/pause. Error: {e}")

        # Tata letak tombol
        control_layout = QHBoxLayout()
        control_layout.addStretch()
        control_layout.addWidget(self.invert_button)
        control_layout.addWidget(self.start_stop_button)

        # --- PERUBAHAN TEKS 2: Label video sekarang lebih deskriptif ---
        self.label_video_1 = QLabel(
            "Stream nonaktif.\nTekan 'Start Stream' untuk menampilkan video."
        )
        self.label_video_1.setAlignment(Qt.AlignCenter)
        self.label_video_1.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.label_video_1.setStyleSheet(
            "background-color: black; color: white; font-size: 16px;"
        )

        self.label_video_2 = QLabel("Stream nonaktif.")
        self.label_video_2.setAlignment(Qt.AlignCenter)
        self.label_video_2.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.label_video_2.setStyleSheet(
            "background-color: black; color: white; font-size: 16px;"
        )

        # Splitter untuk memisahkan dua video
        video_splitter = QSplitter(Qt.Horizontal)
        video_splitter.addWidget(self.label_video_1)
        video_splitter.addWidget(self.label_video_2)

        # Tata letak utama
        layout_utama = QVBoxLayout(self)
        layout_utama.addLayout(control_layout)
        layout_utama.addWidget(video_splitter, 1)
        self.setLayout(layout_utama)

        # Hubungkan sinyal tombol
        self.start_stop_button.clicked.connect(self.toggle_camera_stream)
        self.invert_button.clicked.connect(self.on_inversion_toggled)

    @Slot(np.ndarray)
    def update_frame_1(self, frame):
        self._display_frame(frame, self.label_video_1)

    @Slot(np.ndarray)
    def update_frame_2(self, frame):
        self._display_frame(frame, self.label_video_2)

    def _display_frame(self, frame, label_widget):
        try:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(
                rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888
            )
            pixmap = QPixmap.fromImage(qt_image)
            label_widget.setPixmap(
                pixmap.scaled(
                    label_widget.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
            )
        except Exception as e:
            print(f"GUI Error: Gagal menampilkan frame: {e}")

    def toggle_camera_stream(self):
        """Mengubah status streaming dan memancarkan sinyal ke MainWindow."""
        self.is_camera_running = not self.is_camera_running
        self.toggle_camera_requested.emit(self.is_camera_running)
        self.update_ui_controls(self.is_camera_running)

        # --- PERUBAHAN TEKS 3: Pesan saat stream dijeda ---
        if not self.is_camera_running:
            self.label_video_1.setText("Stream dijeda.\nBackend tetap memproses gambar.")
            self.label_video_1.setPixmap(QPixmap()) # Hapus gambar terakhir
            self.label_video_2.setText("Stream dijeda.")
            self.label_video_2.setPixmap(QPixmap())

    def on_inversion_toggled(self):
        self.inversion_changed.emit(self.invert_button.isChecked())

    def update_ui_controls(self, is_running):
        """Memperbarui tampilan tombol berdasarkan status streaming."""
        # --- PERUBAHAN TEKS 4: Teks tombol diubah menjadi "Stream" ---
        self.start_stop_button.setText(
            "Stop Stream" if is_running else "Start Stream"
        )
        self.start_stop_button.setIcon(
            self.pause_icon if is_running else self.play_icon
        )
        self.invert_button.setEnabled(is_running)