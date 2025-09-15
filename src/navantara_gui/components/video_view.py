# src/navantara_gui/components/video_view.py
import cv2
import numpy as np
import os
from PySide6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout, QSizePolicy, QSplitter
)
from PySide6.QtCore import Signal, Slot, Qt
from PySide6.QtGui import QImage, QPixmap, QIcon

class VideoView(QWidget):
    """
    Widget yang menampilkan DUA feed video dari backend secara berdampingan
    dan menyediakan satu tombol untuk mengontrol stream.
    """
    # Sinyal untuk memberi tahu MainWindow bahwa pengguna ingin memulai/menghentikan stream
    toggle_camera_requested = Signal(bool)
    inversion_changed = Signal(bool)

    def __init__(self, config, parent=None):
        super().__init__(parent)

        self.is_camera_running = False

        # --- PERUBAHAN UTAMA: Hapus ComboBox dan tombol Refresh ---
        # Hanya ada tombol untuk memulai/menghentikan dan inversi
        self.start_stop_button = QPushButton("Start Cameras")
        self.invert_button = QPushButton("Invert Logic")
        self.invert_button.setCheckable(True)
        self.invert_button.setEnabled(False)

        # Coba muat ikon untuk tombol agar lebih intuitif
        try:
            assets_path = os.path.join(os.path.dirname(__file__), "..", "assets")
            self.play_icon = QIcon(os.path.join(assets_path, "play-circle.svg"))
            self.pause_icon = QIcon(os.path.join(assets_path, "pause-circle.svg"))
            self.start_stop_button.setIcon(self.play_icon)
        except Exception as e:
            print(f"Peringatan: Gagal memuat ikon play/pause. Error: {e}")

        # Tata letak untuk tombol kontrol di pojok kanan atas
        control_layout = QHBoxLayout()
        control_layout.addStretch() 
        control_layout.addWidget(self.invert_button)
        control_layout.addWidget(self.start_stop_button)

        # Label untuk menampilkan video
        self.label_video_1 = QLabel("Kamera Atas Nonaktif.\nTekan 'Start Cameras' untuk memulai stream.")
        self.label_video_1.setAlignment(Qt.AlignCenter)
        self.label_video_1.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.label_video_1.setStyleSheet("background-color: black; color: white; font-size: 16px;")

        self.label_video_2 = QLabel("Kamera Bawah Nonaktif.")
        self.label_video_2.setAlignment(Qt.AlignCenter)
        self.label_video_2.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.label_video_2.setStyleSheet("background-color: black; color: white; font-size: 16px;")

        # Splitter untuk memisahkan dua tampilan video
        video_splitter = QSplitter(Qt.Horizontal)
        video_splitter.addWidget(self.label_video_1)
        video_splitter.addWidget(self.label_video_2)

        # Tata letak utama untuk seluruh widget
        layout_utama = QVBoxLayout(self)
        layout_utama.addLayout(control_layout)
        layout_utama.addWidget(video_splitter, 1)
        self.setLayout(layout_utama)

        # Hubungkan sinyal dari tombol ke fungsi internal
        self.start_stop_button.clicked.connect(self.toggle_camera_stream)
        self.invert_button.clicked.connect(self.on_inversion_toggled)

    @Slot(np.ndarray)
    def update_frame_1(self, frame):
        """Menerima frame dari ApiClient dan menampilkannya di label pertama."""
        self._display_frame(frame, self.label_video_1)

    @Slot(np.ndarray)
    def update_frame_2(self, frame):
        """Menerima frame dari ApiClient dan menampilkannya di label kedua."""
        self._display_frame(frame, self.label_video_2)

    def _display_frame(self, frame, label_widget):
        """Helper untuk mengubah frame OpenCV (numpy) ke format Qt (QPixmap)."""
        try:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            label_widget.setPixmap(
                pixmap.scaled(label_widget.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        except Exception as e:
            print(f"GUI Error: Gagal menampilkan frame: {e}")

    # --- FUNGSI list_available_cameras() DIHAPUS TOTAL ---

    def toggle_camera_stream(self):
        """Mengubah status streaming dan memancarkan sinyal ke MainWindow."""
        self.is_camera_running = not self.is_camera_running
        self.toggle_camera_requested.emit(self.is_camera_running)
        self.toggle_ui_controls(self.is_camera_running)
        
        # Jika stream dihentikan, bersihkan tampilan video
        if not self.is_camera_running:
            self.label_video_1.setText("Stream dihentikan.")
            self.label_video_1.setPixmap(QPixmap())
            self.label_video_2.setText("Stream dihentikan.")
            self.label_video_2.setPixmap(QPixmap())

    def on_inversion_toggled(self):
        """Memancarkan sinyal saat tombol inversi ditekan."""
        self.inversion_changed.emit(self.invert_button.isChecked())

    def toggle_ui_controls(self, is_running):
        """Memperbarui tampilan tombol berdasarkan status streaming."""
        self.start_stop_button.setText("Stop Cameras" if is_running else "Start Cameras")
        self.start_stop_button.setIcon(self.pause_icon if is_running else self.play_icon)
        self.invert_button.setEnabled(is_running)