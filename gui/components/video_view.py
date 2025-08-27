# gui/components/video_view.py
# --- VERSI FINAL: Logika tombol yang disederhanakan untuk kontrol manual ---

import cv2
import numpy as np
from PySide6.QtWidgets import (
    QWidget,
    QLabel,
    QVBoxLayout,
    QPushButton,
    QHBoxLayout,
    QSizePolicy,
    QComboBox,
)
from PySide6.QtCore import Signal, Slot, Qt
from PySide6.QtGui import QImage, QPixmap


class VideoView(QWidget):
    """
    Widget yang menampilkan feed video dan menyediakan kontrol kamera
    yang terintegrasi langsung dengan VisionService.
    """

    # Sinyal untuk mengirim perintah ke MainWindow
    camera_changed = Signal(int)
    toggle_camera_requested = Signal(bool)  # True untuk start, False untuk stop
    inversion_changed = Signal(bool)

    def __init__(self, config, parent=None):
        super().__init__(parent)

        self.is_camera_running = False

        # --- UI untuk Kontrol Kamera ---
        self.camera_selector = QComboBox()
        self.refresh_button = QPushButton("Refresh List")
        self.start_stop_button = QPushButton("Start Camera")
        self.invert_button = QPushButton("Invert Logic")
        self.invert_button.setCheckable(True)
        self.invert_button.setEnabled(False)  # Diaktifkan saat kamera berjalan

        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel("Sumber Kamera:"))
        control_layout.addWidget(self.camera_selector, 1)
        control_layout.addWidget(self.refresh_button)
        control_layout.addWidget(self.start_stop_button)
        control_layout.addWidget(self.invert_button)

        # --- UI untuk Tampilan Video ---
        self.label_video = QLabel("Kamera nonaktif. Pilih sumber dan tekan 'Start'.")
        self.label_video.setAlignment(Qt.AlignCenter)
        self.label_video.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.label_video.setMinimumSize(640, 480)
        self.label_video.setStyleSheet(
            "background-color: black; color: white; font-size: 16px;"
        )

        # --- Tata Letak Utama ---
        layout_utama = QVBoxLayout(self)
        layout_utama.addLayout(control_layout)
        layout_utama.addWidget(self.label_video, 1)
        self.setLayout(layout_utama)

        # Hubungkan sinyal dari tombol ke metode internal
        self.refresh_button.clicked.connect(self.list_available_cameras)
        self.start_stop_button.clicked.connect(self.toggle_camera_stream)
        self.invert_button.clicked.connect(self.on_inversion_toggled)
        self.camera_selector.currentIndexChanged.connect(
            self.on_camera_selection_changed
        )

        # Pindai kamera saat pertama kali dijalankan
        self.list_available_cameras()

    @Slot(np.ndarray)
    def update_frame(self, frame):
        """Slot untuk menerima, memperbaiki warna, dan menampilkan frame baru."""
        try:
            # Konversi warna dari BGR (OpenCV) ke RGB (Qt)
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(
                rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888
            )

            pixmap = QPixmap.fromImage(qt_image)
            self.label_video.setPixmap(
                pixmap.scaled(
                    self.label_video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
            )
        except Exception as e:
            print(f"GUI Error: Gagal menampilkan frame: {e}")

    def list_available_cameras(self):
        """Memindai dan menampilkan daftar kamera yang tersedia menggunakan DSHOW."""
        self.camera_selector.clear()
        indices = []
        for i in range(5):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                indices.append(i)
                cap.release()

        if indices:
            self.camera_selector.addItems([f"Kamera {idx}" for idx in indices])
            self.start_stop_button.setEnabled(True)
        else:
            self.camera_selector.addItem("Tidak ada kamera ditemukan")
            self.start_stop_button.setEnabled(False)

    def toggle_camera_stream(self):
        """Mengirim sinyal untuk memulai atau menghentikan stream dan mengubah UI."""
        if not self.is_camera_running:
            if "Kamera" in self.camera_selector.currentText():
                self.toggle_camera_requested.emit(True)
                self.toggle_ui_controls(True)
        else:
            self.toggle_camera_requested.emit(False)
            self.toggle_ui_controls(False)
            self.label_video.setText("Kamera dihentikan.")
            self.label_video.setPixmap(QPixmap())

    def on_camera_selection_changed(self, index):
        """Memberi tahu VisionService jika pilihan kamera berubah."""
        if index >= 0 and "Kamera" in self.camera_selector.currentText():
            try:
                cam_idx = int(self.camera_selector.currentText().split(" ")[1])
                self.camera_changed.emit(cam_idx)
            except (ValueError, IndexError):
                pass

    def on_inversion_toggled(self):
        """Mengirim status tombol 'Invert'."""
        is_checked = self.invert_button.isChecked()
        self.inversion_changed.emit(is_checked)
        self.invert_button.setStyleSheet(
            "background-color: #e74c3c; color: white;" if is_checked else ""
        )

    def toggle_ui_controls(self, is_running):
        """Mengaktifkan/menonaktifkan tombol berdasarkan status kamera."""
        self.is_camera_running = is_running
        self.start_stop_button.setText("Stop Camera" if is_running else "Start Camera")
        self.invert_button.setEnabled(is_running)
        self.camera_selector.setEnabled(not is_running)
        self.refresh_button.setEnabled(not is_running)
