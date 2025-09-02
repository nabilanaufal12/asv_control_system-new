# gui/components/video_view.py
# --- VERSI MODIFIKASI: Mendukung dua stream video berdampingan ---

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
    QComboBox,
    QSplitter,  # <-- Impor QSplitter
)
from PySide6.QtCore import Signal, Slot, Qt
from PySide6.QtGui import QImage, QPixmap, QIcon


class VideoView(QWidget):
    """
    Widget yang menampilkan DUA feed video secara berdampingan
    dan menyediakan kontrol kamera.
    """

    camera_changed = Signal(int)
    toggle_camera_requested = Signal(bool)
    inversion_changed = Signal(bool)

    def __init__(self, config, parent=None):
        super().__init__(parent)

        self.is_camera_running = False

        self.camera_selector = QComboBox()
        self.refresh_button = QPushButton("Refresh List")
        self.start_stop_button = QPushButton("Start Cameras")
        self.invert_button = QPushButton("Invert Logic")
        self.invert_button.setCheckable(True)
        self.invert_button.setEnabled(False)

        try:
            assets_path = os.path.join(os.path.dirname(__file__), "..", "assets")
            self.refresh_button.setIcon(
                QIcon(os.path.join(assets_path, "rotate-cw.svg"))
            )
        except Exception as e:
            print(f"Peringatan: Gagal memuat ikon refresh. Error: {e}")

        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel("Sumber Kamera Utama:"))
        control_layout.addWidget(self.camera_selector, 1)
        control_layout.addWidget(self.refresh_button)
        control_layout.addWidget(self.start_stop_button)
        control_layout.addWidget(self.invert_button)

        # --- PERUBAHAN 1: Buat dua QLabel untuk video ---
        self.label_video_1 = QLabel(
            "Kamera Atas Nonaktif.\nPilih sumber dan tekan 'Start'."
        )
        self.label_video_1.setAlignment(Qt.AlignCenter)
        self.label_video_1.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.label_video_1.setStyleSheet(
            "background-color: black; color: white; font-size: 16px;"
        )

        self.label_video_2 = QLabel("Kamera Bawah Nonaktif.")
        self.label_video_2.setAlignment(Qt.AlignCenter)
        self.label_video_2.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.label_video_2.setStyleSheet(
            "background-color: black; color: white; font-size: 16px;"
        )

        # --- PERUBAHAN 2: Gunakan QSplitter untuk tata letak berdampingan ---
        video_splitter = QSplitter(Qt.Horizontal)
        video_splitter.addWidget(self.label_video_1)
        video_splitter.addWidget(self.label_video_2)

        layout_utama = QVBoxLayout(self)
        layout_utama.addLayout(control_layout)
        layout_utama.addWidget(video_splitter, 1)  # Tambahkan splitter ke layout
        self.setLayout(layout_utama)

        self.refresh_button.clicked.connect(self.list_available_cameras)
        self.start_stop_button.clicked.connect(self.toggle_camera_stream)
        self.invert_button.clicked.connect(self.on_inversion_toggled)
        self.camera_selector.currentIndexChanged.connect(
            self.on_camera_selection_changed
        )
        self.list_available_cameras()

    # --- PERUBAHAN 3: Buat slot terpisah untuk setiap frame kamera ---
    @Slot(np.ndarray)
    def update_frame_1(self, frame):
        """Menerima dan menampilkan frame untuk kamera 1 (kiri)."""
        self._display_frame(frame, self.label_video_1)

    @Slot(np.ndarray)
    def update_frame_2(self, frame):
        """Menerima dan menampilkan frame untuk kamera 2 (kanan)."""
        self._display_frame(frame, self.label_video_2)

    def _display_frame(self, frame, label_widget):
        """Fungsi helper untuk mengubah numpy array ke QPixmap dan menampilkannya."""
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

    def list_available_cameras(self):
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
        if not self.is_camera_running:
            if "Kamera" in self.camera_selector.currentText():
                self.on_camera_selection_changed(self.camera_selector.currentIndex())
                self.toggle_camera_requested.emit(True)
                self.toggle_ui_controls(True)
        else:
            self.toggle_camera_requested.emit(False)
            self.toggle_ui_controls(False)
            # Reset kedua label
            self.label_video_1.setText("Kamera dihentikan.")
            self.label_video_1.setPixmap(QPixmap())
            self.label_video_2.setText("Kamera dihentikan.")
            self.label_video_2.setPixmap(QPixmap())

    def on_camera_selection_changed(self, index):
        if index >= 0 and "Kamera" in self.camera_selector.currentText():
            try:
                cam_idx = int(self.camera_selector.currentText().split(" ")[1])
                self.camera_changed.emit(cam_idx)
            except (ValueError, IndexError):
                pass

    def on_inversion_toggled(self):
        is_checked = self.invert_button.isChecked()
        self.inversion_changed.emit(is_checked)
        self.invert_button.setProperty("inverted", is_checked)
        self.style().polish(self.invert_button)

    def toggle_ui_controls(self, is_running):
        self.is_camera_running = is_running
        self.start_stop_button.setText(
            "Stop Cameras" if is_running else "Start Cameras"
        )
        self.invert_button.setEnabled(is_running)
        self.camera_selector.setEnabled(not is_running)
        self.refresh_button.setEnabled(not is_running)
