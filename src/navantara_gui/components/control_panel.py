# src/navantara_gui/components/control_panel.py
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QGridLayout,
    QPushButton,
    QGroupBox,
    QLabel,
)
from PySide6.QtCore import Signal
import os


class ControlPanel(QWidget):
    # [MODIFIKASI] Sinyal yang lebih spesifik
    surface_overlay_clicked = Signal()
    surface_raw_clicked = Signal()
    underwater_overlay_clicked = Signal()
    underwater_raw_clicked = Signal()

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # --- [MODIFIKASI UTAMA] Camera Capture Controls ---
        capture_group = QGroupBox("Photo Capture Configuration")
        # Menggunakan Grid Layout untuk 4 tombol
        capture_layout = QGridLayout()

        # Label Header
        capture_layout.addWidget(QLabel("<b>Surface (CAM1):</b>"), 0, 0, 1, 2)

        # Tombol Surface
        self.btn_surf_overlay = QPushButton("Overlay")
        self.btn_surf_overlay.setStyleSheet(
            "background-color: #00BCD4; color: white; font-weight: bold; padding: 5px;"
        )
        self.btn_surf_raw = QPushButton("RAW")
        self.btn_surf_raw.setStyleSheet(
            "background-color: #9E9E9E; color: white; font-weight: bold; padding: 5px;"
        )

        capture_layout.addWidget(self.btn_surf_overlay, 1, 0)
        capture_layout.addWidget(self.btn_surf_raw, 1, 1)

        # Spacer/Label Header Underwater
        capture_layout.addWidget(QLabel("<b>Underwater (CAM2):</b>"), 2, 0, 1, 2)

        # Tombol Underwater
        self.btn_under_overlay = QPushButton("Overlay")
        self.btn_under_overlay.setStyleSheet(
            "background-color: #00BCD4; color: white; font-weight: bold; padding: 5px;"
        )
        self.btn_under_raw = QPushButton("RAW")
        self.btn_under_raw.setStyleSheet(
            "background-color: #9E9E9E; color: white; font-weight: bold; padding: 5px;"
        )

        capture_layout.addWidget(self.btn_under_overlay, 3, 0)
        capture_layout.addWidget(self.btn_under_raw, 3, 1)

        capture_group.setLayout(capture_layout)
        layout.addWidget(capture_group)
        # --------------------------------------------------

        # [MODIFIKASI] Koneksi tombol baru
        self.btn_surf_overlay.clicked.connect(self.surface_overlay_clicked.emit)
        self.btn_surf_raw.clicked.connect(self.surface_raw_clicked.emit)
        self.btn_under_overlay.clicked.connect(self.underwater_overlay_clicked.emit)
        self.btn_under_raw.clicked.connect(self.underwater_raw_clicked.emit)

    def _get_icon_path(self, filename):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "..", "assets", filename)
