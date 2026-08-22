# src/navantara_gui/components/control_panel.py
from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QPushButton,
    QGroupBox,
    QLabel,
)
from PySide6.QtCore import Signal
import os


class ControlPanel(QWidget):
    surface_overlay_clicked = Signal()
    surface_raw_clicked = Signal()
    underwater_overlay_clicked = Signal()
    underwater_raw_clicked = Signal()

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.setup_ui()

    def setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)

        capture_group = QGroupBox("Photo Capture Configuration")
        capture_layout = QHBoxLayout()

        # Surface CAM1 Controls
        surf_layout = QHBoxLayout()
        surf_layout.addWidget(QLabel("<b>Surface (CAM1):</b>"))
        self.btn_surf_overlay = QPushButton("Overlay")
        self.btn_surf_overlay.setStyleSheet(
            "background-color: #00BCD4; color: white; font-weight: bold; padding: 5px 10px;"
        )
        self.btn_surf_raw = QPushButton("RAW")
        self.btn_surf_raw.setStyleSheet(
            "background-color: #9E9E9E; color: white; font-weight: bold; padding: 5px 10px;"
        )
        surf_layout.addWidget(self.btn_surf_overlay)
        surf_layout.addWidget(self.btn_surf_raw)
        capture_layout.addLayout(surf_layout)

        capture_layout.addSpacing(15)

        # Underwater CAM2 Controls
        under_layout = QHBoxLayout()
        under_layout.addWidget(QLabel("<b>Underwater (CAM2):</b>"))
        self.btn_under_overlay = QPushButton("Overlay")
        self.btn_under_overlay.setStyleSheet(
            "background-color: #00BCD4; color: white; font-weight: bold; padding: 5px 10px;"
        )
        self.btn_under_raw = QPushButton("RAW")
        self.btn_under_raw.setStyleSheet(
            "background-color: #9E9E9E; color: white; font-weight: bold; padding: 5px 10px;"
        )
        under_layout.addWidget(self.btn_under_overlay)
        under_layout.addWidget(self.btn_under_raw)
        capture_layout.addLayout(under_layout)

        capture_group.setLayout(capture_layout)
        main_layout.addWidget(capture_group)

        # Koneksi tombol
        self.btn_surf_overlay.clicked.connect(self.surface_overlay_clicked.emit)
        self.btn_surf_raw.clicked.connect(self.surface_raw_clicked.emit)
        self.btn_under_overlay.clicked.connect(self.underwater_overlay_clicked.emit)
        self.btn_under_raw.clicked.connect(self.underwater_raw_clicked.emit)

    def _get_icon_path(self, filename):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "..", "assets", filename)
