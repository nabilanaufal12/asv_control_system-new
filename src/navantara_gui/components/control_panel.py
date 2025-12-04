# src/navantara_gui/components/control_panel.py
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,  # Tambahkan QGridLayout
    QPushButton,
    QGroupBox,
    QButtonGroup,
    QLabel,  # Tambahkan QLabel
)
from PySide6.QtCore import Signal
from PySide6.QtGui import QIcon
import os


class ControlPanel(QWidget):
    manual_button_clicked = Signal()
    auto_button_clicked = Signal()

    # [MODIFIKASI] Sinyal yang lebih spesifik
    surface_overlay_clicked = Signal()
    surface_raw_clicked = Signal()
    underwater_overlay_clicked = Signal()
    underwater_raw_clicked = Signal()

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.key_buttons = {}
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # --- Mode Selection ---
        mode_group = QGroupBox("Vehicle Control")
        mode_layout = QHBoxLayout()

        self.mode_btn_group = QButtonGroup(self)
        self.mode_btn_group.setExclusive(True)

        self.manual_mode_btn = QPushButton("MANUAL")
        self.manual_mode_btn.setCheckable(True)
        self.manual_mode_btn.setChecked(False)

        self.auto_mode_btn = QPushButton("AUTO")
        self.auto_mode_btn.setCheckable(True)
        self.auto_mode_btn.setChecked(True)

        self.manual_mode_btn.setStyleSheet(
            "QPushButton:checked { background-color: #e67e22; color: white; font-weight: bold; } QPushButton { padding: 10px; }"
        )
        self.auto_mode_btn.setStyleSheet(
            "QPushButton:checked { background-color: #2ecc71; color: white; font-weight: bold; } QPushButton { padding: 10px; }"
        )

        self.mode_btn_group.addButton(self.manual_mode_btn)
        self.mode_btn_group.addButton(self.auto_mode_btn)

        mode_layout.addWidget(self.manual_mode_btn)
        mode_layout.addWidget(self.auto_mode_btn)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        # --- Manual Controls (WASD) ---
        wasd_group = QGroupBox("Manual Drive")
        wasd_layout = QVBoxLayout()
        row_w = QHBoxLayout()
        self.btn_w = self.create_key_button("W")
        row_w.addStretch()
        row_w.addWidget(self.btn_w)
        row_w.addStretch()
        row_asd = QHBoxLayout()
        self.btn_a = self.create_key_button("A")
        self.btn_s = self.create_key_button("S")
        self.btn_d = self.create_key_button("D")
        row_asd.addWidget(self.btn_a)
        row_asd.addWidget(self.btn_s)
        row_asd.addWidget(self.btn_d)
        wasd_layout.addLayout(row_w)
        wasd_layout.addLayout(row_asd)
        wasd_group.setLayout(wasd_layout)
        layout.addWidget(wasd_group)

        self.key_buttons["W"] = self.btn_w
        self.key_buttons["A"] = self.btn_a
        self.key_buttons["S"] = self.btn_s
        self.key_buttons["D"] = self.btn_d

        # --- Mission Control ---
        mission_group = QGroupBox("Mission Control")
        mission_layout = QVBoxLayout()
        self.start_mission_btn = QPushButton(" Start Mission")
        self.start_mission_btn.setIcon(QIcon(self._get_icon_path("play-circle.svg")))
        self.start_mission_btn.setStyleSheet(
            "background-color: #27ae60; color: white; padding: 8px;"
        )
        self.pause_mission_btn = QPushButton(" Pause / Hold")
        self.pause_mission_btn.setIcon(QIcon(self._get_icon_path("pause-circle.svg")))
        self.pause_mission_btn.setStyleSheet(
            "background-color: #f1c40f; color: black; padding: 8px;"
        )
        self.return_home_btn = QPushButton(" Return to Home")
        self.return_home_btn.setStyleSheet(
            "background-color: #c0392b; color: white; padding: 8px;"
        )

        mission_layout.addWidget(self.start_mission_btn)
        mission_layout.addWidget(self.pause_mission_btn)
        mission_layout.addWidget(self.return_home_btn)
        mission_group.setLayout(mission_layout)
        layout.addWidget(mission_group)

        # --- [MODIFIKASI UTAMA] Camera Capture Controls ---
        capture_group = QGroupBox("Camera Capture")
        # Menggunakan Grid Layout untuk 4 tombol
        capture_layout = QGridLayout()

        # Label Header
        capture_layout.addWidget(QLabel("<b>Surface (CAM1):</b>"), 0, 0, 1, 2)

        # Tombol Surface
        self.btn_surf_overlay = QPushButton("Overlay")
        self.btn_surf_overlay.setStyleSheet(
            "background-color: #3498db; color: white; padding: 5px;"
        )
        self.btn_surf_raw = QPushButton("RAW")
        self.btn_surf_raw.setStyleSheet(
            "background-color: #95a5a6; color: white; padding: 5px;"
        )

        capture_layout.addWidget(self.btn_surf_overlay, 1, 0)
        capture_layout.addWidget(self.btn_surf_raw, 1, 1)

        # Spacer/Label Header Underwater
        capture_layout.addWidget(QLabel("<b>Underwater (CAM2):</b>"), 2, 0, 1, 2)

        # Tombol Underwater
        self.btn_under_overlay = QPushButton("Overlay")
        self.btn_under_overlay.setStyleSheet(
            "background-color: #3498db; color: white; padding: 5px;"
        )
        self.btn_under_raw = QPushButton("RAW")
        self.btn_under_raw.setStyleSheet(
            "background-color: #95a5a6; color: white; padding: 5px;"
        )

        capture_layout.addWidget(self.btn_under_overlay, 3, 0)
        capture_layout.addWidget(self.btn_under_raw, 3, 1)

        capture_group.setLayout(capture_layout)
        layout.addWidget(capture_group)
        # --------------------------------------------------

        layout.addStretch()

        # Koneksi Sinyal
        self.manual_mode_btn.clicked.connect(self.manual_button_clicked.emit)
        self.auto_mode_btn.clicked.connect(self.auto_button_clicked.emit)

        # [MODIFIKASI] Koneksi tombol baru
        self.btn_surf_overlay.clicked.connect(self.surface_overlay_clicked.emit)
        self.btn_surf_raw.clicked.connect(self.surface_raw_clicked.emit)
        self.btn_under_overlay.clicked.connect(self.underwater_overlay_clicked.emit)
        self.btn_under_raw.clicked.connect(self.underwater_raw_clicked.emit)

    def create_key_button(self, text):
        btn = QPushButton(text)
        btn.setFixedSize(50, 50)
        btn.setStyleSheet(
            """
            QPushButton {
                background-color: #ecf0f1;
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:pressed {
                background-color: #3498db;
                color: white;
                border-color: #2980b9;
            }
        """
        )
        return btn

    def update_key_press_status(self, key_char, is_pressed):
        btn = self.key_buttons.get(key_char)
        if btn:
            if is_pressed:
                btn.setStyleSheet(
                    "background-color: #3498db; color: white; border: 2px solid #2980b9; border-radius: 5px; font-weight: bold; font-size: 16px;"
                )
            else:
                btn.setStyleSheet(
                    "background-color: #ecf0f1; border: 2px solid #bdc3c7; border-radius: 5px; font-weight: bold; font-size: 16px;"
                )

    def _get_icon_path(self, filename):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "..", "assets", filename)
