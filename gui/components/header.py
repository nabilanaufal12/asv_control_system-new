# gui/components/header.py
# --- MODIFIKASI: Menambahkan status RTH dan konsistensi ---

import os
from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpacerItem,
    QSizePolicy,
)
from PySide6.QtCore import Slot, Qt
from PySide6.QtGui import QFont, QPixmap


class Header(QWidget):
    """
    Widget untuk header yang berisi logo, judul, dan indikator status.
    """

    def __init__(self, config):
        super().__init__()
        self.setObjectName("Header")

        self.config = config

        layout = QHBoxLayout()
        layout.setContentsMargins(10, 5, 10, 5)

        self.logo_label = QLabel()
        logo_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "assets", "logo_umrah.png")
        )

        if os.path.exists(logo_path):
            pixmap = QPixmap(logo_path)
            self.logo_label.setPixmap(
                pixmap.scaledToHeight(40, Qt.SmoothTransformation)
            )
        else:
            print(f"Peringatan: File logo tidak ditemukan di {logo_path}")
            self.logo_label.setText("[Logo]")

        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        self.title_label = QLabel("NAVANTARA - UMRAH")
        self.title_label.setFont(title_font)

        spacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.connection_status_label = QLabel("DISCONNECTED")
        self.connection_status_label.setObjectName("StatusLabel")

        self.gps_status_label = QLabel("GPS: 0 Sats")
        self.gps_status_label.setObjectName("StatusLabel")

        self.mode_label = QLabel("Manual Mode")
        self.mode_label.setObjectName("StatusLabel")

        self.theme_button = QPushButton("Switch to Light Mode")

        layout.addWidget(self.logo_label)
        layout.addWidget(self.title_label)
        layout.addSpacerItem(spacer)
        layout.addWidget(self.connection_status_label)
        layout.addWidget(self.gps_status_label)
        layout.addWidget(self.mode_label)
        layout.addWidget(self.theme_button)

        self.setLayout(layout)

        self.setStyleSheet(
            """
            #Header {
                background-color: #34495E;
                color: white;
            }
            #StatusLabel {
                padding: 5px 10px;
                border-radius: 5px;
                font-weight: bold;
                margin: 0 5px;
            }
        """
        )
        self.update_status({"status": "DISCONNECTED", "control_mode": "MANUAL"})

    @Slot(dict)
    def update_status(self, data):
        """Menerima data dan mengupdate label status."""
        status = data.get("status", "DISCONNECTED")
        self.connection_status_label.setText(status)

        # --- MODIFIKASI DIMULAI DI SINI ---
        if status == "DISCONNECTED":
            self.connection_status_label.setStyleSheet(
                "#StatusLabel { background-color: #E74C3C; color: white; }"
            )
        elif status == "RETURNING TO HOME":
            self.connection_status_label.setStyleSheet(
                "#StatusLabel { background-color: #3498db; color: white; }" # Warna biru untuk RTH
            )
        else: # Mencakup CONNECTED, NAVIGATING, IDLE, dll.
            self.connection_status_label.setStyleSheet(
                "#StatusLabel { background-color: #2ECC71; color: white; }"
            )
        # --- AKHIR MODIFIKASI ---

        mode = data.get("control_mode", "MANUAL")
        if mode == "AUTO":
            self.mode_label.setText("Auto Mode")
            self.mode_label.setStyleSheet(
                "#StatusLabel { background-color: #27ae60; color: white; }"
            )
        else:
            self.mode_label.setText("Manual Mode")
            self.mode_label.setStyleSheet(
                "#StatusLabel { background-color: #f39c12; color: white; }"
            )