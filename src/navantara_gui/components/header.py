# src/navantara_gui/components/header.py
import os
from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpacerItem,
    QSizePolicy,
)
from PySide6.QtCore import Slot, Qt, Signal
from PySide6.QtGui import QFont, QPixmap


class Header(QWidget):
    """
    Widget untuk header yang berisi logo, judul, dan indikator status.
    """

    theme_changed_requested = Signal()

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
        self.title_label = QLabel("NAVANTARA ASV GROUND CONTROL STATION")
        self.title_label.setFont(title_font)
        spacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.theme_button = QPushButton("Switch to Dark Mode")
        self.theme_button.clicked.connect(self.theme_changed_requested.emit)

        layout.addWidget(self.logo_label)
        layout.addWidget(self.title_label)
        layout.addSpacerItem(spacer)
        layout.addWidget(self.theme_button)
        self.setLayout(layout)

        self.update_status({"status": "DISCONNECTED", "control_mode": "MANUAL"})

    @Slot(dict)
    def update_status(self, data):
        """Menerima data dan mengupdate (kini tidak melakukan apa-apa karena indikator dihapus)."""
        pass
