# gui/components/debug_panel.py
# --- MODIFIKASI: Menerima objek 'config' ---

from PySide6.QtWidgets import (
    QGroupBox,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
)
from PySide6.QtCore import Signal


class DebugPanel(QWidget):
    """
    Widget yang berisi alat untuk debugging Manual Waypoint Update.
    """

    debug_command_sent = Signal(str, object)

    def __init__(self, config):
        super().__init__()

        self.config = config

        main_layout = QVBoxLayout(self)

        # === Manual Waypoint Update ===
        wp_control_group = QGroupBox("Mission Waypoint Override")
        wp_layout = QHBoxLayout()

        self.counter_plus_btn = QPushButton("Counter Plus")
        self.counter_min_btn = QPushButton("Counter Min")
        self.reset_btn = QPushButton("Reset")

        wp_layout.addWidget(self.counter_plus_btn)
        wp_layout.addWidget(self.counter_min_btn)
        wp_layout.addWidget(self.reset_btn)

        wp_control_group.setLayout(wp_layout)

        # Tambahkan ke layout utama
        main_layout.addWidget(wp_control_group)

        # Hubungkan tombol ke fungsi yang memancarkan sinyal
        self.counter_plus_btn.clicked.connect(
            lambda: self.debug_command_sent.emit("DEBUG_WP_COUNTER", {"action": "INC"})
        )
        self.counter_min_btn.clicked.connect(
            lambda: self.debug_command_sent.emit("DEBUG_WP_COUNTER", {"action": "DEC"})
        )
        self.reset_btn.clicked.connect(
            lambda: self.debug_command_sent.emit(
                "DEBUG_WP_COUNTER", {"action": "RESET"}
            )
        )
