# gui/components/control_panel.py
# Komponen ini berisi tombol-tombol utama untuk mengendalikan ASV.

from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                               QPushButton)
from PySide6.QtCore import Signal, Qt, Slot

class ControlPanel(QGroupBox):
    """
    Sebuah GroupBox yang berisi tombol-tombol kendali utama.
    """
    mode_changed = Signal(str)
    emergency_stop_clicked = Signal()
    manual_control_updated = Signal(str)
    navigation_command = Signal(str)

    def __init__(self, title="Vehicle Control"):
        super().__init__(title)
        self.current_mode = "MANUAL"

        main_layout = QVBoxLayout()

        # === Bagian Kontrol Mode ===
        self.mode_button = QPushButton("Switch to Auto Mode")
        self.emergency_stop_button = QPushButton("Emergency Stop")
        self.mode_button.setStyleSheet("background-color: #2a82da; color: white; font-weight: bold;")
        self.emergency_stop_button.setStyleSheet("background-color: #d9534f; color: white; font-weight: bold;")
        main_layout.addWidget(self.mode_button)
        main_layout.addWidget(self.emergency_stop_button)

        # === Panel Navigasi (tersembunyi) ===
        self.navigation_group = QGroupBox("Navigation")
        nav_layout = QVBoxLayout()
        self.start_mission_button = QPushButton("Start Mission")
        self.pause_mission_button = QPushButton("Pause Mission")
        self.return_home_button = QPushButton("Return Home")
        nav_layout.addWidget(self.start_mission_button)
        nav_layout.addWidget(self.pause_mission_button)
        nav_layout.addWidget(self.return_home_button)
        self.navigation_group.setLayout(nav_layout)
        self.navigation_group.setVisible(False)
        main_layout.addWidget(self.navigation_group)

        # === Bagian Kontrol Manual (disederhanakan) ===
        self.manual_control_group = QGroupBox("Manual Control")
        direction_layout = QHBoxLayout()
        self.left_button = QPushButton("Left")
        self.forward_button = QPushButton("Forward")
        self.right_button = QPushButton("Right")
        direction_layout.addWidget(self.left_button)
        direction_layout.addWidget(self.forward_button)
        direction_layout.addWidget(self.right_button)
        self.manual_control_group.setLayout(direction_layout)

        main_layout.addWidget(self.manual_control_group)
        main_layout.addStretch()

        self.setLayout(main_layout)

        # Hubungkan event 'clicked'
        self.mode_button.clicked.connect(self.toggle_mode)
        self.emergency_stop_button.clicked.connect(self.emergency_stop_clicked.emit)
        self.left_button.clicked.connect(lambda: self.manual_control_updated.emit("LEFT"))
        self.forward_button.clicked.connect(lambda: self.manual_control_updated.emit("FORWARD"))
        self.right_button.clicked.connect(lambda: self.manual_control_updated.emit("RIGHT"))
        self.start_mission_button.clicked.connect(lambda: self.navigation_command.emit("START"))
        self.pause_mission_button.clicked.connect(lambda: self.navigation_command.emit("PAUSE"))
        self.return_home_button.clicked.connect(lambda: self.navigation_command.emit("RETURN"))

    def toggle_mode(self):
        if self.current_mode == "MANUAL":
            self.mode_changed.emit("AUTO")
        else:
            self.mode_changed.emit("MANUAL")

    @Slot(str)
    def set_mode(self, mode):
        self.current_mode = mode
        if mode == "AUTO":
            self.mode_button.setText("Switch to Manual Mode")
            self.manual_control_group.setVisible(False)
            self.navigation_group.setVisible(True)
        else: # MANUAL
            self.mode_button.setText("Switch to Auto Mode")
            self.manual_control_group.setVisible(True)
            self.navigation_group.setVisible(False)

