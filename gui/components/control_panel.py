# gui/components/control_panel.py
# --- MODIFIKASI: Menerima objek 'config' untuk konsistensi ---

from PySide6.QtWidgets import QVBoxLayout, QGroupBox, QPushButton, QGridLayout
from PySide6.QtCore import Signal
from PySide6.QtGui import QFont


class ControlPanel(QGroupBox):
    mode_changed = Signal(str)
    emergency_stop_clicked = Signal()
    manual_control_updated = Signal(str)
    navigation_command = Signal(str)

    # --- 1. UBAH TANDA TANGAN FUNGSI __init__ ---
    def __init__(self, config, title="Vehicle Control"):
        super().__init__(title)

        # --- 2. SIMPAN OBJEK KONFIGURASI ---
        self.config = config

        main_layout = QVBoxLayout()

        # --- Bagian Mode Control ---
        mode_box = QGroupBox("Mode")
        mode_layout = QVBoxLayout()

        self.manual_mode_btn = QPushButton("MANUAL")
        self.manual_mode_btn.setCheckable(True)
        self.auto_mode_btn = QPushButton("AUTO")  # Tombol AUTO yang digabungkan
        self.auto_mode_btn.setCheckable(True)

        self.manual_mode_btn.clicked.connect(lambda: self.set_mode("MANUAL"))
        self.auto_mode_btn.clicked.connect(lambda: self.set_mode("AUTO"))

        self.emergency_stop_btn = QPushButton("EMERGENCY STOP")
        self.emergency_stop_btn.setStyleSheet(
            "background-color: #D32F2F; color: white; font-weight: bold;"
        )
        self.emergency_stop_btn.clicked.connect(self.emergency_stop_clicked.emit)

        mode_layout.addWidget(self.manual_mode_btn)
        mode_layout.addWidget(self.auto_mode_btn)
        mode_layout.addWidget(self.emergency_stop_btn)
        mode_box.setLayout(mode_layout)

        # (Sisa kode tidak berubah)
        navigation_box = QGroupBox("Navigation")
        navigation_layout = QVBoxLayout()
        self.start_mission_btn = QPushButton("Start Mission")
        self.pause_mission_btn = QPushButton("Pause Mission")
        self.return_home_btn = QPushButton("Return Home")
        self.start_mission_btn.clicked.connect(
            lambda: self.navigation_command.emit("START")
        )
        self.pause_mission_btn.clicked.connect(
            lambda: self.navigation_command.emit("PAUSE")
        )
        self.return_home_btn.clicked.connect(
            lambda: self.navigation_command.emit("RETURN")
        )
        navigation_layout.addWidget(self.start_mission_btn)
        navigation_layout.addWidget(self.pause_mission_btn)
        navigation_layout.addWidget(self.return_home_btn)
        navigation_box.setLayout(navigation_layout)
        manual_control_box = QGroupBox("Manual Control (WASD)")
        keyboard_layout = QGridLayout()
        key_style = "padding: 10px; font-weight: bold; font-size: 14px;"
        self.key_buttons = {
            "W": QPushButton("W (↑)"),
            "A": QPushButton("A (←)"),
            "S": QPushButton("S (↓)"),
            "D": QPushButton("D (→)"),
        }
        self.key_buttons["W"].pressed.connect(
            lambda: self.manual_control_updated.emit("FORWARD")
        )
        self.key_buttons["A"].pressed.connect(
            lambda: self.manual_control_updated.emit("LEFT")
        )
        self.key_buttons["S"].pressed.connect(
            lambda: self.manual_control_updated.emit("BACKWARD")
        )
        self.key_buttons["D"].pressed.connect(
            lambda: self.manual_control_updated.emit("RIGHT")
        )
        for button in self.key_buttons.values():
            button.released.connect(lambda: self.manual_control_updated.emit("STOP"))
            button.setStyleSheet(key_style)
            button.setFont(QFont("Monospace", 12))
        keyboard_layout.addWidget(self.key_buttons["W"], 0, 1)
        keyboard_layout.addWidget(self.key_buttons["A"], 1, 0)
        keyboard_layout.addWidget(self.key_buttons["S"], 1, 1)
        keyboard_layout.addWidget(self.key_buttons["D"], 1, 2)
        manual_control_box.setLayout(keyboard_layout)

        main_layout.addWidget(mode_box)
        main_layout.addWidget(navigation_box)
        main_layout.addWidget(manual_control_box)
        main_layout.addStretch()
        self.setLayout(main_layout)

        self.set_mode("MANUAL")

    def set_mode(self, mode):
        is_manual = mode == "MANUAL"
        self.manual_mode_btn.setChecked(is_manual)
        self.auto_mode_btn.setChecked(not is_manual)

        for button in self.key_buttons.values():
            button.setEnabled(is_manual)

        self.start_mission_btn.setEnabled(not is_manual)
        self.pause_mission_btn.setEnabled(not is_manual)
        self.return_home_btn.setEnabled(not is_manual)

        self.mode_changed.emit(mode)

    def update_key_press_status(self, key_char, is_pressed):
        key_char = key_char.upper()
        if key_char in self.key_buttons:
            self.key_buttons[key_char].setDown(is_pressed)
