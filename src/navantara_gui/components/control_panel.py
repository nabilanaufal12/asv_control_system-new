# gui/components/control_panel.py
# --- VERSI FINAL: Disederhanakan, semua logika dipindah ke MainWindow ---

from PySide6.QtWidgets import QVBoxLayout, QGroupBox, QPushButton, QGridLayout
from PySide6.QtCore import Signal
from PySide6.QtGui import QFont


class ControlPanel(QGroupBox):
    # HANYA sinyal untuk memberi tahu tombol mana yang diklik
    manual_button_clicked = Signal()
    auto_button_clicked = Signal()
    emergency_stop_clicked = Signal()
    navigation_command = Signal(str)

    def __init__(self, config, title="Vehicle Control"):
        super().__init__(title)

        self.config = config
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)

        button_font = QFont()
        button_font.setPointSize(9)

        mode_box = QGroupBox("Mode")
        mode_layout = QVBoxLayout()

        self.manual_mode_btn = QPushButton("MANUAL")
        self.manual_mode_btn.setCheckable(True)
        self.manual_mode_btn.setFont(button_font)

        self.auto_mode_btn = QPushButton("AUTO")
        self.auto_mode_btn.setCheckable(True)
        self.auto_mode_btn.setFont(button_font)

        # Hubungkan klik tombol ke sinyal baru
        self.manual_mode_btn.clicked.connect(self.manual_button_clicked.emit)
        self.auto_mode_btn.clicked.connect(self.auto_button_clicked.emit)

        self.emergency_stop_btn = QPushButton("EMERGENCY\nSTOP")
        self.emergency_stop_btn.setStyleSheet(
            "background-color: #D32F2F; color: white; font-weight: bold;"
        )
        self.emergency_stop_btn.setFont(button_font)
        self.emergency_stop_btn.clicked.connect(self.emergency_stop_clicked.emit)

        mode_layout.addWidget(self.manual_mode_btn)
        mode_layout.addWidget(self.auto_mode_btn)
        mode_layout.addWidget(self.emergency_stop_btn)
        mode_box.setLayout(mode_layout)

        navigation_box = QGroupBox("Navigation")
        navigation_layout = QVBoxLayout()
        self.start_mission_btn = QPushButton("Start\nMission")
        self.start_mission_btn.setFont(button_font)
        self.pause_mission_btn = QPushButton("Pause\nMission")
        self.pause_mission_btn.setFont(button_font)
        self.return_home_btn = QPushButton("Return\nHome")
        self.return_home_btn.setFont(button_font)

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
        key_style = "padding: 8px; font-weight: bold; font-size: 12px;"
        self.key_buttons = {
            "W": QPushButton("W (↑)"),
            "A": QPushButton("A (←)"),
            "S": QPushButton("S (↓)"),
            "D": QPushButton("D (→)"),
        }
        for button in self.key_buttons.values():
            button.setStyleSheet(key_style)
            # Logika press/release akan ditangani oleh MainWindow

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

    def update_key_press_status(self, key_char, is_pressed):
        key_char = key_char.upper()
        if key_char in self.key_buttons:
            self.key_buttons[key_char].setDown(is_pressed)
