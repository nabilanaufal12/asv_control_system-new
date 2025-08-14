# gui/components/debug_panel.py
# --- MODIFIKASI: Menerima objek 'config' ---

from PySide6.QtWidgets import (QGroupBox, QWidget, QVBoxLayout, QHBoxLayout,
                               QPushButton, QFormLayout, QLineEdit)
from PySide6.QtCore import Signal

class DebugPanel(QWidget):
    """
    Widget yang berisi berbagai alat untuk debugging dan kalibrasi,
    diambil dari fungsionalitas GUI lama.
    """
    debug_command_sent = Signal(str, object)

    # --- 1. UBAH TANDA TANGAN FUNGSI __init__ ---
    def __init__(self, config):
        super().__init__()

        # --- 2. SIMPAN OBJEK KONFIGURASI ---
        self.config = config

        main_layout = QVBoxLayout(self)

        # === Panel Kontrol AI & Motor ===
        ai_control_group = QGroupBox("Panel Kontrol AI & Motor")
        ai_layout = QVBoxLayout()
        
        ai_buttons_layout = QHBoxLayout()
        self.counter_plus_btn = QPushButton("Counter Plus")
        self.counter_min_btn = QPushButton("Counter Min")
        self.inverse_btn = QPushButton("Inverse")
        ai_buttons_layout.addWidget(self.counter_plus_btn)
        ai_buttons_layout.addWidget(self.counter_min_btn)
        ai_buttons_layout.addWidget(self.inverse_btn)

        motor_buttons_layout = QHBoxLayout()
        self.motor_set_btn = QPushButton("Motor Set")
        self.reset_btn = QPushButton("Reset")
        motor_buttons_layout.addWidget(self.motor_set_btn)
        motor_buttons_layout.addWidget(self.reset_btn)

        ai_layout.addLayout(ai_buttons_layout)
        ai_layout.addLayout(motor_buttons_layout)
        ai_control_group.setLayout(ai_layout)

        # === Panel Input Data Spesifik ===
        specific_data_group = QGroupBox("Input Data Spesifik")
        form_layout = QFormLayout()
        
        self.azimuth_input = QLineEdit()
        self.lat_direction_input = QLineEdit()
        self.long_direction_input = QLineEdit()
        
        form_layout.addRow("Azimuth:", self.azimuth_input)
        form_layout.addRow("Lat Direction:", self.lat_direction_input)
        form_layout.addRow("Long Direction:", self.long_direction_input)
        
        self.send_specific_data_btn = QPushButton("Send Specific Data")
        form_layout.addRow(self.send_specific_data_btn)
        
        specific_data_group.setLayout(form_layout)

        # Tambahkan semua ke layout utama
        main_layout.addWidget(ai_control_group)
        main_layout.addWidget(specific_data_group)
        main_layout.addStretch()

        # Hubungkan tombol ke fungsi yang memancarkan sinyal
        self.counter_plus_btn.clicked.connect(lambda: self.debug_command_sent.emit("AI_CONTROL", "COUNTER_PLUS"))
        self.counter_min_btn.clicked.connect(lambda: self.debug_command_sent.emit("AI_CONTROL", "COUNTER_MIN"))
        self.inverse_btn.clicked.connect(lambda: self.debug_command_sent.emit("AI_CONTROL", "INVERSE"))
        self.motor_set_btn.clicked.connect(lambda: self.debug_command_sent.emit("AI_CONTROL", "MOTOR_SET"))
        self.reset_btn.clicked.connect(lambda: self.debug_command_sent.emit("AI_CONTROL", "RESET"))
        
        self.send_specific_data_btn.clicked.connect(self.send_data)

    def send_data(self):
        """Mengumpulkan data dari input dan mengirimkannya."""
        data = {
            "azimuth": self.azimuth_input.text(),
            "lat_direction": self.lat_direction_input.text(),
            "long_direction": self.long_direction_input.text()
        }
        self.debug_command_sent.emit("SPECIFIC_DATA", data)