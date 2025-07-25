# gui/components/servo_view.py
# Komponen untuk mengatur batas sudut servo.

from PySide6.QtWidgets import (QGroupBox, QWidget, QVBoxLayout, 
                               QLineEdit, QPushButton, QFormLayout)
from PySide6.QtCore import Signal, Slot
from PySide6.QtGui import QIntValidator

class ServoView(QWidget):
    """
    Widget yang berisi input untuk batas sudut servo kiri dan kanan.
    """
    # Sinyal yang membawa dictionary berisi nilai sudut
    servo_settings_updated = Signal(dict)

    def __init__(self):
        super().__init__()

        main_layout = QVBoxLayout(self)
        
        # GroupBox untuk menampung pengaturan
        settings_group = QGroupBox("Servo Angle Limits")
        form_layout = QFormLayout()

        # Validator untuk memastikan hanya angka 0-180 yang bisa dimasukkan
        validator = QIntValidator(0, 180, self)

        # Input fields untuk sudut
        self.left_angle_input = QLineEdit("45")
        self.left_angle_input.setValidator(validator)
        
        self.right_angle_input = QLineEdit("135")
        self.right_angle_input.setValidator(validator)

        # Tambahkan input fields ke form layout
        form_layout.addRow("Max Left Angle (°):", self.left_angle_input)
        form_layout.addRow("Max Right Angle (°):", self.right_angle_input)
        
        settings_group.setLayout(form_layout)

        # Tombol untuk menyimpan pengaturan
        self.save_button = QPushButton("Save Servo Settings")
        self.save_button.setStyleSheet("background-color: #2a82da; color: white; font-weight: bold;")

        # Hubungkan tombol ke fungsi
        self.save_button.clicked.connect(self.on_save_clicked)

        # Tambahkan groupbox dan tombol ke layout utama
        main_layout.addWidget(settings_group)
        main_layout.addWidget(self.save_button)
        main_layout.addStretch() # Agar tidak memenuhi seluruh tab

    def on_save_clicked(self):
        """
        Dipanggil saat tombol simpan ditekan.
        Mengambil nilai dari input, membuat dictionary, dan memancarkan sinyal.
        """
        try:
            servo_values = {
                "left_angle": int(self.left_angle_input.text()),
                "right_angle": int(self.right_angle_input.text())
            }
            # Pancarkan sinyal dengan data baru
            self.servo_settings_updated.emit(servo_values)
            print(f"[Servo View] Sinyal servo_settings_updated dipancarkan: {servo_values}")
        except ValueError:
            print("[Servo View] Error: Input tidak valid. Pastikan semua field terisi angka.")
