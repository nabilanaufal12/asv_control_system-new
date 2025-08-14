# gui/components/servo_view.py
# --- MODIFIKASI: Menggunakan config.json dan menambahkan umpan balik visual ---

from PySide6.QtWidgets import (QGroupBox, QWidget, QVBoxLayout, 
                               QLineEdit, QPushButton, QFormLayout)
from PySide6.QtCore import Signal, Slot, QTimer # <-- 1. Impor QTimer
from PySide6.QtGui import QIntValidator

class ServoView(QWidget):
    """
    Widget yang berisi input untuk batas sudut servo kiri dan kanan.
    """
    servo_settings_updated = Signal(dict)

    # --- 2. Menerima objek 'config' ---
    def __init__(self, config):
        super().__init__()
        self.config = config # Simpan objek konfigurasi

        main_layout = QVBoxLayout(self)
        
        settings_group = QGroupBox("Servo Angle Limits")
        form_layout = QFormLayout()

        validator = QIntValidator(0, 180, self)

        # --- 3. Menggunakan nilai dari config.json ---
        actuator_config = self.config.get("actuators", {})
        default_left = str(actuator_config.get("servo_min_angle", 45))
        default_right = str(actuator_config.get("servo_max_angle", 135))

        self.left_angle_input = QLineEdit(default_left)
        self.left_angle_input.setValidator(validator)
        
        self.right_angle_input = QLineEdit(default_right)
        self.right_angle_input.setValidator(validator)

        form_layout.addRow("Max Left Angle (°):", self.left_angle_input)
        form_layout.addRow("Max Right Angle (°):", self.right_angle_input)
        
        settings_group.setLayout(form_layout)

        self.save_button = QPushButton("Save Servo Settings")
        self.original_button_style = "background-color: #2a82da; color: white; font-weight: bold;"
        self.save_button.setStyleSheet(self.original_button_style)

        self.save_button.clicked.connect(self.on_save_clicked)

        main_layout.addWidget(settings_group)
        main_layout.addWidget(self.save_button)
        main_layout.addStretch()
        self.setLayout(main_layout)

    def on_save_clicked(self):
        """
        Dipanggil saat tombol simpan ditekan, memancarkan sinyal, dan memberikan umpan balik.
        """
        try:
            servo_values = {
                "left_angle": int(self.left_angle_input.text()),
                "right_angle": int(self.right_angle_input.text())
            }
            self.servo_settings_updated.emit(servo_values)
            print(f"[Servo View] Sinyal servo_settings_updated dipancarkan: {servo_values}")

            # --- 4. Umpan balik visual untuk SUKSES ---
            self.save_button.setText("Saved!")
            self.save_button.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
            QTimer.singleShot(1500, self.reset_button_style)

        except ValueError:
            print("[Servo View] Error: Input tidak valid. Pastikan semua field terisi angka.")
            # --- 5. Umpan balik visual untuk GAGAL ---
            self.save_button.setText("Error!")
            self.save_button.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
            QTimer.singleShot(2000, self.reset_button_style)

    def reset_button_style(self):
        """Fungsi baru untuk mengembalikan tombol ke tampilan aslinya."""
        self.save_button.setText("Save Servo Settings")
        self.save_button.setStyleSheet(self.original_button_style)