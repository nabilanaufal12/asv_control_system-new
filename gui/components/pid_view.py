# gui/components/pid_view.py
# Komponen untuk mengatur konstanta PID.

from PySide6.QtWidgets import (QGroupBox, QWidget, QHBoxLayout, QVBoxLayout, 
                               QLabel, QLineEdit, QPushButton, QFormLayout)
from PySide6.QtCore import Signal, Slot
from PySide6.QtGui import QDoubleValidator

class PidView(QGroupBox):
    """
    Sebuah GroupBox yang berisi input untuk konstanta P, I, D
    dan sebuah tombol untuk mengirim pembaruan.
    """
    # Sinyal yang membawa dictionary berisi nilai P, I, D baru
    pid_updated = Signal(dict)

    def __init__(self, title="PID Controller Settings"):
        super().__init__(title)

        # Layout utama
        main_layout = QVBoxLayout()
        
        # Layout form untuk input
        form_layout = QFormLayout()

        # Validator untuk memastikan hanya angka yang bisa dimasukkan
        validator = QDoubleValidator(0.0, 999.0, 5, self)
        validator.setNotation(QDoubleValidator.StandardNotation)

        # Input fields untuk P, I, D
        self.p_input = QLineEdit("0.0")
        self.p_input.setValidator(validator)
        
        self.i_input = QLineEdit("0.0")
        self.i_input.setValidator(validator)

        self.d_input = QLineEdit("0.0")
        self.d_input.setValidator(validator)

        # Tambahkan input fields ke form layout
        form_layout.addRow("P (Proportional):", self.p_input)
        form_layout.addRow("I (Integral):", self.i_input)
        form_layout.addRow("D (Derivative):", self.d_input)

        # Tombol untuk mengirim pembaruan
        self.update_button = QPushButton("Update PID Constants")
        self.update_button.setStyleSheet("background-color: #5bc0de; color: white;")

        # Hubungkan tombol ke fungsi internal
        self.update_button.clicked.connect(self.on_update_clicked)

        # Tambahkan form dan tombol ke layout utama
        main_layout.addLayout(form_layout)
        main_layout.addWidget(self.update_button)

        self.setLayout(main_layout)

    def on_update_clicked(self):
        """
        Dipanggil saat tombol update ditekan.
        Mengambil nilai dari input, membuat dictionary, dan memancarkan sinyal.
        """
        try:
            pid_values = {
                "p": float(self.p_input.text()),
                "i": float(self.i_input.text()),
                "d": float(self.d_input.text())
            }
            # Pancarkan sinyal dengan data baru
            self.pid_updated.emit(pid_values)
            print(f"[PID View] Sinyal pid_updated dipancarkan: {pid_values}")
        except ValueError:
            print("[PID View] Error: Input tidak valid. Pastikan semua field terisi angka.")

