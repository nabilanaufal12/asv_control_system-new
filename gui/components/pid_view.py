# gui/components/pid_view.py
# --- MODIFIKASI: Menambahkan umpan balik visual pada tombol update ---

from PySide6.QtWidgets import (QGroupBox, QWidget, QHBoxLayout, QVBoxLayout, 
                               QLabel, QLineEdit, QPushButton, QFormLayout)
from PySide6.QtCore import Signal, Slot, QTimer # <-- 1. Impor QTimer
from PySide6.QtGui import QDoubleValidator

class PidView(QGroupBox):
    """
    Sebuah GroupBox yang berisi input untuk konstanta P, I, D
    dan sebuah tombol untuk mengirim pembaruan.
    """
    pid_updated = Signal(dict)

    def __init__(self, title="PID Controller Settings"):
        super().__init__(title)

        main_layout = QVBoxLayout()
        form_layout = QFormLayout()

        validator = QDoubleValidator(0.0, 999.0, 5, self)
        validator.setNotation(QDoubleValidator.StandardNotation)

        self.p_input = QLineEdit("0.0")
        self.p_input.setValidator(validator)
        
        self.i_input = QLineEdit("0.0")
        self.i_input.setValidator(validator)

        self.d_input = QLineEdit("0.0")
        self.d_input.setValidator(validator)

        form_layout.addRow("P (Proportional):", self.p_input)
        form_layout.addRow("I (Integral):", self.i_input)
        form_layout.addRow("D (Derivative):", self.d_input)

        self.update_button = QPushButton("Update PID Constants")
        # --- 2. Simpan style asli tombol ---
        self.original_button_style = "background-color: #5bc0de; color: white;"
        self.update_button.setStyleSheet(self.original_button_style)

        self.update_button.clicked.connect(self.on_update_clicked)

        main_layout.addLayout(form_layout)
        main_layout.addWidget(self.update_button)
        self.setLayout(main_layout)

    def on_update_clicked(self):
        """
        Dipanggil saat tombol update ditekan.
        Mengambil nilai, memancarkan sinyal, dan memberikan umpan balik visual.
        """
        try:
            pid_values = {
                "p": float(self.p_input.text().replace(',', '.')), # Ganti koma dengan titik
                "i": float(self.i_input.text().replace(',', '.')),
                "d": float(self.d_input.text().replace(',', '.'))
            }
            self.pid_updated.emit(pid_values)
            print(f"[PID View] Sinyal pid_updated dipancarkan: {pid_values}")

            # --- 3. Umpan balik visual untuk SUKSES ---
            self.update_button.setText("Updated!")
            self.update_button.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
            
            # Kembalikan ke semula setelah 1.5 detik
            QTimer.singleShot(1500, self.reset_button_style)

        except ValueError:
            print("[PID View] Error: Input tidak valid. Pastikan semua field terisi angka.")
            
            # --- 4. Umpan balik visual untuk GAGAL ---
            self.update_button.setText("Error! Input tidak valid")
            self.update_button.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
            QTimer.singleShot(2000, self.reset_button_style)

    def reset_button_style(self):
        """Fungsi baru untuk mengembalikan tombol ke tampilan aslinya."""
        self.update_button.setText("Update PID Constants")
        self.update_button.setStyleSheet(self.original_button_style)