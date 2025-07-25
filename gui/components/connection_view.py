# gui/components/connection_view.py
# Komponen untuk mengatur koneksi ke backend dan port serial.

from PySide6.QtWidgets import (QGroupBox, QWidget, QVBoxLayout, QLineEdit, 
                               QPushButton, QFormLayout, QComboBox)
from PySide6.QtCore import Signal

class ConnectionView(QWidget):
    """
    Widget yang berisi input untuk pengaturan koneksi.
    """
    # Sinyal yang membawa dictionary berisi detail koneksi
    connect_requested = Signal(dict)

    def __init__(self):
        super().__init__()

        main_layout = QVBoxLayout(self)
        
        # === Pengaturan Koneksi Backend ===
        backend_group = QGroupBox("Backend Connection")
        backend_form = QFormLayout()

        self.ip_input = QLineEdit("127.0.0.1") # Default ke localhost
        self.port_input = QLineEdit("5000")

        backend_form.addRow("Backend IP Address:", self.ip_input)
        backend_form.addRow("Backend Port:", self.port_input)
        backend_group.setLayout(backend_form)

        # === Pengaturan Port Serial (untuk dikirim ke backend) ===
        serial_group = QGroupBox("Serial Port (on ASV)")
        serial_form = QFormLayout()

        self.serial_port_combo = QComboBox()
        # Nanti kita bisa isi dengan hasil scan, untuk sekarang kita isi manual
        self.serial_port_combo.addItems(["/dev/ttyUSB0", "/dev/ttyUSB1", "COM3", "COM4"])

        self.baud_rate_combo = QComboBox()
        self.baud_rate_combo.addItems(["9600", "57600", "115200"])
        self.baud_rate_combo.setCurrentText("115200")

        serial_form.addRow("Serial Port:", self.serial_port_combo)
        serial_form.addRow("Baud Rate:", self.baud_rate_combo)
        serial_group.setLayout(serial_form)

        # === Tombol Aksi ===
        self.connect_button = QPushButton("Save & Connect")
        self.connect_button.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold;")

        # Hubungkan tombol ke fungsi
        self.connect_button.clicked.connect(self.on_connect_clicked)

        # Tambahkan semua ke layout utama
        main_layout.addWidget(backend_group)
        main_layout.addWidget(serial_group)
        main_layout.addWidget(self.connect_button)
        main_layout.addStretch()

    def on_connect_clicked(self):
        """Mengambil semua detail koneksi dan memancarkan sinyal."""
        connection_details = {
            "ip_address": self.ip_input.text(),
            "port": int(self.port_input.text()),
            "serial_port": self.serial_port_combo.currentText(),
            "baud_rate": int(self.baud_rate_combo.currentText())
        }
        self.connect_requested.emit(connection_details)
        print(f"[ConnectionView] Sinyal connect_requested dipancarkan: {connection_details}")

