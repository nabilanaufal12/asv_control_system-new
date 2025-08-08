# gui/components/connection_view.py
# --- PERBAIKAN: Memperbaiki urutan inisialisasi untuk mengatasi AttributeError ---

import serial.tools.list_ports
from PySide6.QtWidgets import (QGroupBox, QWidget, QVBoxLayout, QLineEdit, 
                               QPushButton, QFormLayout, QComboBox, QHBoxLayout)
from PySide6.QtCore import Signal

class ConnectionView(QWidget):
    """
    Widget yang berisi input untuk pengaturan koneksi dengan pemindaian port otomatis.
    """
    connect_requested = Signal(dict)

    def __init__(self):
        super().__init__()

        main_layout = QVBoxLayout(self)
        
        backend_group = QGroupBox("Backend Connection")
        backend_form = QFormLayout()
        self.ip_input = QLineEdit("127.0.0.1")
        self.port_input = QLineEdit("5000")
        backend_form.addRow("Backend IP Address:", self.ip_input)
        backend_form.addRow("Backend Port:", self.port_input)
        backend_group.setLayout(backend_form)

        serial_group = QGroupBox("Serial Port (on ASV)")
        serial_form = QFormLayout()

        self.serial_port_combo = QComboBox()
        self.refresh_ports_button = QPushButton("Refresh Ports")
        
        port_layout = QHBoxLayout()
        port_layout.addWidget(self.serial_port_combo, 1)
        port_layout.addWidget(self.refresh_ports_button)

        self.baud_rate_combo = QComboBox()
        self.baud_rate_combo.addItems(["9600", "57600", "115200"])
        self.baud_rate_combo.setCurrentText("115200")

        serial_form.addRow("Serial Port:", port_layout)
        serial_form.addRow("Baud Rate:", self.baud_rate_combo)
        serial_group.setLayout(serial_form)

        # === Tombol Aksi ===
        self.connect_button = QPushButton("Save & Connect")
        self.connect_button.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold;")
        
        # --- PERUBAHAN UTAMA DI SINI ---
        # Panggil fungsi refresh_serial_ports SETELAH semua tombol dibuat.
        self.refresh_serial_ports()

        # Hubungkan sinyal tombol
        self.refresh_ports_button.clicked.connect(self.refresh_serial_ports)
        self.connect_button.clicked.connect(self.on_connect_clicked)

        # Tambahkan semua ke layout utama
        main_layout.addWidget(backend_group)
        main_layout.addWidget(serial_group)
        main_layout.addWidget(self.connect_button)
        main_layout.addStretch()

    def refresh_serial_ports(self):
        """
        Memindai semua port serial yang tersedia dan memperbarui QComboBox.
        """
        self.serial_port_combo.clear()
        ports = serial.tools.list_ports.comports()
        
        self.serial_port_combo.addItem("AUTO")
        
        port_list = [port.device for port in ports]
        if not port_list:
            self.serial_port_combo.addItem("Tidak ada port ditemukan")
            self.connect_button.setEnabled(False) # Sekarang ini akan berjalan tanpa error
        else:
            self.serial_port_combo.addItems(port_list)
            self.connect_button.setEnabled(True)
        print("Daftar port serial telah diperbarui.")

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