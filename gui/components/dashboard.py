# gui/components/dashboard.py
# --- MODIFIKASI: Menerima objek 'config' ---

from PySide6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QLabel, QFormLayout
from PySide6.QtCore import Slot
from PySide6.QtGui import QFont


class Dashboard(QGroupBox):
    """
    Sebuah GroupBox yang berisi label-label untuk menampilkan data telemetri.
    """

    # --- 1. UBAH TANDA TANGAN FUNGSI __init__ ---
    def __init__(self, config, title="System Status"):
        super().__init__(title)

        # --- 2. SIMPAN OBJEK KONFIGURASI ---
        self.config = config

        label_font = QFont()
        label_font.setBold(True)
        value_font = QFont()
        value_font.setPointSize(10)

        layout = QFormLayout()

        self.status_label = QLabel("DISCONNECTED")
        self.status_label.setFont(value_font)
        self.lat_label = QLabel("-")
        self.lat_label.setFont(value_font)
        self.lon_label = QLabel("-")
        self.lon_label.setFont(value_font)
        self.heading_label = QLabel("-")
        self.heading_label.setFont(value_font)
        self.speed_label = QLabel("-")
        self.speed_label.setFont(value_font)
        self.voltage_label = QLabel("-")
        self.voltage_label.setFont(value_font)
        self.mission_time_label = QLabel("-")
        self.mission_time_label.setFont(value_font)

        layout.addRow("Status:", self.status_label)
        layout.addRow("Latitude:", self.lat_label)
        layout.addRow("Longitude:", self.lon_label)
        layout.addRow("Heading:", self.heading_label)
        layout.addRow("Speed:", self.speed_label)
        layout.addRow("Battery:", self.voltage_label)
        layout.addRow("Mission Time:", self.mission_time_label)

        self.setLayout(layout)

    @Slot(dict)
    def update_data(self, data):
        """
        Slot publik untuk mengupdate semua label dengan data baru dari dictionary.
        Metode ini sekarang lebih tangguh terhadap data yang hilang atau rusak.
        """
        try:
            status = data.get("status", "N/A")
            self.status_label.setText(status)
            if status == "NAVIGATING":
                self.status_label.setStyleSheet("color: lightgreen;")
            elif status == "IDLE":
                self.status_label.setStyleSheet("color: lightblue;")
            else:
                self.status_label.setStyleSheet("color: orange;")

            self.lat_label.setText(f"{float(data.get('latitude', 0)):.6f}")
            self.lon_label.setText(f"{float(data.get('longitude', 0)):.6f}")
            self.heading_label.setText(f"{float(data.get('heading', 0)):.2f}°")
            self.speed_label.setText(f"{float(data.get('speed', 0)):.2f} m/s")
            self.voltage_label.setText(f"{float(data.get('battery_voltage', 0)):.2f} V")

            self.mission_time_label.setText(data.get("mission_time", "00:00:00"))

        except (ValueError, TypeError) as e:
            print(f"Dashboard Error: Data tidak valid diterima. Error: {e}")
            self.status_label.setText("DATA ERROR")
            self.status_label.setStyleSheet("color: red;")
        except Exception as e:
            print(f"Dashboard Error: Terjadi kesalahan tak terduga. Error: {e}")
