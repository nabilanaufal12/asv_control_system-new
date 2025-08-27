# gui/components/dashboard.py
# --- VERSI FINAL: Dengan perbaikan Word Wrap yang benar ---

from PySide6.QtWidgets import QGroupBox, QLabel, QFormLayout
from PySide6.QtCore import Slot
from PySide6.QtGui import QFont


class Dashboard(QGroupBox):
    """
    Sebuah GroupBox yang berisi label-label untuk menampilkan data telemetri.
    """

    def __init__(self, config, title="System Status"):
        super().__init__(title)

        self.config = config

        value_font = QFont()
        value_font.setPointSize(10)

        # Gunakan QFormLayout seperti biasa
        layout = QFormLayout()

        # --- PERBAIKAN DI SINI ---
        # Baris yang menyebabkan error telah dihapus.
        # layout.setWrappingPolicy(QFormLayout.WrapAllRows) # <-- BARIS INI DIHAPUS

        # Buat semua label
        self.status_label = QLabel("DISCONNECTED")
        self.lat_label = QLabel("-")
        self.lon_label = QLabel("-")
        self.heading_label = QLabel("-")
        self.speed_label = QLabel("-")
        self.voltage_label = QLabel("-")
        self.mission_time_label = QLabel("-")

        # Kumpulkan semua label nilai ke dalam satu list untuk di-looping
        all_value_labels = [
            self.status_label, self.lat_label, self.lon_label,
            self.heading_label, self.speed_label, self.voltage_label,
            self.mission_time_label
        ]

        # Terapkan font dan aktifkan Word Wrap untuk semua label nilai
        # Ini adalah cara yang benar dan sudah ada sebelumnya.
        for label in all_value_labels:
            label.setFont(value_font)
            label.setWordWrap(True) # Aktifkan word wrap di setiap QLabel

        # Tambahkan baris ke form layout
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
        """
        try:
            status = data.get("status", "N/A")
            self.status_label.setText(status)

            if status == "NAVIGATING":
                self.status_label.setStyleSheet("color: lightgreen;")
            elif status == "RETURNING TO HOME":
                self.status_label.setStyleSheet(
                    "color: #3498db; font-weight: bold;"
                )
            elif status == "IDLE":
                self.status_label.setStyleSheet("color: lightblue;")
            elif status == "CONNECTED":
                self.status_label.setStyleSheet(
                    "color: white;"
                )
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