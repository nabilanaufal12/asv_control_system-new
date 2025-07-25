# gui/components/waypoints_panel.py
# Komponen untuk mengelola daftar waypoints.

from PySide6.QtWidgets import (QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, 
                               QLineEdit, QPushButton, QListWidget, QAbstractItemView)
from PySide6.QtCore import Signal
from PySide6.QtGui import QDoubleValidator

class WaypointsPanel(QGroupBox):
    """
    Panel untuk menambah, menghapus, dan mengirim daftar waypoints.
    """
    send_waypoints = Signal(list)

    def __init__(self, title="Waypoints"):
        super().__init__(title)

        main_layout = QVBoxLayout()
        
        input_layout = QHBoxLayout()
        self.lat_input = QLineEdit()
        self.lat_input.setPlaceholderText("e.g., -6.2100")
        self.lon_input = QLineEdit()
        self.lon_input.setPlaceholderText("e.g., 106.8400")

        validator = QDoubleValidator(-90.0, 90.0, 6, self)
        self.lat_input.setValidator(validator)
        validator_lon = QDoubleValidator(-180.0, 180.0, 6, self)
        self.lon_input.setValidator(validator_lon)
        
        input_layout.addWidget(QLabel("Latitude:"))
        input_layout.addWidget(self.lat_input)
        input_layout.addWidget(QLabel("Longitude:"))
        input_layout.addWidget(self.lon_input)

        self.waypoints_list = QListWidget()
        self.waypoints_list.setSelectionMode(QAbstractItemView.SingleSelection)

        button_layout = QHBoxLayout()
        self.add_button = QPushButton("Add")
        self.delete_button = QPushButton("Delete")
        self.send_all_button = QPushButton("Send All")
        self.send_all_button.setStyleSheet("background-color: #2a82da; color: white; font-weight: bold;")
        
        button_layout.addWidget(self.add_button)
        button_layout.addWidget(self.delete_button)
        button_layout.addStretch()
        button_layout.addWidget(self.send_all_button)

        main_layout.addLayout(input_layout)
        main_layout.addWidget(self.waypoints_list)
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)

        self.add_button.clicked.connect(self.add_waypoint)
        self.delete_button.clicked.connect(self.delete_waypoint)
        self.send_all_button.clicked.connect(self.send_all_waypoints)

    def add_waypoint(self):
        """Menambahkan waypoint baru ke dalam daftar."""
        # --- PERBAIKAN: Ganti koma dengan titik untuk menangani input lokal ---
        lat_text = self.lat_input.text().replace(',', '.')
        lon_text = self.lon_input.text().replace(',', '.')

        if lat_text and lon_text:
            try:
                lat_float = float(lat_text)
                lon_float = float(lon_text)
                
                waypoint_text = f"Lat: {lat_float:.6f}, Lon: {lon_float:.6f}"
                self.waypoints_list.addItem(waypoint_text)
                
                self.lat_input.clear()
                self.lon_input.clear()
            except ValueError:
                print("Error: Input waypoint tidak valid. Pastikan menggunakan angka.")

    def delete_waypoint(self):
        """Menghapus waypoint yang dipilih dari daftar."""
        selected_items = self.waypoints_list.selectedItems()
        if not selected_items:
            return
        for item in selected_items:
            self.waypoints_list.takeItem(self.waypoints_list.row(item))

    def send_all_waypoints(self):
        """Mengambil semua waypoint dari daftar dan memancarkan sinyal."""
        all_waypoints = []
        for i in range(self.waypoints_list.count()):
            item_text = self.waypoints_list.item(i).text()
            try:
                parts = item_text.replace(" ", "").split(',')
                lat = float(parts[0].split(':')[1])
                lon = float(parts[1].split(':')[1])
                all_waypoints.append({"lat": lat, "lon": lon})
            except (ValueError, IndexError):
                print(f"Gagal mem-parsing item waypoint: {item_text}")
        
        if all_waypoints:
            self.send_waypoints.emit(all_waypoints)
            print(f"[WaypointsPanel] Sinyal send_waypoints dipancarkan: {all_waypoints}")
        else:
            print("[WaypointsPanel] Tidak ada waypoint untuk dikirim.")
