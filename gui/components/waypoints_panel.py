# gui/components/waypoints_panel.py
# --- VERSI FINAL: Dengan tata letak form yang lebih fleksibel ---

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QListWidget, QAbstractItemView, QFormLayout # <-- Impor QFormLayout
)
from PySide6.QtCore import Signal, Slot
from PySide6.QtGui import QDoubleValidator


class WaypointsPanel(QGroupBox):
    send_waypoints = Signal(list)
    add_current_pos_requested = Signal()
    waypoints_updated = Signal(list)
    load_mission_requested = Signal(str)

    def __init__(self, config, title="Waypoints"):
        super().__init__(title)
        self.config = config
        main_layout = QVBoxLayout()

        mission_box = QGroupBox("Predefined Missions")
        mission_layout = QHBoxLayout()
        self.load_a_button = QPushButton("Load Lintasan A")
        self.load_b_button = QPushButton("Load Lintasan B")
        mission_layout.addWidget(self.load_a_button)
        mission_layout.addWidget(self.load_b_button)
        mission_box.setLayout(mission_layout)

        # --- PERBAIKAN DI SINI: Gunakan QFormLayout untuk input Latitude/Longitude ---
        input_form_layout = QFormLayout()
        self.lat_input = QLineEdit()
        self.lat_input.setPlaceholderText("e.g., -6.2100")
        self.lon_input = QLineEdit()
        self.lon_input.setPlaceholderText("e.g., 106.8400")
        
        validator_lat = QDoubleValidator(-90.0, 90.0, 6, self)
        self.lat_input.setValidator(validator_lat)
        validator_lon = QDoubleValidator(-180.0, 180.0, 6, self)
        self.lon_input.setValidator(validator_lon)
        
        input_form_layout.addRow("Latitude:", self.lat_input)
        input_form_layout.addRow("Longitude:", self.lon_input)
        # --- AKHIR PERBAIKAN ---

        self.waypoints_list = QListWidget()
        self.waypoints_list.setSelectionMode(QAbstractItemView.SingleSelection)

        button_layout = QHBoxLayout()
        self.add_manual_button = QPushButton("Add Manual")
        self.add_current_pos_button = QPushButton("Add Current Pos")
        self.delete_button = QPushButton("Delete")
        button_layout.addWidget(self.add_manual_button)
        button_layout.addWidget(self.add_current_pos_button)
        button_layout.addWidget(self.delete_button)

        send_layout = QHBoxLayout()
        self.send_all_button = QPushButton("Send All")
        self.send_all_button.setStyleSheet(
            "background-color: #2a82da; color: white; font-weight: bold;"
        )
        send_layout.addStretch()
        send_layout.addWidget(self.send_all_button)

        main_layout.addWidget(mission_box)
        main_layout.addLayout(input_form_layout) # <-- Gunakan layout form yang baru
        main_layout.addWidget(self.waypoints_list)
        main_layout.addLayout(button_layout)
        main_layout.addLayout(send_layout)
        self.setLayout(main_layout)

        # Hubungkan sinyal tombol
        self.load_a_button.clicked.connect(lambda: self.load_mission_requested.emit("A"))
        self.load_b_button.clicked.connect(lambda: self.load_mission_requested.emit("B"))
        self.add_manual_button.clicked.connect(self.add_manual_waypoint)
        self.add_current_pos_button.clicked.connect(self.add_current_pos_requested.emit)
        self.delete_button.clicked.connect(self.delete_waypoint)
        self.send_all_button.clicked.connect(self.send_all_waypoints)

    @Slot(list)
    def load_waypoints_to_list(self, waypoints):
        """Menghapus daftar saat ini dan mengisinya dengan waypoint baru."""
        self.waypoints_list.clear()
        for wp in waypoints:
            waypoint_text = f"Lat: {wp['lat']:.6f}, Lon: {wp['lon']:.6f}"
            self.waypoints_list.addItem(waypoint_text)
        self._emit_updated_waypoints()

    def _emit_updated_waypoints(self):
        current_waypoints = self._get_all_waypoints_from_list()
        self.waypoints_updated.emit(current_waypoints)

    @Slot(float, float)
    def add_waypoint_from_pos(self, lat, lon):
        if lat is not None and lon is not None:
            waypoint_text = f"Lat: {lat:.6f}, Lon: {lon:.6f}"
            self.waypoints_list.addItem(waypoint_text)
            self._emit_updated_waypoints()

    def add_manual_waypoint(self):
        lat_text = self.lat_input.text().replace(",", ".")
        lon_text = self.lon_input.text().replace(",", ".")
        if lat_text and lon_text:
            try:
                lat_float = float(lat_text)
                lon_float = float(lon_text)
                waypoint_text = f"Lat: {lat_float:.6f}, Lon: {lon_float:.6f}"
                self.waypoints_list.addItem(waypoint_text)
                self.lat_input.clear()
                self.lon_input.clear()
                self._emit_updated_waypoints()
            except ValueError:
                print("Error: Input waypoint tidak valid.")

    def delete_waypoint(self):
        selected_items = self.waypoints_list.selectedItems()
        if not selected_items:
            return
        for item in selected_items:
            self.waypoints_list.takeItem(self.waypoints_list.row(item))
        self._emit_updated_waypoints()

    def _get_all_waypoints_from_list(self):
        all_waypoints = []
        for i in range(self.waypoints_list.count()):
            item_text = self.waypoints_list.item(i).text()
            try:
                parts = item_text.replace(" ", "").split(",")
                lat = float(parts[0].split(":")[1])
                lon = float(parts[1].split(":")[1])
                all_waypoints.append({"lat": lat, "lon": lon})
            except (ValueError, IndexError):
                print(f"Gagal mem-parsing item waypoint: {item_text}")
        return all_waypoints

    def send_all_waypoints(self):
        all_waypoints = self._get_all_waypoints_from_list()
        if all_waypoints:
            self.send_waypoints.emit(all_waypoints)
        else:
            print("[WaypointsPanel] Tidak ada waypoint untuk dikirim.")