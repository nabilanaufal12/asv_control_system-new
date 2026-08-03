# gui/components/waypoints_panel.py
# --- VERSI MODIFIKASI FINAL: Arena ID + Misi Foto + Dynamic Inversion Trigger ---

from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QLineEdit,
    QPushButton,
    QListWidget,
    QAbstractItemView,
    QFormLayout,
)
from PySide6.QtCore import Signal, Slot
from PySide6.QtGui import QDoubleValidator, QIntValidator


class WaypointsPanel(QGroupBox):
    # Sinyal komunikasi ke MainWindow/Backend
    send_waypoints = Signal(dict)  # Mengirim {'waypoints': [...], 'arena': 'A'/'B'}
    add_current_pos_requested = Signal()
    waypoints_updated = Signal(list)
    load_mission_requested = Signal(str)
    send_photo_mission = Signal(dict)

    send_photo_mission = Signal(dict)

    replace_with_live_gps_requested = Signal(int)
    arm_replace_rc_requested = Signal(int)

    def __init__(self, config, title="Navigation & Waypoint Setup"):
        super().__init__(title)
        self.config = config
        self.current_arena = None  # Menyimpan arena yang sedang aktif (A/B)

        main_layout = QVBoxLayout()

        # --- 1. Predefined Missions ---
        mission_box = QGroupBox("Mission Trajectory Templates")
        mission_layout = QHBoxLayout()
        self.load_a_button = QPushButton("Load Lintasan A")
        self.load_b_button = QPushButton("Load Lintasan B")
        mission_layout.addWidget(self.load_a_button)
        mission_layout.addWidget(self.load_b_button)
        mission_box.setLayout(mission_layout)

        # --- 2. Manual Input Form ---
        input_form_layout = QFormLayout()
        self.lat_input = QLineEdit()
        self.lon_input = QLineEdit()

        gui_settings = self.config.get("gui_settings", {})
        placeholders = gui_settings.get("placeholders", {})
        lat_placeholder = placeholders.get("latitude", "e.g., -6.9175")
        lon_placeholder = placeholders.get("longitude", "e.g., 107.6191")

        self.lat_input.setPlaceholderText(lat_placeholder)
        self.lon_input.setPlaceholderText(lon_placeholder)

        validator_lat = QDoubleValidator(-90.0, 90.0, 6, self)
        self.lat_input.setValidator(validator_lat)
        validator_lon = QDoubleValidator(-180.0, 180.0, 6, self)
        self.lon_input.setValidator(validator_lon)

        input_form_layout.addRow("Latitude:", self.lat_input)
        input_form_layout.addRow("Longitude:", self.lon_input)

        # --- 3. Waypoints List & Buttons ---
        self.waypoints_list = QListWidget()
        self.waypoints_list.setSelectionMode(QAbstractItemView.SingleSelection)

        button_layout = QHBoxLayout()
        self.add_manual_button = QPushButton("Add Manual")
        self.add_current_pos_button = QPushButton("Add Current Pos")
        self.delete_button = QPushButton("Delete")
        button_layout.addWidget(self.add_manual_button)
        button_layout.addWidget(self.add_current_pos_button)
        button_layout.addWidget(self.delete_button)

        edit_button_layout = QHBoxLayout()
        self.replace_gui_button = QPushButton("Replace (Live GPS)")
        self.replace_gui_button.setStyleSheet(
            "background-color: #2E8B57; color: white;"
        )
        self.arm_rc_button = QPushButton("Arm Replace (RC)")
        self.arm_rc_button.setStyleSheet("background-color: #DC143C; color: white;")
        edit_button_layout.addWidget(self.replace_gui_button)
        edit_button_layout.addWidget(self.arm_rc_button)

        # --- 4. Photo Mission Box (Segmen) ---
        photo_mission_box = QGroupBox("Photography Mission Segments")
        photo_mission_layout = QVBoxLayout()

        photo_form_layout = QFormLayout()
        self.surf_wp1_input = QLineEdit("13")
        self.surf_wp2_input = QLineEdit("14")
        self.under_wp1_input = QLineEdit("11")
        self.under_wp2_input = QLineEdit("12")
        self.photo_count_input = QLineEdit("5")

        int_validator = QIntValidator(self)
        self.surf_wp1_input.setValidator(int_validator)
        self.surf_wp2_input.setValidator(int_validator)
        self.under_wp1_input.setValidator(int_validator)
        self.under_wp2_input.setValidator(int_validator)
        self.photo_count_input.setValidator(int_validator)

        self.surf_wp1_input.setPlaceholderText("Surface Start")
        self.surf_wp2_input.setPlaceholderText("Surface Stop")
        self.under_wp1_input.setPlaceholderText("Underwater Start")
        self.under_wp2_input.setPlaceholderText("Underwater Stop")
        self.photo_count_input.setPlaceholderText("Max Total Foto")

        photo_form_layout.addRow("Surface Start WP:", self.surf_wp1_input)
        photo_form_layout.addRow("Surface Stop WP:", self.surf_wp2_input)
        photo_form_layout.addRow("Underwater Start WP:", self.under_wp1_input)
        photo_form_layout.addRow("Underwater Stop WP:", self.under_wp2_input)
        photo_form_layout.addRow("Max Foto (Masing-masing):", self.photo_count_input)

        self.set_photo_mission_button = QPushButton("Set Segment Mission")
        self.set_photo_mission_button.setStyleSheet(
            "background-color: #DAA520; color: white; font-weight: bold;"
        )

        photo_mission_layout.addLayout(photo_form_layout)
        photo_mission_layout.addWidget(self.set_photo_mission_button)
        photo_mission_box.setLayout(photo_mission_layout)

        # --- 5. [DIHAPUS] Konfigurasi Trigger Inversi ---
        # Fitur ini dihapus sesuai instruksi

        # --- 6. Send All Button ---
        send_layout = QHBoxLayout()
        self.send_all_button = QPushButton("Send All Waypoints")
        self.send_all_button.setStyleSheet(
            "background-color: #2a82da; color: white; font-weight: bold;"
        )
        send_layout.addStretch()
        send_layout.addWidget(self.send_all_button)

        # --- Menyusun Layout Utama ---
        main_layout.addWidget(mission_box)
        main_layout.addLayout(input_form_layout)
        main_layout.addWidget(self.waypoints_list)
        main_layout.addLayout(button_layout)
        main_layout.addLayout(edit_button_layout)

        main_layout.addLayout(send_layout)

        main_layout.addWidget(photo_mission_box)

        self.setLayout(main_layout)

        # --- Koneksi Sinyal ---
        self.load_a_button.clicked.connect(lambda: self._on_load_mission("A"))
        self.load_b_button.clicked.connect(lambda: self._on_load_mission("B"))

        self.add_manual_button.clicked.connect(self.add_manual_waypoint)
        self.add_current_pos_button.clicked.connect(self.add_current_pos_requested.emit)
        self.delete_button.clicked.connect(self.delete_waypoint)
        self.send_all_button.clicked.connect(self.send_all_waypoints)
        self.set_photo_mission_button.clicked.connect(self._on_set_photo_mission)

        self.replace_gui_button.clicked.connect(self._on_replace_gui)
        self.arm_rc_button.clicked.connect(self._on_arm_rc)

    def _on_replace_gui(self):
        selected = self.waypoints_list.currentRow()
        if selected >= 0:
            self.replace_with_live_gps_requested.emit(selected)
        else:
            print("[GUI] Error: Silakan pilih waypoint di tabel terlebih dahulu.")

    def _on_arm_rc(self):
        selected = self.waypoints_list.currentRow()
        if selected >= 0:
            self.arm_replace_rc_requested.emit(selected)
            print(f"[GUI] Target bidikan WP {selected} diaktifkan untuk RC.")
        else:
            print("[GUI] Error: Silakan pilih waypoint di tabel terlebih dahulu.")

    def _on_load_mission(self, arena_id):
        self.current_arena = arena_id
        self.load_mission_requested.emit(arena_id)

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

    @Slot(list)
    def sync_waypoints_from_backend(self, waypoints):
        """Menerima data sinkronisasi waypoint dari backend (ESP32) dan mengisinya ke UI."""
        print(f"[GUI] Sinkronisasi {len(waypoints)} waypoint dari ESP32...")
        self.load_waypoints_to_list(waypoints)

    @Slot(float, float)
    def add_waypoint_from_pos(self, lat, lon):
        if lat is not None and lon is not None and lat != 0.0:
            waypoint_text = f"Lat: {lat:.6f}, Lon: {lon:.6f}"
            self.waypoints_list.addItem(waypoint_text)
            self._emit_updated_waypoints()
        else:
            print("[GUI] Gagal menambah waypoint: Posisi saat ini tidak valid.")

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
                self.current_arena = None
            except ValueError:
                print("[GUI] Error: Input waypoint manual tidak valid.")

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
                print(f"[GUI] Gagal mem-parsing item waypoint: {item_text}")
        return all_waypoints

    def send_all_waypoints(self):
        all_waypoints = self._get_all_waypoints_from_list()

        if self.current_arena is None:
            print(
                "[WaypointsPanel] Peringatan: Tidak ada arena (A/B) yang dipilih/di-load."
            )
            current_arena_to_send = "A"  # Default ke A
        else:
            current_arena_to_send = self.current_arena

        payload = {"waypoints": all_waypoints, "arena": current_arena_to_send}
        self.send_waypoints.emit(payload)
        print(
            f"[WaypointsPanel] Sinyal send_waypoints dipancarkan: {len(all_waypoints)} waypoints, Arena: {current_arena_to_send}"
        )

    @Slot()
    def _on_set_photo_mission(self):
        """Handler untuk set misi foto segmen."""
        surf1_text = self.surf_wp1_input.text()
        surf2_text = self.surf_wp2_input.text()
        under1_text = self.under_wp1_input.text()
        under2_text = self.under_wp2_input.text()
        count_text = self.photo_count_input.text()

        if (
            not surf1_text
            or not surf2_text
            or not under1_text
            or not under2_text
            or not count_text
        ):
            print(
                "[WaypointsPanel] Error: Harap isi semua field indeks dan Jumlah Foto."
            )
            return

        try:
            surf1 = int(surf1_text)
            surf2 = int(surf2_text)
            under1 = int(under1_text)
            under2 = int(under2_text)
            count = int(count_text)

            if count <= 0:
                print("[WaypointsPanel] Error: Jumlah Foto harus > 0.")
                return

            payload = {
                "surf_wp1": surf1,
                "surf_wp2": surf2,
                "under_wp1": under1,
                "under_wp2": under2,
                "count": count,
            }
            self.send_photo_mission.emit(payload)
            print(f"[WaypointsPanel] Sinyal send_photo_mission dipancarkan: {payload}")
        except ValueError:
            print("[WaypointsPanel] Error: Input indeks / jumlah foto tidak valid.")
