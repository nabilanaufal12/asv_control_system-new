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
    QMessageBox,
    QGridLayout,
)
from PySide6.QtCore import Signal, Slot
from PySide6.QtGui import QDoubleValidator

MAX_WAYPOINTS = 20


class WaypointsPanel(QGroupBox):
    # Sinyal komunikasi ke MainWindow/Backend
    send_waypoints = Signal(dict)  # Mengirim {'waypoints': [...], 'arena': 'A'/'B'}
    add_current_pos_requested = Signal()
    replace_with_live_gps_requested = Signal(int)
    arm_replace_rc_requested = Signal(int)
    load_mission_requested = Signal(str)
    request_wp_sync = Signal()  # Sinyal baru untuk meminta sync dari ESP32
    waypoints_updated = Signal(list)
    counter_action_requested = Signal(
        str
    )  # Sinyal untuk kontrol target WP (INC, DEC, RESET)

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
        btn_style_teal = (
            "background-color: #009688; color: white; font-weight: bold; padding: 5px;"
        )
        self.load_a_button.setStyleSheet(btn_style_teal)
        self.load_b_button.setStyleSheet(btn_style_teal)
        mission_layout.addWidget(self.load_a_button)
        mission_layout.addWidget(self.load_b_button)
        mission_box.setLayout(mission_layout)

        # --- 1.5. Waypoint Target Counter Controls ---
        counter_box = QGroupBox("Target Waypoint Override")
        counter_layout = QHBoxLayout()
        self.counter_dec_button = QPushButton("<< Prev WP (-)")
        self.counter_reset_button = QPushButton("Reset Target (0)")
        self.counter_inc_button = QPushButton("Next WP (+) >>")

        btn_style_counter = (
            "background-color: #607D8B; color: white; font-weight: bold; padding: 5px;"
        )
        self.counter_dec_button.setStyleSheet(btn_style_counter)
        self.counter_inc_button.setStyleSheet(btn_style_counter)
        self.counter_reset_button.setStyleSheet(
            "background-color: #E91E63; color: white; font-weight: bold; padding: 5px;"
        )

        counter_layout.addWidget(self.counter_dec_button)
        counter_layout.addWidget(self.counter_reset_button)
        counter_layout.addWidget(self.counter_inc_button)
        counter_box.setLayout(counter_layout)

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
        self.add_manual_button = QPushButton("Add Manual")
        self.add_manual_button.setStyleSheet(
            "background-color: #2196F3; color: white; font-weight: bold; padding: 5px;"
        )

        self.replace_manual_button = QPushButton("Replace Manual")
        self.replace_manual_button.setStyleSheet(
            "background-color: #FF9800; color: white; font-weight: bold; padding: 5px;"
        )

        self.add_current_pos_button = QPushButton("Add Current Pos")
        self.add_current_pos_button.setStyleSheet(
            "background-color: #2196F3; color: white; font-weight: bold; padding: 5px;"
        )
        self.delete_button = QPushButton("Delete")
        self.delete_button.setStyleSheet(
            "background-color: #F44336; color: white; font-weight: bold; padding: 5px;"
        )

        self.replace_gui_button = QPushButton("Replace (Live GPS)")
        self.replace_gui_button.setStyleSheet(
            "background-color: #FF9800; color: white; font-weight: bold; padding: 5px;"
        )

        self.arm_rc_button = QPushButton("Arm Replace (RC)")
        self.arm_rc_button.setStyleSheet(
            "background-color: #D32F2F; color: white; font-weight: bold; padding: 5px;"
        )

        self.send_all_button = QPushButton("Send All Waypoints")
        self.send_all_button.setStyleSheet(
            "background-color: #4CAF50; color: white; font-weight: bold; padding: 5px;"
        )

        self.request_sync_button = QPushButton("Request Sync")
        self.request_sync_button.setStyleSheet(
            "background-color: #9C27B0; color: white; font-weight: bold; padding: 5px;"
        )

        grid_layout = QGridLayout()

        # Baris 0: Penambahan Titik Baru
        grid_layout.addWidget(self.add_manual_button, 0, 0)
        grid_layout.addWidget(self.add_current_pos_button, 0, 1)

        # Baris 1: Penggantian (Replace)
        grid_layout.addWidget(self.replace_manual_button, 1, 0)
        grid_layout.addWidget(self.replace_gui_button, 1, 1)

        # Baris 2: Hapus & RC Overide
        grid_layout.addWidget(self.delete_button, 2, 0)
        grid_layout.addWidget(self.arm_rc_button, 2, 1)

        # Baris 3: Kirim Data / Sinkronisasi
        grid_layout.addWidget(self.send_all_button, 3, 0)
        grid_layout.addWidget(self.request_sync_button, 3, 1)

        # --- Menyusun Layout Utama ---
        main_layout.addWidget(mission_box)
        main_layout.addWidget(counter_box)
        main_layout.addLayout(input_form_layout)
        main_layout.addWidget(self.waypoints_list)
        main_layout.addLayout(grid_layout)

        self.setLayout(main_layout)

        # --- Koneksi Sinyal ---
        self.load_a_button.clicked.connect(lambda: self._on_load_mission("A"))
        self.load_b_button.clicked.connect(lambda: self._on_load_mission("B"))

        self.counter_inc_button.clicked.connect(
            lambda: self.counter_action_requested.emit("INC")
        )
        self.counter_dec_button.clicked.connect(
            lambda: self.counter_action_requested.emit("DEC")
        )
        self.counter_reset_button.clicked.connect(
            lambda: self.counter_action_requested.emit("RESET")
        )

        self.add_manual_button.clicked.connect(self.add_manual_waypoint)
        self.replace_manual_button.clicked.connect(self.replace_manual_waypoint)
        self.add_current_pos_button.clicked.connect(self.add_current_pos_requested.emit)
        self.delete_button.clicked.connect(self.delete_waypoint)
        self.send_all_button.clicked.connect(self.send_all_waypoints)
        self.request_sync_button.clicked.connect(self.request_wp_sync.emit)

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

        if len(waypoints) > MAX_WAYPOINTS:
            QMessageBox.warning(
                self,
                "Batas Waypoint",
                f"Jumlah waypoint melebihi batas maksimum ({MAX_WAYPOINTS}). Hanya {MAX_WAYPOINTS} waypoint pertama yang akan dimuat.",
            )
            waypoints = waypoints[:MAX_WAYPOINTS]

        for i, wp in enumerate(waypoints):
            waypoint_text = f"[{i}] Lat: {wp['lat']:.6f}, Lon: {wp['lon']:.6f}"
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
        if self.waypoints_list.count() >= MAX_WAYPOINTS:
            QMessageBox.warning(
                self,
                "Batas Waypoint",
                f"Maksimal {MAX_WAYPOINTS} waypoint telah tercapai.",
            )
            return

        if lat is not None and lon is not None and lat != 0.0:
            idx = self.waypoints_list.count()
            waypoint_text = f"[{idx}] Lat: {lat:.6f}, Lon: {lon:.6f}"
            self.waypoints_list.addItem(waypoint_text)
            self._emit_updated_waypoints()
        else:
            print("[GUI] Gagal menambah waypoint: Posisi saat ini tidak valid.")

    def add_manual_waypoint(self):
        if self.waypoints_list.count() >= MAX_WAYPOINTS:
            QMessageBox.warning(
                self,
                "Batas Waypoint",
                f"Maksimal {MAX_WAYPOINTS} waypoint telah tercapai.",
            )
            return

        lat_text = self.lat_input.text().replace(",", ".")
        lon_text = self.lon_input.text().replace(",", ".")
        if lat_text and lon_text:
            try:
                lat_float = float(lat_text)
                lon_float = float(lon_text)
                idx = self.waypoints_list.count()
                waypoint_text = f"[{idx}] Lat: {lat_float:.6f}, Lon: {lon_float:.6f}"
                self.waypoints_list.addItem(waypoint_text)
                self.lat_input.clear()
                self.lon_input.clear()
                self._emit_updated_waypoints()
                self.current_arena = None
            except ValueError:
                print("[GUI] Error: Input waypoint manual tidak valid.")

    def replace_manual_waypoint(self):
        selected_row = self.waypoints_list.currentRow()
        if selected_row < 0:
            QMessageBox.warning(
                self,
                "Pilih Waypoint",
                "Silakan pilih indeks waypoint di tabel terlebih dahulu.",
            )
            return

        lat_text = self.lat_input.text().replace(",", ".")
        lon_text = self.lon_input.text().replace(",", ".")
        if lat_text and lon_text:
            try:
                lat_float = float(lat_text)
                lon_float = float(lon_text)
                waypoint_text = (
                    f"[{selected_row}] Lat: {lat_float:.6f}, Lon: {lon_float:.6f}"
                )

                item = self.waypoints_list.item(selected_row)
                item.setText(waypoint_text)

                self.lat_input.clear()
                self.lon_input.clear()
                self._emit_updated_waypoints()
                self.send_all_waypoints()  # Langsung sync ke ESP32
                print(
                    f"[GUI] Titik {selected_row} diganti dengan Manual Input dan disinkronkan."
                )
            except ValueError:
                print("[GUI] Error: Input waypoint manual tidak valid.")

    def delete_waypoint(self):
        selected_items = self.waypoints_list.selectedItems()
        if not selected_items:
            return
        for item in selected_items:
            self.waypoints_list.takeItem(self.waypoints_list.row(item))
        self._reindex_waypoints()
        self._emit_updated_waypoints()
        self.send_all_waypoints()  # Langsung sync ke ESP32
        print("[GUI] Titik waypoint dihapus dan disinkronkan.")

    def _get_all_waypoints_from_list(self):
        all_waypoints = []
        for i in range(self.waypoints_list.count()):
            item_text = self.waypoints_list.item(i).text()
            try:
                # Hapus prefix indeks "[N] " jika ada
                clean_text = item_text
                if clean_text.startswith("["):
                    clean_text = clean_text.split("] ", 1)[1]
                parts = clean_text.replace(" ", "").split(",")
                lat = float(parts[0].split(":")[1])
                lon = float(parts[1].split(":")[1])
                all_waypoints.append({"lat": lat, "lon": lon})
            except (ValueError, IndexError):
                print(f"[GUI] Gagal mem-parsing item waypoint: {item_text}")
        return all_waypoints

    def _reindex_waypoints(self):
        """Perbarui nomor indeks di setiap item waypoint setelah delete."""
        for i in range(self.waypoints_list.count()):
            item = self.waypoints_list.item(i)
            text = item.text()
            # Hapus prefix lama jika ada
            if text.startswith("["):
                text = text.split("] ", 1)[1]
            item.setText(f"[{i}] {text}")

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
