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
    QSpinBox,  # [BARU] Untuk input angka trigger
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

    # [BARU] Sinyal untuk update trigger inversi secara dinamis
    update_inversion_trigger = Signal(dict)

    def __init__(self, config, title="Waypoints"):
        super().__init__(title)
        self.config = config
        self.current_arena = None  # Menyimpan arena yang sedang aktif (A/B)

        main_layout = QVBoxLayout()

        # --- 1. Predefined Missions ---
        mission_box = QGroupBox("Predefined Missions")
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

        # --- 4. Photo Mission Box (Segmen) ---
        photo_mission_box = QGroupBox("Misi Fotografi (Segmen)")
        photo_mission_layout = QVBoxLayout()

        photo_form_layout = QFormLayout()
        self.wp_target1_input = QLineEdit()
        self.wp_target2_input = QLineEdit()
        self.photo_count_input = QLineEdit()

        int_validator = QIntValidator(self)
        self.wp_target1_input.setValidator(int_validator)
        self.wp_target2_input.setValidator(int_validator)
        self.photo_count_input.setValidator(int_validator)

        self.wp_target1_input.setPlaceholderText("Start Index (Contoh: 1)")
        self.wp_target2_input.setPlaceholderText("Stop Index (Contoh: 3)")
        self.photo_count_input.setPlaceholderText("Max Total Foto")

        photo_form_layout.addRow("Start WP Index:", self.wp_target1_input)
        photo_form_layout.addRow("Stop WP Index:", self.wp_target2_input)
        photo_form_layout.addRow("Max Foto:", self.photo_count_input)

        self.set_photo_mission_button = QPushButton("Set Segment Mission")
        self.set_photo_mission_button.setStyleSheet(
            "background-color: #DAA520; color: white; font-weight: bold;"
        )

        photo_mission_layout.addLayout(photo_form_layout)
        photo_mission_layout.addWidget(self.set_photo_mission_button)
        photo_mission_box.setLayout(photo_mission_layout)

        # --- 5. [BARU] Konfigurasi Trigger Inversi ---
        inversion_box = QGroupBox("Konfigurasi Inversi Servo")
        inversion_layout = QHBoxLayout()

        self.trigger_wp_input = QSpinBox()
        self.trigger_wp_input.setRange(1, 100)
        self.trigger_wp_input.setValue(8)  # Default ke WP 6
        self.trigger_wp_input.setPrefix("Trigger Inv di WP: ")

        self.set_trigger_btn = QPushButton("Set Trigger")
        self.set_trigger_btn.setStyleSheet(
            "background-color: #6A5ACD; color: white; font-weight: bold;"
        )

        inversion_layout.addWidget(self.trigger_wp_input)
        inversion_layout.addWidget(self.set_trigger_btn)
        inversion_box.setLayout(inversion_layout)

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
        main_layout.addWidget(photo_mission_box)
        main_layout.addWidget(inversion_box)  # [BARU] Tambahkan ke layout
        main_layout.addLayout(send_layout)

        self.setLayout(main_layout)

        # --- Koneksi Sinyal ---
        self.load_a_button.clicked.connect(lambda: self._on_load_mission("A"))
        self.load_b_button.clicked.connect(lambda: self._on_load_mission("B"))

        self.add_manual_button.clicked.connect(self.add_manual_waypoint)
        self.add_current_pos_button.clicked.connect(self.add_current_pos_requested.emit)
        self.delete_button.clicked.connect(self.delete_waypoint)
        self.send_all_button.clicked.connect(self.send_all_waypoints)
        self.set_photo_mission_button.clicked.connect(self._on_set_photo_mission)

        # [BARU] Koneksi tombol trigger inversi
        self.set_trigger_btn.clicked.connect(self._on_set_inversion_trigger)

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
        wp1_text = self.wp_target1_input.text()
        wp2_text = self.wp_target2_input.text()
        count_text = self.photo_count_input.text()

        if not wp1_text or not wp2_text or not count_text:
            print(
                "[WaypointsPanel] Error: Harap isi Start Index, Stop Index, dan Jumlah Foto."
            )
            return

        try:
            wp1 = int(wp1_text)
            wp2 = int(wp2_text)
            count = int(count_text)

            if count <= 0:
                print("[WaypointsPanel] Error: Jumlah Foto harus > 0.")
                return

            payload = {"wp1": wp1, "wp2": wp2, "count": count}
            self.send_photo_mission.emit(payload)
            print(
                f"[WaypointsPanel] Misi Segmen dikirim: Start={wp1}, Stop={wp2}, Max={count}"
            )

        except ValueError:
            print("[WaypointsPanel] Error: Input harus berupa angka bulat.")

    # --- [HANDLER BARU] ---
    @Slot()
    def _on_set_inversion_trigger(self):
        """Handler untuk mengirim konfigurasi trigger inversi."""
        # Ambil nilai (1-based dari UI)
        wp_target = self.trigger_wp_input.value()

        payload = {"index": wp_target}
        self.update_inversion_trigger.emit(payload)
        print(f"[WaypointsPanel] Request update trigger inversi ke WP {wp_target}")
