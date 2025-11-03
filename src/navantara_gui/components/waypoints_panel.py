# gui/components/waypoints_panel.py
# --- VERSI MODIFIKASI: Mengirim arena ID bersama waypoints AND Tambahan Misi Foto ---

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
# --- [MODIFIKASI 1: Tambahkan QIntValidator] ---
from PySide6.QtGui import QDoubleValidator, QIntValidator
# --- [AKHIR MODIFIKASI 1] ---


class WaypointsPanel(QGroupBox):
    # --- MODIFIKASI 1 (dari file asli): Ubah tipe sinyal ke dict ---
    send_waypoints = Signal(
        dict
    )  # Sekarang mengirim {'waypoints': [...], 'arena': 'A'/'B'}
    # ---------------------------------------------
    add_current_pos_requested = Signal()
    waypoints_updated = Signal(list)
    load_mission_requested = Signal(str)
    # --- [MODIFIKASI 2: Tambahkan sinyal Misi Foto] ---
    send_photo_mission = Signal(dict)
    # --- [AKHIR MODIFIKASI 2] ---

    def __init__(self, config, title="Waypoints"):
        super().__init__(title)
        self.config = config
        # --- MODIFIKASI 2 (dari file asli): Tambahkan variabel untuk menyimpan arena aktif ---
        self.current_arena = None  # Awalnya belum ada arena yang dipilih
        # -----------------------------------------------------------------
        main_layout = QVBoxLayout()

        mission_box = QGroupBox("Predefined Missions")
        mission_layout = QHBoxLayout()
        self.load_a_button = QPushButton("Load Lintasan A")
        self.load_b_button = QPushButton("Load Lintasan B")
        mission_layout.addWidget(self.load_a_button)
        mission_layout.addWidget(self.load_b_button)
        mission_box.setLayout(mission_layout)

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

        self.waypoints_list = QListWidget()
        self.waypoints_list.setSelectionMode(QAbstractItemView.SingleSelection)

        button_layout = QHBoxLayout()
        self.add_manual_button = QPushButton("Add Manual")
        self.add_current_pos_button = QPushButton("Add Current Pos")
        self.delete_button = QPushButton("Delete")
        button_layout.addWidget(self.add_manual_button)
        button_layout.addWidget(self.add_current_pos_button)
        button_layout.addWidget(self.delete_button)

        # --- [MODIFIKASI 3: UI Misi Foto] ---
        photo_mission_box = QGroupBox("Misi Fotografi Otomatis")
        photo_mission_layout = QVBoxLayout()
        
        photo_form_layout = QFormLayout()
        self.wp_target_input = QLineEdit()
        self.photo_count_input = QLineEdit()
        
        # Validasi input agar hanya angka integer
        int_validator = QIntValidator(self)
        self.wp_target_input.setValidator(int_validator)
        self.photo_count_input.setValidator(int_validator)

        self.wp_target_input.setPlaceholderText("Indeks WP (misal: 1)")
        self.photo_count_input.setPlaceholderText("Jumlah foto (misal: 3)")
        
        photo_form_layout.addRow("Target WP:", self.wp_target_input)
        photo_form_layout.addRow("Jumlah Foto:", self.photo_count_input)
        
        self.set_photo_mission_button = QPushButton("Set Photo Mission")
        self.set_photo_mission_button.setStyleSheet(
            "background-color: #DAA520; color: white; font-weight: bold;" # Warna kuning/emas
        )

        photo_mission_layout.addLayout(photo_form_layout)
        photo_mission_layout.addWidget(self.set_photo_mission_button)
        photo_mission_box.setLayout(photo_mission_layout)
        # --- [AKHIR MODIFIKASI 3] ---

        send_layout = QHBoxLayout()
        self.send_all_button = QPushButton("Send All Waypoints")
        self.send_all_button.setStyleSheet(
            "background-color: #2a82da; color: white; font-weight: bold;"
        )
        send_layout.addStretch()
        send_layout.addWidget(self.send_all_button)

        # --- [MODIFIKASI 4: Tambahkan widget ke layout utama] ---
        main_layout.addWidget(mission_box)
        main_layout.addLayout(input_form_layout)
        main_layout.addWidget(self.waypoints_list)
        main_layout.addLayout(button_layout)
        main_layout.addWidget(photo_mission_box) # <-- BARU
        main_layout.addLayout(send_layout)
        # --- [AKHIR MODIFIKASI 4] ---
        self.setLayout(main_layout)

        # Hubungkan sinyal tombol
        # --- MODIFIKASI 3 (dari file asli): Update arena saat tombol load ditekan ---
        self.load_a_button.clicked.connect(lambda: self._on_load_mission("A"))
        self.load_b_button.clicked.connect(lambda: self._on_load_mission("B"))
        # ---------------------------------------------------------
        self.add_manual_button.clicked.connect(self.add_manual_waypoint)
        self.add_current_pos_button.clicked.connect(self.add_current_pos_requested.emit)
        self.delete_button.clicked.connect(self.delete_waypoint)
        self.send_all_button.clicked.connect(self.send_all_waypoints)
        # --- [MODIFIKASI 5: Hubungkan sinyal tombol baru] ---
        self.set_photo_mission_button.clicked.connect(self._on_set_photo_mission)
        # --- [AKHIR MODIFIKASI 5] ---

    # --- MODIFIKASI 4 (dari file asli): Fungsi baru untuk menangani load mission ---
    def _on_load_mission(self, arena_id):
        self.current_arena = arena_id  # Simpan arena yang di-load
        self.load_mission_requested.emit(arena_id)  # Minta MainWindow memuat data

    # -------------------------------------------------------------

    @Slot(list)
    def load_waypoints_to_list(self, waypoints):
        """Menghapus daftar saat ini dan mengisinya dengan waypoint baru."""
        # Arena sudah di-set oleh _on_load_mission
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
        """Menerima koordinat dan menambahkannya sebagai item baru di QListWidget."""
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
                # --- MODIFIKASI 5 (dari file asli): Reset arena jika menambah manual? (Opsional) ---
                # self.current_arena = None # Atau set ke 'MANUAL'?
                # -----------------------------------------------------------------
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

    # --- MODIFIKASI 6 (dari file asli): Kirim payload dictionary ---
    def send_all_waypoints(self):
        all_waypoints = self._get_all_waypoints_from_list()
        if not all_waypoints:
            print("[WaypointsPanel] Tidak ada waypoint untuk dikirim.")
            return
        if self.current_arena is None:
            print(
                "[WaypointsPanel] Peringatan: Tidak ada arena (A/B) yang dipilih/di-load."
            )
            # Anda bisa memutuskan untuk mengirim None, atau default ke 'A', atau tidak mengirim sama sekali
            # current_arena_to_send = None
            current_arena_to_send = "A"  # Default ke A jika belum diset
        else:
            current_arena_to_send = self.current_arena

        payload = {"waypoints": all_waypoints, "arena": current_arena_to_send}
        self.send_waypoints.emit(payload)
        print(f"[WaypointsPanel] Sinyal send_waypoints dipancarkan: {payload}")

    # ---------------------------------------------

    # --- [MODIFIKASI 6: Fungsi handler baru untuk Misi Foto] ---
    @Slot()
    def _on_set_photo_mission(self):
        """Mengirim perintah untuk mengatur misi fotografi otomatis."""
        wp_idx_text = self.wp_target_input.text()
        count_text = self.photo_count_input.text()

        if not wp_idx_text or not count_text:
            print("[WaypointsPanel] Error: Input Misi Foto tidak boleh kosong.")
            return

        try:
            wp_idx = int(wp_idx_text)
            count = int(count_text)

            # WP 0 adalah indeks yang valid (misal, RTH)
            if wp_idx < 0 or count <= 0:
                print("[WaypointsPanel] Error: WP harus >= 0 dan Jumlah Foto harus > 0.")
                return

            payload = {"wp_index": wp_idx, "count": count}
            self.send_photo_mission.emit(payload)
            print(f"[WaypointsPanel] Sinyal send_photo_mission dipancarkan: {payload}")
            
            # Anda bisa memilih untuk mengosongkan input setelah dikirim
            # self.wp_target_input.clear()
            # self.photo_count_input.clear()

        except ValueError:
            print("[WaypointsPanel] Error: Input Misi Foto tidak valid (bukan angka).")
    # --- [AKHIR MODIFIKASI 6] ---