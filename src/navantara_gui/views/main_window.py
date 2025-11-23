# src/navantara_gui/views/main_window.py
import sys
import os
import logging  # <- Tambahkan import logging jika belum ada

# Blok ini memperbaiki path agar impor dari folder lain berhasil
try:
    project_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    )
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
except NameError:
    sys.path.insert(0, ".")

from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QTabWidget,
    QStatusBar,
    QScrollArea,
    QApplication,
    QSplitter,
)
from PySide6.QtCore import Slot, Qt

from navantara_gui.components.control_panel import ControlPanel
from navantara_gui.components.dashboard import Dashboard
from navantara_gui.components.settings_panel import SettingsPanel
from navantara_gui.components.video_view import VideoView
from navantara_gui.components.map_view import MapView
from navantara_gui.components.header import Header
from navantara_gui.components.waypoints_panel import WaypointsPanel
from navantara_gui.components.log_panel import LogPanel
from navantara_gui.missions import get_lintasan_a, get_lintasan_b
from navantara_gui.api_client import ApiClient


class MainWindow(QMainWindow):
    def __init__(self, config):
        super().__init__()
        self.setWindowTitle("ASV Control System - Navantara Client")
        self.config = config

        self.api_client = ApiClient(config=self.config)

        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.current_control_mode = "MANUAL"
        self.is_rc_override = False  # Lacak status override RC

        self.header = Header(config=self.config)
        self.control_panel = ControlPanel(config=self.config)
        self.system_status_panel = Dashboard(config=self.config)
        self.settings_panel = SettingsPanel(config=self.config)
        self.video_view = VideoView(config=self.config)
        self.map_view = MapView(config=self.config)
        self.waypoints_panel = WaypointsPanel(config=self.config)
        self.log_panel = LogPanel(config=self.config)
        self.active_manual_keys = set()

        self.current_theme = "light"
        self.themes = {}
        self._load_themes()
        self._apply_theme(self.current_theme)

        self.setup_ui()
        self.connect_signals()

        self.set_mode("MANUAL")  # Atur mode awal saat start

        print("Memulai koneksi klien API ke server...")
        self.api_client.connect_to_server()

        self.showMaximized()

    # ... (fungsi _load_themes, _apply_theme, toggle_theme, setup_ui tidak berubah) ...
    def _load_themes(self):
        try:
            gui_dir = os.path.dirname(os.path.abspath(__file__))
            dark_theme_path = os.path.join(
                gui_dir, "..", "assets", "resources", "dark_theme.qss"
            )
            with open(dark_theme_path, "r") as f:
                self.themes["dark"] = f.read()
            light_theme_path = os.path.join(
                gui_dir, "..", "assets", "resources", "light_theme.qss"
            )
            with open(light_theme_path, "r") as f:
                self.themes["light"] = f.read()
        except Exception as e:
            print(f"Peringatan: Gagal memuat file tema. Error: {e}")

    def _apply_theme(self, theme_name):
        if theme_name in self.themes:
            QApplication.instance().setStyleSheet(self.themes[theme_name])
            button_text = (
                "Switch to Light Mode"
                if theme_name == "dark"
                else "Switch to Dark Mode"
            )
            self.header.theme_button.setText(button_text)
            self.current_theme = theme_name

    @Slot()
    def toggle_theme(self):
        self._apply_theme("light" if self.current_theme == "dark" else "dark")

    def setup_ui(self):
        # (Fungsi ini tidak berubah)
        layout_sidebar_kiri = QVBoxLayout()
        layout_sidebar_kiri.addWidget(self.control_panel)
        layout_sidebar_kiri.addWidget(self.settings_panel)
        layout_sidebar_kiri.addStretch()
        widget_sidebar_kiri = QWidget()
        widget_sidebar_kiri.setLayout(layout_sidebar_kiri)
        scroll_area_kiri = QScrollArea()
        scroll_area_kiri.setWidget(widget_sidebar_kiri)
        scroll_area_kiri.setWidgetResizable(True)
        scroll_area_kiri.setFrameShape(QScrollArea.NoFrame)
        scroll_area_kiri.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.tab_tengah = QTabWidget()
        self.tab_tengah.addTab(self.video_view, "Video Stream")
        self.tab_tengah.addTab(self.map_view, "Map View")
        layout_sidebar_kanan = QVBoxLayout()
        layout_sidebar_kanan.addWidget(self.waypoints_panel)
        layout_sidebar_kanan.addWidget(self.system_status_panel)
        layout_sidebar_kanan.addWidget(self.log_panel)
        layout_sidebar_kanan.addStretch()
        widget_sidebar_kanan = QWidget()
        widget_sidebar_kanan.setLayout(layout_sidebar_kanan)
        scroll_area_kanan = QScrollArea()
        scroll_area_kanan.setWidget(widget_sidebar_kanan)
        scroll_area_kanan.setWidgetResizable(True)
        scroll_area_kanan.setFrameShape(QScrollArea.NoFrame)
        scroll_area_kanan.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.addWidget(scroll_area_kiri)
        main_splitter.addWidget(self.tab_tengah)
        main_splitter.addWidget(scroll_area_kanan)
        gui_settings = self.config.get("gui_settings", {})
        splitter_sizes = gui_settings.get("main_splitter_sizes", [350, 800, 350])
        main_splitter.setSizes(splitter_sizes)
        main_splitter.setCollapsible(0, False)
        main_splitter.setCollapsible(2, False)
        layout_keseluruhan = QVBoxLayout()
        layout_keseluruhan.addWidget(self.header, 0)
        layout_keseluruhan.addWidget(main_splitter, 1)
        widget_pusat = QWidget()
        widget_pusat.setLayout(layout_keseluruhan)
        self.setCentralWidget(widget_pusat)
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Aplikasi Siap. Menunggu koneksi ke backend...")

    def connect_signals(self):
        """Menghubungkan semua sinyal dan slot antar komponen."""
        self.header.theme_changed_requested.connect(self.toggle_theme)

        self.video_view.toggle_camera_requested.connect(
            self.api_client.request_data_stream
        )

        self.settings_panel.debug_command_sent.connect(self.api_client.send_command)

        # --- PERUBAHAN UTAMA: Hubungkan sinyal dari ControlPanel ke fungsi logika ---
        self.control_panel.manual_button_clicked.connect(
            lambda: self.set_mode("MANUAL")
        )
        self.control_panel.auto_button_clicked.connect(lambda: self.set_mode("AUTO"))

        self.settings_panel.connect_requested.connect(
            lambda details: self.api_client.send_command("CONFIGURE_SERIAL", details)
        )
        self.waypoints_panel.send_waypoints.connect(
            lambda wps: self.api_client.send_command("SET_WAYPOINTS", wps)
        )
        self.waypoints_panel.add_current_pos_requested.connect(self.on_add_current_pos)
        self.api_client.connection_status_changed.connect(
            self.on_connection_status_change
        )
        self.api_client.data_updated.connect(self.on_data_updated)
        self.waypoints_panel.load_mission_requested.connect(
            self.load_predefined_mission
        )
        self.api_client.frame_cam1_updated.connect(self.video_view.update_frame_1)
        self.api_client.frame_cam2_updated.connect(self.video_view.update_frame_2)
        self.waypoints_panel.waypoints_updated.connect(self.map_view.update_waypoints)

        # --- [MODIFIKASI TAMBAHAN]: Hubungkan sinyal misi foto dari waypoints_panel ---
        # (Sepertinya ini hilang di file Anda, saya tambahkan kembali)
        self.waypoints_panel.send_photo_mission.connect(
            lambda payload: self.api_client.send_command("SET_PHOTO_MISSION", payload)
        )

        # --- [MODIFIKASI BARU UNTUK MANUAL CAPTURE] ---
        # Hubungkan sinyal BARU dari control_panel ke slot/fungsi baru
        self.control_panel.capture_surface_clicked.connect(
            self.on_request_manual_capture_surface
        )
        self.control_panel.capture_underwater_clicked.connect(
            self.on_request_manual_capture_underwater
        )
        # --- [AKHIR MODIFIKASI BARU] ---

    @Slot(str)
    def set_mode(self, mode):
        """Fungsi terpusat untuk mengubah mode operasi."""
        self.current_control_mode = mode
        self.api_client.send_command("CHANGE_MODE", {"mode": mode})

        is_manual = mode == "MANUAL"

        # Perbarui tampilan tombol
        self.control_panel.manual_mode_btn.setChecked(is_manual)
        self.control_panel.auto_mode_btn.setChecked(not is_manual)

        # Kelola status aktif/nonaktif tombol, dengan mempertimbangkan override RC
        self.update_button_states()

        self.setFocus()  # Rebut kembali fokus keyboard

    def update_button_states(self):
        """Memperbarui status enabled/disabled semua tombol berdasarkan mode dan override RC."""
        is_manual = self.current_control_mode == "MANUAL"

        # Tombol mode hanya bisa diubah jika RC tidak override
        self.control_panel.manual_mode_btn.setEnabled(not self.is_rc_override)
        self.control_panel.auto_mode_btn.setEnabled(not self.is_rc_override)

        # Tombol navigasi aktif jika mode AUTO dan RC tidak override
        self.control_panel.start_mission_btn.setEnabled(
            not is_manual and not self.is_rc_override
        )
        self.control_panel.pause_mission_btn.setEnabled(
            not is_manual and not self.is_rc_override
        )
        self.control_panel.return_home_btn.setEnabled(
            not is_manual and not self.is_rc_override
        )

        # Tombol WASD aktif jika mode MANUAL dan RC tidak override
        for button in self.control_panel.key_buttons.values():
            button.setEnabled(is_manual and not self.is_rc_override)

    @Slot(dict)
    def on_data_updated(self, data):
        self.current_latitude = data.get("latitude", self.current_latitude)
        self.current_longitude = data.get("longitude", self.current_longitude)

        status_text = data.get("status", "")
        self.is_rc_override = "RC MANUAL OVERRIDE" in status_text.upper()

        # Perbarui semua panel
        self.system_status_panel.update_data(data)
        self.map_view.update_data(data)
        self.log_panel.update_log(data)
        self.header.update_status(data)

        # Perbarui status tombol setiap kali data baru masuk
        self.update_button_states()

    @Slot()
    def on_add_current_pos(self):
        self.waypoints_panel.add_waypoint_from_pos(
            self.current_latitude, self.current_longitude
        )

    @Slot(bool, str)
    def on_connection_status_change(self, is_connected, message):
        self.status_bar.showMessage(message)
        status_text = "CONNECTED" if is_connected else "DISCONNECTED"
        status_prop = "connected" if is_connected else "disconnected"
        self.header.connection_status_label.setText(status_text)
        self.header.connection_status_label.setProperty("status", status_prop)
        self.style().polish(self.header.connection_status_label)

    # --- [MODIFIKASI BARU UNTUK MANUAL CAPTURE] ---
    @Slot()
    def on_request_manual_capture_surface(self):
        """
        Dipanggil ketika tombol 'Capture Surface' diklik.
        """
        print("[GUI] Meminta capture 'surface'...")
        self.api_client.send_command("MANUAL_CAPTURE", {"type": "surface"})

    @Slot()
    def on_request_manual_capture_underwater(self):
        """
        Dipanggil ketika tombol 'Capture Underwater' diklik.
        """
        print("[GUI] Meminta capture 'underwater'...")
        self.api_client.send_command("MANUAL_CAPTURE", {"type": "underwater"})

    # --- [AKHIR MODIFIKASI BARU] ---

    def closeEvent(self, event):
        print("Menutup aplikasi...")
        self.api_client.shutdown()
        event.accept()

    # --- [MODIFIKASI UTAMA DI SINI] ---
    @Slot(str)
    def load_predefined_mission(self, mission_id):
        """
        Memuat misi yang telah ditentukan (A atau B).
        Fungsi ini sekarang mem-parsing dictionary yang dikembalikan oleh missions.py.
        """
        mission_data = None
        if mission_id == "A":
            mission_data = (
                get_lintasan_a()
            )  # Mengembalikan {"arena": "A", "waypoints": []}
        elif mission_id == "B":
            mission_data = (
                get_lintasan_b()
            )  # Mengembalikan {"arena": "B", "waypoints": []}
        else:
            logging.warning(f"ID Arena tidak dikenal: {mission_id}")
            return

        if mission_data:
            # 1. Ekstrak HANYA list 'waypoints' (yang sekarang kosong)
            waypoints_list = mission_data.get("waypoints")

            # 2. Ekstrak arena (untuk logging, karena panel sudah tahu arenanya)
            arena = mission_data.get("arena")

            print(f"Memuat Lintasan {arena} (Peta)...")

            # 3. Kirim HANYA list waypoints (yang kosong) ke fungsi
            if waypoints_list is not None:
                self.waypoints_panel.load_waypoints_to_list(waypoints_list)
            else:
                logging.error(f"Gagal memuat misi {mission_id}: format data salah.")

    # --- [AKHIR MODIFIKASI UTAMA] ---

    def handle_manual_keys(self):
        self.api_client.send_command("MANUAL_CONTROL", list(self.active_manual_keys))

    def keyPressEvent(self, event):
        if self.current_control_mode != "MANUAL" or self.is_rc_override:
            return

        if event.isAutoRepeat():
            return
        key_map = {Qt.Key_W: "W", Qt.Key_A: "A", Qt.Key_S: "S", Qt.Key_D: "D"}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            if key_char not in self.active_manual_keys:
                self.active_manual_keys.add(key_char)
                self.control_panel.update_key_press_status(key_char, True)
                self.handle_manual_keys()

    def keyReleaseEvent(self, event):
        if self.current_control_mode != "MANUAL" or self.is_rc_override:
            return

        if event.isAutoRepeat():
            return
        key_map = {Qt.Key_W: "W", Qt.Key_A: "A", Qt.Key_S: "S", Qt.Key_D: "D"}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            self.active_manual_keys.discard(key_char)
            self.control_panel.update_key_press_status(key_char, False)
            self.handle_manual_keys()
