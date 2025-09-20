# src/navantara_gui/views/main_window.py
import sys
import os

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
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QTabWidget, QStatusBar,
    QScrollArea, QApplication, QSplitter
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
    """
    Jendela utama aplikasi GUI. Bertindak sebagai perekat untuk semua komponen,
    menghubungkan sinyal dan slot, serta mendelegasikan komunikasi ke ApiClient.
    """

    def __init__(self, config):
        super().__init__()
        self.setWindowTitle("ASV Control System - Navantara Client")
        self.config = config

        self.api_client = ApiClient(config=self.config)

        # --- VARIABEL BARU: Untuk menyimpan posisi terakhir dari ASV ---
        self.current_latitude = 0.0
        self.current_longitude = 0.0

        # Inisialisasi semua komponen UI
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

        print("Memulai koneksi klien API ke server...")
        self.api_client.connect()

        self.showMaximized()

    def _load_themes(self):
        # (Fungsi ini tidak berubah)
        try:
            gui_dir = os.path.dirname(os.path.abspath(__file__))
            dark_theme_path = os.path.join(gui_dir, "..", "assets", "resources", "dark_theme.qss")
            with open(dark_theme_path, "r") as f: self.themes["dark"] = f.read()
            light_theme_path = os.path.join(gui_dir, "..", "assets", "resources", "light_theme.qss")
            with open(light_theme_path, "r") as f: self.themes["light"] = f.read()
        except Exception as e:
            print(f"Peringatan: Gagal memuat file tema. Error: {e}")

    def _apply_theme(self, theme_name):
        # (Fungsi ini tidak berubah)
        if theme_name in self.themes:
            QApplication.instance().setStyleSheet(self.themes[theme_name])
            button_text = "Switch to Light Mode" if theme_name == "dark" else "Switch to Dark Mode"
            self.header.theme_button.setText(button_text)
            self.current_theme = theme_name

    @Slot()
    def toggle_theme(self):
        # (Fungsi ini tidak berubah)
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
        # Koneksi dari UI ke ApiClient
        self.header.theme_changed_requested.connect(self.toggle_theme)
        self.settings_panel.connect_requested.connect(
            lambda details: self.api_client.send_command("CONFIGURE_SERIAL", details)
        )
        self.control_panel.mode_changed.connect(
            lambda mode: self.api_client.send_command("CHANGE_MODE", {"mode": mode})
        )
        self.waypoints_panel.send_waypoints.connect(
            lambda wps: self.api_client.send_command("SET_WAYPOINTS", wps)
        )

        # --- PERUBAHAN 1: Hubungkan sinyal `add_current_pos_requested` ke slot baru ---
        self.waypoints_panel.add_current_pos_requested.connect(self.on_add_current_pos)

        # Koneksi dari ApiClient ke UI
        self.api_client.connection_status_changed.connect(self.on_connection_status_change)
        
        # --- PERUBAHAN 2: Ganti koneksi data_updated ke satu slot pusat ---
        self.api_client.data_updated.connect(self.on_data_updated)
        
        # Koneksi lain yang tidak berubah
        self.waypoints_panel.load_mission_requested.connect(self.load_predefined_mission)
        self.api_client.frame_cam1_updated.connect(self.video_view.update_frame_1)
        self.api_client.frame_cam2_updated.connect(self.video_view.update_frame_2)
        self.waypoints_panel.waypoints_updated.connect(self.map_view.update_waypoints)

    # --- PERUBAHAN 3: SLOT BARU untuk menangani semua pembaruan data telemetri ---
    @Slot(dict)
    def on_data_updated(self, data):
        """
        Satu slot pusat untuk menerima data dari backend dan mendistribusikannya
        ke semua komponen UI yang relevan.
        """
        # 1. Simpan posisi saat ini untuk digunakan oleh WaypointsPanel
        self.current_latitude = data.get("latitude", self.current_latitude)
        self.current_longitude = data.get("longitude", self.current_longitude)

        # 2. Perbarui semua panel lain seperti biasa
        self.system_status_panel.update_data(data)
        self.map_view.update_data(data)
        self.log_panel.update_log(data)
        self.header.update_status(data)
        
        # 3. Cek status untuk mengunci/membuka ControlPanel berdasarkan override RC
        status_text = data.get("status", "")
        self.control_panel.update_control_locks(status_text)
        
    # --- PERUBAHAN 4: SLOT BARU yang dipanggil saat tombol "Add Current Pos" ditekan ---
    @Slot()
    def on_add_current_pos(self):
        """
        Menanggapi permintaan dari WaypointsPanel dengan mengirimkan
        koordinat yang saat ini disimpan di MainWindow.
        """
        self.waypoints_panel.add_waypoint_from_pos(self.current_latitude, self.current_longitude)

    @Slot(bool, str)
    def on_connection_status_change(self, is_connected, message):
        # (Fungsi ini tidak berubah)
        self.status_bar.showMessage(message)
        status_text = "CONNECTED" if is_connected else "DISCONNECTED"
        status_prop = "connected" if is_connected else "disconnected"
        self.header.connection_status_label.setText(status_text)
        self.header.connection_status_label.setProperty("status", status_prop)
        self.style().polish(self.header.connection_status_label)

    def closeEvent(self, event):
        # (Fungsi ini tidak berubah)
        print("Menutup aplikasi...")
        self.api_client.shutdown()
        event.accept()

    @Slot(str)
    def load_predefined_mission(self, mission_id):
        # (Fungsi ini tidak berubah)
        waypoints = get_lintasan_a() if mission_id == "A" else get_lintasan_b()
        print(f"Memuat Lintasan {mission_id}...")
        self.waypoints_panel.load_waypoints_to_list(waypoints)

    def handle_manual_keys(self):
        # (Fungsi ini tidak berubah)
        self.api_client.send_command("MANUAL_CONTROL", list(self.active_manual_keys))

    def keyPressEvent(self, event):
        # (Fungsi ini tidak berubah)
        if event.isAutoRepeat(): return
        key_map = {Qt.Key_W: "W", Qt.Key_A: "A", Qt.Key_S: "S", Qt.Key_D: "D"}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            if key_char not in self.active_manual_keys:
                self.active_manual_keys.add(key_char)
                self.control_panel.update_key_press_status(key_char, True)
                self.handle_manual_keys()

    def keyReleaseEvent(self, event):
        # (Fungsi ini tidak berubah)
        if event.isAutoRepeat(): return
        key_map = {Qt.Key_W: "W", Qt.Key_A: "A", Qt.Key_S: "S", Qt.Key_D: "D"}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            self.active_manual_keys.discard(key_char)
            self.control_panel.update_key_press_status(key_char, False)
            self.handle_manual_keys()