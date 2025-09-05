# gui/views/main_window.py
# --- VERSI MODIFIKASI: Kontrol Tampilan, Bukan Logika ---

import sys
import os

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
    QHBoxLayout,
    QVBoxLayout,
    QTabWidget,
    QStatusBar,
    QScrollArea,
    QApplication,
    QSplitter,
)
from PySide6.QtCore import Slot, Qt, QThread, Signal  # <-- Impor Signal

from navantara_gui.components.control_panel import ControlPanel
from navantara_gui.components.dashboard import Dashboard
from navantara_gui.components.settings_panel import SettingsPanel
from navantara_gui.components.video_view import VideoView
from navantara_gui.components.map_view import MapView
from navantara_gui.components.header import Header
from navantara_gui.components.waypoints_panel import WaypointsPanel
from navantara_gui.components.log_panel import LogPanel
from navantara_gui.missions import get_lintasan_a, get_lintasan_b

from navantara_backend.core.asv_handler import AsvHandler
from navantara_backend.services.vision_service import VisionService


class MainWindow(QMainWindow):
    # --- PERUBAHAN 1: Tambahkan sinyal baru untuk komunikasi antar thread ---
    set_stream_listening_requested = Signal(bool)
    # ----------------------------------------------------------------------

    def __init__(self, config):
        super().__init__()
        self.setWindowTitle("ASV Control System - All-in-One")

        self.config = config

        self.asv_handler = AsvHandler(config=self.config)
        self.vision_service = VisionService(
            config=self.config, asv_handler=self.asv_handler
        )
        self.asv_handler_thread = QThread()
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

        # --- PERUBAHAN 2: Pindahkan proses start backend ke sini ---
        self.start_backend_services()
        # ----------------------------------------------------------

        self.showMaximized()

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
            if theme_name == "dark":
                self.header.theme_button.setText("Switch to Light Mode")
            else:
                self.header.theme_button.setText("Switch to Dark Mode")
            self.current_theme = theme_name

    @Slot()
    def toggle_theme(self):
        if self.current_theme == "dark":
            self._apply_theme("light")
        else:
            self._apply_theme("dark")

    def setup_ui(self):
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
        self.status_bar.showMessage("Aplikasi Siap.")

    def connect_signals(self):
        # --- PERUBAHAN 3: Hubungkan sinyal baru ke slot di VisionService ---
        self.set_stream_listening_requested.connect(
            self.vision_service.set_gui_listening
        )
        # --------------------------------------------------------------------

        self.header.theme_changed_requested.connect(self.toggle_theme)

        # Tombol di VideoView sekarang akan memanggil on_toggle_gui_stream
        self.video_view.toggle_camera_requested.connect(self.on_toggle_gui_stream)

        self.control_panel.mode_changed.connect(
            lambda mode: self.asv_handler.process_command("CHANGE_MODE", mode)
        )
        self.control_panel.navigation_command.connect(self.handle_navigation_command)
        self.waypoints_panel.send_waypoints.connect(
            lambda wps: self.asv_handler.process_command("SET_WAYPOINTS", wps)
        )
        self.settings_panel.connect_requested.connect(
            lambda details: self.asv_handler.process_command(
                "CONFIGURE_SERIAL", details
            )
        )
        self.waypoints_panel.load_mission_requested.connect(
            self.load_predefined_mission
        )
        self.control_panel.mode_changed.connect(self.vision_service.set_mode)
        self.video_view.inversion_changed.connect(self.vision_service.set_inversion)

        self.vision_service.frame_ready_cam1.connect(self.video_view.update_frame_1)
        self.vision_service.frame_ready_cam2.connect(self.video_view.update_frame_2)

        self.asv_handler.telemetry_updated.connect(self.system_status_panel.update_data)
        self.asv_handler.telemetry_updated.connect(self.map_view.update_data)
        self.asv_handler.telemetry_updated.connect(self.log_panel.update_log)
        self.asv_handler.telemetry_updated.connect(self.header.update_status)
        self.waypoints_panel.waypoints_updated.connect(self.map_view.update_waypoints)
        self.waypoints_panel.add_current_pos_requested.connect(
            lambda: self.waypoints_panel.add_waypoint_from_pos(
                self.asv_handler.current_state.get("latitude"),
                self.asv_handler.current_state.get("longitude"),
            )
        )

    def start_backend_services(self):
        """Memulai semua layanan backend (ASV & Visi) di thread terpisah."""
        # Start AsvHandler
        self.asv_handler.moveToThread(self.asv_handler_thread)
        self.asv_handler_thread.started.connect(self.asv_handler.run)
        self.asv_handler_thread.start()
        print("Thread untuk AsvHandler dimulai.")

        # --- PERUBAHAN 4: VisionService juga dimulai otomatis ---
        print("Memulai layanan Visi di latar belakang...")
        self.vision_service.start()
        # --------------------------------------------------------

    # --- PERUBAHAN 5: Logika toggle_vision_service diubah total ---
    @Slot(bool)
    def on_toggle_gui_stream(self, show_video: bool):
        """
        Slot ini dipanggil oleh tombol di VideoView.
        Fungsinya hanya untuk meminta backend mengirim atau berhenti mengirim frame.
        """
        print(
            f"GUI meminta untuk {'menampilkan' if show_video else 'menyembunyikan'} stream video."
        )
        # Menggunakan sinyal untuk komunikasi thread-safe ke VisionService
        self.set_stream_listening_requested.emit(show_video)

    # -----------------------------------------------------------------

    def closeEvent(self, event):
        print("Menutup aplikasi...")
        # Hentikan semua service di backend
        self.vision_service.stop()
        self.asv_handler.stop()

        # Hentikan thread utama AsvHandler
        self.asv_handler_thread.quit()
        self.asv_handler_thread.wait(3000)

        print("Semua thread backend telah dihentikan.")
        super().closeEvent(event)

    @Slot(str)
    def load_predefined_mission(self, mission_id):
        if mission_id == "A":
            waypoints = get_lintasan_a()
            print("Memuat Lintasan A...")
        elif mission_id == "B":
            waypoints = get_lintasan_b()
            print("Memuat Lintasan B...")
        else:
            return
        self.waypoints_panel.load_waypoints_to_list(waypoints)

    @Slot(str)
    def handle_navigation_command(self, command: str):
        if command == "RETURN":
            print(
                "[MainWindow] Perintah Return to Home diterima, mengirim ke AsvHandler..."
            )
            self.asv_handler.process_command("INITIATE_RTH", {})
        elif command == "START":
            self.asv_handler.process_command("START_MISSION", {})
        else:
            print(
                f"[MainWindow] Perintah navigasi '{command}' belum diimplementasikan."
            )

    def handle_manual_keys(self):
        self.asv_handler.process_command(
            "MANUAL_CONTROL", list(self.active_manual_keys)
        )

    def keyPressEvent(self, event):
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
        if event.isAutoRepeat():
            return
        key_map = {Qt.Key_W: "W", Qt.Key_A: "A", Qt.Key_S: "S", Qt.Key_D: "D"}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            self.active_manual_keys.discard(key_char)
            self.control_panel.update_key_press_status(key_char, False)
            self.handle_manual_keys()
