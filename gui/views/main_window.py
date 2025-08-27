# gui/views/main_window.py
# --- VERSI MODIFIKASI: Mengintegrasikan fitur Return to Home (RTH) ---
import sys
import os

# Tambahkan blok kode ini untuk memperbaiki path
try:
    # Cari path ke direktori root proyek (dua level di atas file ini)
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
)
from PySide6.QtCore import Slot, Qt, QThread

# Impor semua komponen kustom yang dibutuhkan
from gui.components.control_panel import ControlPanel
from gui.components.dashboard import Dashboard
from gui.components.settings_panel import SettingsPanel
from gui.components.video_view import VideoView
from gui.components.map_view import MapView
from gui.components.header import Header
from gui.components.waypoints_panel import WaypointsPanel
from gui.components.log_panel import LogPanel
from gui.missions import get_lintasan_a, get_lintasan_b

# Impor logika 'backend'
from backend.core.asv_handler import AsvHandler
from backend.services.vision_service import VisionService


class MainWindow(QMainWindow):
    """
    Jendela utama aplikasi yang menampung, menghubungkan, dan menjalankan
    semua komponen dan logika backend secara internal.
    """

    def __init__(self, config):
        super().__init__()
        self.setWindowTitle("ASV Control System - All-in-One")
        self.resize(1600, 900)

        self.config = config

        # --- 1. Inisialisasi Logika Backend sebagai Objek ---
        self.asv_handler = AsvHandler(config=self.config)
        self.vision_service = VisionService(
            config=self.config, asv_handler=self.asv_handler
        )

        # Inisialisasi thread, tapi jangan jalankan vision_thread dulu
        self.asv_handler_thread = QThread()
        self.vision_thread = QThread()

        # --- 2. Inisialisasi semua komponen UI ---
        self.header = Header(config=self.config)
        self.control_panel = ControlPanel(config=self.config)
        self.system_status_panel = Dashboard(config=self.config)
        self.settings_panel = SettingsPanel(config=self.config)
        self.video_view = VideoView(config=self.config)
        self.map_view = MapView(config=self.config)
        self.waypoints_panel = WaypointsPanel(config=self.config)
        self.log_panel = LogPanel(config=self.config)

        self.active_manual_keys = set()

        self.setup_ui()
        self.connect_signals()
        self.start_core_backend_thread()  # Ganti nama fungsi

    def setup_ui(self):
        """Mengatur tata letak semua komponen di dalam jendela utama."""
        layout_sidebar_kiri = QVBoxLayout()
        layout_sidebar_kiri.addWidget(self.control_panel)
        layout_sidebar_kiri.addWidget(self.settings_panel)
        layout_sidebar_kiri.addStretch()
        widget_sidebar_kiri = QWidget()
        widget_sidebar_kiri.setLayout(layout_sidebar_kiri)

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

        layout_kolom_utama = QHBoxLayout()
        layout_kolom_utama.addWidget(widget_sidebar_kiri, 2)
        layout_kolom_utama.addWidget(self.tab_tengah, 5)
        layout_kolom_utama.addWidget(widget_sidebar_kanan, 2)

        layout_keseluruhan = QVBoxLayout()
        layout_keseluruhan.addWidget(self.header)
        layout_keseluruhan.addLayout(layout_kolom_utama)

        widget_pusat = QWidget()
        widget_pusat.setLayout(layout_keseluruhan)
        self.setCentralWidget(widget_pusat)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Aplikasi Siap.")

    def connect_signals(self):
        """Menghubungkan sinyal dari GUI ke slot di logika backend dan sebaliknya."""
        # ### PERBAIKAN: Hubungkan sinyal start/stop dari VideoView ###
        self.video_view.toggle_camera_requested.connect(self.toggle_vision_service)

        # Alur: GUI -> Backend
        self.control_panel.mode_changed.connect(
            lambda mode: self.asv_handler.process_command("CHANGE_MODE", mode)
        )
        # --- MODIFIKASI RTH: Hubungkan sinyal navigasi ke handler baru ---
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

        # Sinyal untuk VisionService
        self.control_panel.mode_changed.connect(self.vision_service.set_mode)
        self.video_view.inversion_changed.connect(self.vision_service.set_inversion)

        # Alur: Backend -> GUI
        self.vision_service.frame_ready.connect(self.video_view.update_frame)
        self.asv_handler.telemetry_updated.connect(self.system_status_panel.update_data)
        self.asv_handler.telemetry_updated.connect(self.map_view.update_data)
        self.asv_handler.telemetry_updated.connect(self.log_panel.update_log)
        self.asv_handler.telemetry_updated.connect(self.header.update_status)

        # Alur antar komponen GUI
        self.waypoints_panel.waypoints_updated.connect(self.map_view.update_waypoints)
        self.waypoints_panel.add_current_pos_requested.connect(
            lambda: self.waypoints_panel.add_waypoint_from_pos(
                self.asv_handler.current_state.get("latitude"),
                self.asv_handler.current_state.get("longitude"),
            )
        )

    def start_core_backend_thread(self):
        """Hanya memulai thread AsvHandler yang harus berjalan otomatis."""
        self.asv_handler.moveToThread(self.asv_handler_thread)
        self.asv_handler_thread.started.connect(self.asv_handler.run)
        self.asv_handler_thread.start()
        print("Thread untuk AsvHandler dimulai.")

    @Slot(bool)
    def toggle_vision_service(self, start):
        """Memulai atau menghentikan thread VisionService berdasarkan permintaan GUI."""
        if start:
            if not self.vision_thread.isRunning():
                self.vision_service.moveToThread(self.vision_thread)
                self.vision_thread.started.connect(self.vision_service.run)
                # Set ulang flag 'running' di dalam service jika akan dimulai lagi
                self.vision_service.running = True
                self.vision_thread.start()
                print("Thread VisionService dimulai oleh pengguna.")
        else:
            if self.vision_thread.isRunning():
                self.vision_service.stop()  # Memberi sinyal agar loop di dalam 'run' berhenti
                self.vision_thread.quit()  # Menghentikan event loop thread
                self.vision_thread.wait(
                    3000
                )  # Tunggu hingga thread benar-benar berhenti
                print("Thread VisionService dihentikan oleh pengguna.")

    def closeEvent(self, event):
        """Memastikan semua thread ditutup dengan aman saat aplikasi ditutup."""
        print("Menutup aplikasi...")
        self.toggle_vision_service(
            False
        )  # Pastikan thread visi berhenti jika sedang berjalan
        self.asv_handler.stop()

        self.asv_handler_thread.quit()
        self.asv_handler_thread.wait(3000)

        print("Semua thread backend telah dihentikan.")
        super().closeEvent(event)

    @Slot(str)
    def load_predefined_mission(self, mission_id):
        """Memuat waypoint dari misi yang telah ditentukan."""
        if mission_id == "A":
            waypoints = get_lintasan_a()
            print("Memuat Lintasan A...")
        elif mission_id == "B":
            waypoints = get_lintasan_b()
            print("Memuat Lintasan B...")
        else:
            return

        self.waypoints_panel.load_waypoints_to_list(waypoints)

    # --- MODIFIKASI RTH: Tambahkan metode handler baru ---
    @Slot(str)
    def handle_navigation_command(self, command: str):
        """Menangani perintah navigasi non-spesifik seperti START, PAUSE, atau RETURN."""
        if command == "RETURN":
            print(
                "[MainWindow] Perintah Return to Home diterima, mengirim ke AsvHandler..."
            )
            # Kirim perintah baru yang spesifik untuk RTH
            self.asv_handler.process_command("INITIATE_RTH", {})
        else:
            # Di sini Anda bisa menangani perintah "START" atau "PAUSE" jika diperlukan
            print(
                f"[MainWindow] Perintah navigasi '{command}' belum diimplementasikan."
            )

    def handle_manual_keys(self):
        """Mengirim status tombol keyboard ke AsvHandler."""
        self.asv_handler.process_command(
            "MANUAL_CONTROL", list(self.active_manual_keys)
        )

    def keyPressEvent(self, event):
        """Menangani input keyboard untuk mode manual (WASD)."""
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
        """Menangani saat tombol keyboard dilepas."""
        if event.isAutoRepeat():
            return
        key_map = {Qt.Key_W: "W", Qt.Key_A: "A", Qt.Key_S: "S", Qt.Key_D: "D"}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            self.active_manual_keys.discard(key_char)
            self.control_panel.update_key_press_status(key_char, False)
            self.handle_manual_keys()
