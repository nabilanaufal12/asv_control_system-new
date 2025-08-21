# gui/views/main_window.py
# --- VERSI FINAL: Mengintegrasikan semua komponen secara internal ---

import sys
from PySide6.QtWidgets import (QMainWindow, QWidget, QApplication,
                               QHBoxLayout, QVBoxLayout, QTabWidget,
                               QMessageBox, QStatusBar, QLabel)
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

# --- PERUBAHAN: Impor logika 'backend' ---
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
        self.vision_service = VisionService(config=self.config, asv_handler=self.asv_handler)

        # --- 2. Inisialisasi semua komponen UI ---
        self.header = Header(config=self.config)
        self.control_panel = ControlPanel(config=self.config)
        self.system_status_panel = Dashboard(config=self.config)
        self.settings_panel = SettingsPanel(config=self.config)
        self.video_view = VideoView(config=self.config) # VideoView akan disederhanakan
        self.map_view = MapView(config=self.config)
        self.waypoints_panel = WaypointsPanel(config=self.config)
        self.log_panel = LogPanel(config=self.config)
        
        # Set untuk melacak tombol keyboard yang sedang ditekan
        self.active_manual_keys = set()
        
        self.setup_ui()
        self.connect_signals()

        # --- 3. Siapkan dan Jalankan Thread untuk Logika Backend ---
        self.setup_and_start_backend_threads()

    def setup_ui(self):
        """Mengatur tata letak semua komponen di dalam jendela utama."""
        # --- Sidebar Kiri ---
        layout_sidebar_kiri = QVBoxLayout()
        layout_sidebar_kiri.addWidget(self.control_panel)
        layout_sidebar_kiri.addWidget(self.settings_panel)
        layout_sidebar_kiri.addStretch()
        widget_sidebar_kiri = QWidget()
        widget_sidebar_kiri.setLayout(layout_sidebar_kiri)
        
        # --- Konten Tengah (dengan Tab) ---
        self.tab_tengah = QTabWidget()
        self.tab_tengah.addTab(self.video_view, "Video Stream")
        self.tab_tengah.addTab(self.map_view, "Map View")
        
        # --- Sidebar Kanan ---
        layout_sidebar_kanan = QVBoxLayout()
        layout_sidebar_kanan.addWidget(self.waypoints_panel)
        layout_sidebar_kanan.addWidget(self.system_status_panel)
        layout_sidebar_kanan.addWidget(self.log_panel)
        layout_sidebar_kanan.addStretch()
        widget_sidebar_kanan = QWidget()
        widget_sidebar_kanan.setLayout(layout_sidebar_kanan)
        
        # --- Layout Kolom Utama ---
        layout_kolom_utama = QHBoxLayout()
        layout_kolom_utama.addWidget(widget_sidebar_kiri, 2)
        layout_kolom_utama.addWidget(self.tab_tengah, 5)
        layout_kolom_utama.addWidget(widget_sidebar_kanan, 2)
        
        # --- Layout Keseluruhan ---
        layout_keseluruhan = QVBoxLayout()
        layout_keseluruhan.addWidget(self.header)
        layout_keseluruhan.addLayout(layout_kolom_utama)
        
        widget_pusat = QWidget()
        widget_pusat.setLayout(layout_keseluruhan)
        self.setCentralWidget(widget_pusat)
        
        # --- Status Bar (disederhanakan) ---
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Aplikasi Siap.")

    def connect_signals(self):
        """Menghubungkan sinyal dari GUI ke slot di logika backend dan sebaliknya."""
        # Alur: Kontrol Pengguna (GUI) -> Logika Backend (via pemanggilan metode langsung)
        self.control_panel.mode_changed.connect(
            lambda mode: self.asv_handler.process_command("CHANGE_MODE", mode)
        )
        self.waypoints_panel.send_waypoints.connect(
            lambda wps: self.asv_handler.process_command("SET_WAYPOINTS", wps)
        )
        self.settings_panel.connect_requested.connect(
            lambda details: self.asv_handler.process_command("CONFIGURE_SERIAL", details)
        )
        # Sinyal untuk memberitahu VisionService tentang perubahan mode dan inversi
        self.control_panel.mode_changed.connect(self.vision_service.set_mode)
        # Anda mungkin perlu menambahkan sinyal dari tombol 'Invert' di VideoView ke VisionService
        # (Akan ditambahkan di modifikasi video_view.py)

        # Alur: Logika Backend (Sinyal) -> Tampilan (GUI Slot)
        self.vision_service.frame_ready.connect(self.video_view.update_frame)
        self.asv_handler.telemetry_updated.connect(self.system_status_panel.update_data)
        self.asv_handler.telemetry_updated.connect(self.map_view.update_data)
        self.asv_handler.telemetry_updated.connect(self.log_panel.update_log)
        self.asv_handler.telemetry_updated.connect(self.header.update_status)

    def setup_and_start_backend_threads(self):
        """Membuat, memindahkan, dan memulai QThread untuk layanan backend."""
        # VisionService perlu dijalankan di QThread agar tidak memblokir GUI
        self.vision_thread = QThread()
        self.vision_service.moveToThread(self.vision_thread)
        self.vision_thread.started.connect(self.vision_service.run)
        self.vision_thread.start()
        print("Thread untuk VisionService dimulai.")

        # AsvHandler menggunakan threading internal Python, jadi cukup panggil metode start-nya
        self.asv_handler.start_threads()

    def closeEvent(self, event):
        """Memastikan semua thread ditutup dengan aman saat aplikasi ditutup."""
        print("Menutup aplikasi...")
        self.vision_service.stop()
        self.asv_handler.stop()
        
        self.vision_thread.quit()
        self.vision_thread.wait(5000) # Tunggu hingga 5 detik agar thread berhenti dengan bersih
        
        print("Semua thread backend telah dihentikan.")
        super().closeEvent(event)

    def handle_manual_keys(self):
        """Mengirim status tombol keyboard ke AsvHandler."""
        self.asv_handler.process_command("MANUAL_CONTROL", list(self.active_manual_keys))

    def keyPressEvent(self, event):
        """Menangani input keyboard untuk mode manual (WASD)."""
        if event.isAutoRepeat(): return
        key_map = {Qt.Key_W: 'W', Qt.Key_A: 'A', Qt.Key_S: 'S', Qt.Key_D: 'D'}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            if key_char not in self.active_manual_keys:
                self.active_manual_keys.add(key_char)
                self.control_panel.update_key_press_status(key_char, True)
                self.handle_manual_keys()

    def keyReleaseEvent(self, event):
        """Menangani saat tombol keyboard dilepas."""
        if event.isAutoRepeat(): return
        key_map = {Qt.Key_W: 'W', Qt.Key_A: 'A', Qt.Key_S: 'S', Qt.Key_D: 'D'}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            self.active_manual_keys.discard(key_char)
            self.control_panel.update_key_press_status(key_char, False)
            self.handle_manual_keys()