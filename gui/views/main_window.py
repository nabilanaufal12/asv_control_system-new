# gui/views/main_window.py
# --- FINAL: Mengintegrasikan semua komponen UI dan klien API ---

import sys
from PySide6.QtWidgets import (QMainWindow, QWidget, QApplication,
                               QHBoxLayout, QVBoxLayout, QTabWidget,
                               QMessageBox, QStatusBar, QLabel)
from PySide6.QtCore import Slot, Qt

# Impor semua komponen kustom yang dibutuhkan
from gui.components.control_panel import ControlPanel
from gui.components.dashboard import Dashboard
from gui.components.settings_panel import SettingsPanel
from gui.components.video_view import VideoView
from gui.components.map_view import MapView
from gui.components.header import Header
from gui.components.waypoints_panel import WaypointsPanel
from gui.components.log_panel import LogPanel
from gui.api_client import ApiClient

class MainWindow(QMainWindow):
    """
    Jendela utama aplikasi yang menampung dan menghubungkan semua widget.
    """
    def __init__(self, config, api_client):
        super().__init__()
        self.setWindowTitle("ASV Control System - GUI")
        self.resize(1600, 900)
        
        self.config = config
        # Menggunakan instance ApiClient yang sudah ada, yang dibuat di main.py
        self.api_client = api_client
        
        # Inisialisasi semua komponen UI dengan meneruskan config
        self.header = Header(config=self.config)
        self.control_panel = ControlPanel(config=self.config)
        self.system_status_panel = Dashboard(config=self.config)
        self.settings_panel = SettingsPanel(config=self.config)
        self.video_view = VideoView(config=self.config) # VideoView sekarang menangani koneksi videonya sendiri
        self.map_view = MapView(config=self.config)
        self.waypoints_panel = WaypointsPanel(config=self.config)
        self.log_panel = LogPanel(config=self.config)
        
        # Set untuk melacak tombol keyboard yang sedang ditekan
        self.active_manual_keys = set()
        
        self.setup_ui()
        self.connect_signals()

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
        layout_kolom_utama.addWidget(widget_sidebar_kiri, 2)  # Proporsi 2
        layout_kolom_utama.addWidget(self.tab_tengah, 5)     # Proporsi 5 (lebih besar)
        layout_kolom_utama.addWidget(widget_sidebar_kanan, 2) # Proporsi 2
        
        # --- Layout Keseluruhan ---
        layout_keseluruhan = QVBoxLayout()
        layout_keseluruhan.addWidget(self.header)
        layout_keseluruhan.addLayout(layout_kolom_utama)
        
        widget_pusat = QWidget()
        widget_pusat.setLayout(layout_keseluruhan)
        self.setCentralWidget(widget_pusat)
        
        # --- Status Bar ---
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.label_status_koneksi = QLabel("Menunggu koneksi...")
        self.status_bar.addWidget(self.label_status_koneksi)

    def connect_signals(self):
        """Menghubungkan sinyal dari satu komponen ke slot di komponen lain."""
        # Alur Sinyal: Kontrol Pengguna -> ApiClient (untuk dikirim ke backend)
        self.control_panel.mode_changed.connect(self.api_client.handle_mode_change)
        self.waypoints_panel.send_waypoints.connect(self.api_client.set_waypoints)
        self.settings_panel.connect_requested.connect(self.api_client.connect_to_port)
        
        # Alur Sinyal: Kontrol Pengguna -> VideoView (untuk memberitahu backend via koneksi videonya)
        # Ini penting agar backend tahu mode apa yang sedang aktif untuk logika visi
        self.control_panel.mode_changed.connect(self.video_view.set_mode)
        
        # Alur Sinyal: ApiClient (menerima data dari backend) -> UI (untuk update tampilan)
        self.api_client.connection_status_changed.connect(self.update_status_koneksi)
        self.api_client.data_updated.connect(self.map_view.update_data)
        self.api_client.data_updated.connect(self.system_status_panel.update_data)
        self.api_client.data_updated.connect(self.log_panel.update_log)
        self.api_client.data_updated.connect(self.header.update_status)

    def keyPressEvent(self, event):
        """Menangani input keyboard untuk mode manual (WASD)."""
        if event.isAutoRepeat(): return
        key_map = {Qt.Key_W: 'W', Qt.Key_A: 'A', Qt.Key_S: 'S', Qt.Key_D: 'D'}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            if key_char not in self.active_manual_keys:
                self.active_manual_keys.add(key_char)
                self.control_panel.update_key_press_status(key_char, True)
                self.api_client.handle_manual_keys(list(self.active_manual_keys))

    def keyReleaseEvent(self, event):
        """Menangani saat tombol keyboard dilepas."""
        if event.isAutoRepeat(): return
        key_map = {Qt.Key_W: 'W', Qt.Key_A: 'A', Qt.Key_S: 'S', Qt.Key_D: 'D'}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            self.active_manual_keys.discard(key_char)
            self.control_panel.update_key_press_status(key_char, False)
            self.api_client.handle_manual_keys(list(self.active_manual_keys))
            
    @Slot(bool, str)
    def update_status_koneksi(self, terhubung, pesan):
        """Mengupdate teks dan warna di status bar berdasarkan status koneksi."""
        self.label_status_koneksi.setText(pesan)
        if terhubung:
            self.label_status_koneksi.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.label_status_koneksi.setStyleSheet("color: #F44336; font-weight: bold;")

    def closeEvent(self, event):
        """Memastikan semua thread dan koneksi ditutup dengan aman saat aplikasi ditutup."""
        print("Menutup aplikasi...")
        # Hentikan klien API utama (untuk perintah dan telemetri)
        self.api_client.shutdown()
        
        # VideoView akan ditutup secara otomatis sebagai child widget,
        # yang akan memicu closeEvent-nya sendiri untuk menghentikan thread videonya.
        # Tidak perlu memanggil self.video_view.stop_camera() lagi.
        
        super().closeEvent(event)