# gui/views/main_window.py
# --- PERBAIKAN: Memperbarui path impor untuk VideoView ---

import sys
from PySide6.QtWidgets import (QMainWindow, QWidget, QApplication, 
                               QHBoxLayout, QVBoxLayout, QTabWidget, 
                               QMessageBox, QStatusBar, QLabel)
from PySide6.QtCore import Slot, Qt

# Impor semua komponen kustom yang dibutuhkan
from gui.components.control_panel import ControlPanel 
from gui.components.dashboard import Dashboard
from gui.components.settings_panel import SettingsPanel
# === PERUBAHAN DI SINI ===
# Path impor diubah untuk menunjuk ke folder 'camera' yang baru
from gui.components.camera.video_view import VideoView 
from gui.components.map_view import MapView
from gui.components.header import Header
from gui.components.waypoints_panel import WaypointsPanel
from gui.components.log_panel import LogPanel
from gui.api_client import ApiClient

class MainWindow(QMainWindow):
    """
    Jendela utama aplikasi yang menampung dan menghubungkan semua widget.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ASV Control System - GUI")
        self.resize(1600, 900)
        
        # Inisialisasi komponen utama
        self.api_client = ApiClient() 
        self.header = Header()
        self.control_panel = ControlPanel("Vehicle Control")
        self.system_status_panel = Dashboard("System Status")
        self.settings_panel = SettingsPanel()
        self.video_view = VideoView()
        self.map_view = MapView()
        self.waypoints_panel = WaypointsPanel()
        self.log_panel = LogPanel()
        
        self.active_manual_keys = set()
        
        # Atur UI dan hubungkan sinyal
        self.setup_ui()
        self.connect_signals()

    def setup_ui(self):
        """Mengatur tata letak semua komponen di dalam jendela utama."""
        # Sidebar Kiri (Kontrol & Pengaturan)
        left_sidebar_layout = QVBoxLayout()
        left_sidebar_layout.addWidget(self.control_panel)
        left_sidebar_layout.addWidget(self.settings_panel)
        left_sidebar_layout.addStretch()
        left_sidebar_widget = QWidget()
        left_sidebar_widget.setLayout(left_sidebar_layout)
        
        # Area Tengah dengan Tab (Video & Peta)
        self.center_tabs = QTabWidget()
        self.center_tabs.addTab(self.video_view, "Video Stream")
        self.center_tabs.addTab(self.map_view, "Map View")
        
        # Sidebar Kanan (Waypoint, Status, Log)
        right_sidebar_layout = QVBoxLayout()
        right_sidebar_layout.addWidget(self.waypoints_panel)
        right_sidebar_layout.addWidget(self.system_status_panel)
        right_sidebar_layout.addWidget(self.log_panel)
        right_sidebar_layout.addStretch()
        right_sidebar_widget = QWidget()
        right_sidebar_widget.setLayout(right_sidebar_layout)
        
        # Layout Kolom Utama
        main_columns_layout = QHBoxLayout()
        main_columns_layout.addWidget(left_sidebar_widget, 2)
        main_columns_layout.addWidget(self.center_tabs, 5)
        main_columns_layout.addWidget(right_sidebar_widget, 2)
        
        # Layout Keseluruhan
        overall_layout = QVBoxLayout()
        overall_layout.addWidget(self.header)
        overall_layout.addLayout(main_columns_layout)
        
        central_widget = QWidget()
        central_widget.setLayout(overall_layout)
        self.setCentralWidget(central_widget)
        
        # Status Bar di bagian bawah
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.connection_status_label = QLabel("Menunggu koneksi manual...")
        self.status_bar.addWidget(self.connection_status_label)

    def connect_signals(self):
        """Menghubungkan sinyal dari satu komponen ke slot di komponen lain."""
        # Alur Sinyal: Kontrol Pengguna -> ApiClient
        self.control_panel.mode_changed.connect(self.api_client.handle_mode_change)
        self.waypoints_panel.send_waypoints.connect(self.api_client.set_waypoints)
        self.settings_panel.connect_requested.connect(self.api_client.connect_manual)
        
        # Alur Sinyal: Logika Visi -> ApiClient
        self.video_view.vision_status_updated.connect(self.api_client.handle_vision_status)
        
        # Alur Sinyal: ApiClient -> UI (untuk update tampilan)
        self.api_client.connection_status_changed.connect(self.update_connection_status)
        self.api_client.data_updated.connect(self.map_view.update_data)
        self.api_client.data_updated.connect(self.system_status_panel.update_data)
        
        # Mengirim data sensor ke thread video untuk Geo-tagging
        self.api_client.data_updated.connect(self.video_view.update_telemetry_in_thread)
        
        # Alur Sinyal: ApiClient -> VideoView (untuk sinkronisasi mode)
        self.api_client.mode_changed_for_video.connect(self.video_view.set_mode)

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
    def update_connection_status(self, is_connected, message):
        """Mengupdate teks dan warna di status bar berdasarkan status koneksi."""
        self.connection_status_label.setText(message)
        if is_connected:
            self.connection_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.connection_status_label.setStyleSheet("color: #F44336; font-weight: bold;")

    def closeEvent(self, event):
        """Memastikan semua thread dan koneksi ditutup dengan aman saat aplikasi ditutup."""
        print("Menutup aplikasi...")
        self.api_client.shutdown()
        self.video_view.stop_camera()
        super().closeEvent(event)