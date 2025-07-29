# gui/views/main_window.py
# --- MODIFIKASI: Memperbaiki dan menyederhanakan alur sinyal ---

import sys
from PySide6.QtWidgets import (QMainWindow, QWidget, QApplication, 
                               QHBoxLayout, QVBoxLayout, QTabWidget, 
                               QMessageBox, QStatusBar, QLabel)
from PySide6.QtCore import Slot, Qt

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
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ASV Control System - GUI")
        self.resize(1600, 900)
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
        self.setup_ui()
        self.connect_signals()

    def setup_ui(self):
        left_sidebar_layout = QVBoxLayout()
        left_sidebar_layout.addWidget(self.control_panel)
        left_sidebar_layout.addWidget(self.settings_panel)
        left_sidebar_layout.addStretch()
        left_sidebar_widget = QWidget()
        left_sidebar_widget.setLayout(left_sidebar_layout)
        self.center_tabs = QTabWidget()
        self.center_tabs.addTab(self.video_view, "Video Stream")
        self.center_tabs.addTab(self.map_view, "Map View")
        right_sidebar_layout = QVBoxLayout()
        right_sidebar_layout.addWidget(self.waypoints_panel)
        right_sidebar_layout.addWidget(self.system_status_panel)
        right_sidebar_layout.addWidget(self.log_panel)
        right_sidebar_layout.addStretch()
        right_sidebar_widget = QWidget()
        right_sidebar_widget.setLayout(right_sidebar_layout)
        main_columns_layout = QHBoxLayout()
        main_columns_layout.addWidget(left_sidebar_widget, 2)
        main_columns_layout.addWidget(self.center_tabs, 5)
        main_columns_layout.addWidget(right_sidebar_widget, 2)
        overall_layout = QVBoxLayout()
        overall_layout.addWidget(self.header)
        overall_layout.addLayout(main_columns_layout)
        central_widget = QWidget()
        central_widget.setLayout(overall_layout)
        self.setCentralWidget(central_widget)
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.connection_status_label = QLabel("Menunggu koneksi manual...")
        self.status_bar.addWidget(self.connection_status_label)

    def connect_signals(self):
        # --- ALUR SINYAL BARU YANG LEBIH BERSIH ---
        # Semua sinyal dari panel kontrol sekarang langsung menuju ApiClient
        self.control_panel.mode_changed.connect(self.api_client.handle_mode_change)
        
        # Sinyal dari komponen lain
        self.video_view.vision_status_updated.connect(self.api_client.handle_vision_status)
        self.waypoints_panel.send_waypoints.connect(self.api_client.set_waypoints)
        self.settings_panel.connect_requested.connect(self.api_client.connect_manual)
        
        # Sinyal dari ApiClient ke UI
        self.api_client.connection_status_changed.connect(self.update_connection_status)
        
        # Sinyal dari ApiClient untuk meneruskan mode ke VideoView
        self.api_client.mode_changed_for_video.connect(self.video_view.set_mode)

    def keyPressEvent(self, event):
        if event.isAutoRepeat(): return
        key_map = {Qt.Key_W: 'W', Qt.Key_A: 'A', Qt.Key_S: 'S', Qt.Key_D: 'D'}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            if key_char not in self.active_manual_keys:
                self.active_manual_keys.add(key_char)
                self.control_panel.update_key_press_status(key_char, True)
                # Langsung kirim set key yang aktif ke ApiClient
                self.api_client.handle_manual_keys(list(self.active_manual_keys))

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat(): return
        key_map = {Qt.Key_W: 'W', Qt.Key_A: 'A', Qt.Key_S: 'S', Qt.Key_D: 'D'}
        if event.key() in key_map:
            key_char = key_map[event.key()]
            self.active_manual_keys.discard(key_char)
            self.control_panel.update_key_press_status(key_char, False)
            # Langsung kirim set key yang aktif ke ApiClient
            self.api_client.handle_manual_keys(list(self.active_manual_keys))
            
    @Slot(bool, str)
    def update_connection_status(self, is_connected, message):
        self.connection_status_label.setText(message)
        if is_connected: self.connection_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else: self.connection_status_label.setStyleSheet("color: #F44336; font-weight: bold;")

    def closeEvent(self, event):
        self.api_client.shutdown()
        self.video_view.stop_camera()
        super().closeEvent(event)