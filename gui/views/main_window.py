# gui/views/main_window.py
# File ini merakit semua komponen UI menjadi satu jendela utama.

import sys
from PySide6.QtWidgets import (QMainWindow, QWidget, QApplication, 
                               QHBoxLayout, QVBoxLayout, QTabWidget, QMessageBox)
from PySide6.QtCore import Slot

from components.dashboard import Dashboard
from components.control_panel import ControlPanel
from components.settings_panel import SettingsPanel
from components.video_view import VideoView
from components.map_view import MapView
from components.header import Header
from components.waypoints_panel import WaypointsPanel
from components.log_panel import LogPanel
from api_client import ApiClient

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
        
        # Atur layout
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

        self.connect_signals()

    def connect_signals(self):
        """Metode terpusat untuk menghubungkan semua sinyal dan slot."""
        # Sinyal dari ApiClient
        self.api_client.data_updated.connect(self.header.update_status)
        self.api_client.data_updated.connect(self.system_status_panel.update_data)
        self.api_client.data_updated.connect(self.map_view.update_data)
        self.api_client.data_updated.connect(self.log_panel.update_log)
        self.api_client.data_updated.connect(self.update_ui_from_state)
        
        # Sinyal dari ControlPanel
        self.control_panel.mode_changed.connect(self.handle_mode_change)
        self.control_panel.emergency_stop_clicked.connect(self.emergency_stop)
        self.control_panel.manual_control_updated.connect(self.handle_manual_command)
        self.control_panel.navigation_command.connect(self.handle_navigation_command) # <-- Hubungkan sinyal baru

        # Sinyal dari SettingsPanel
        self.settings_panel.pid_updated.connect(self.update_pid_constants)
        self.settings_panel.servo_settings_updated.connect(self.update_servo_settings)
        self.settings_panel.connect_requested.connect(self.handle_connection_request)
        self.settings_panel.debug_command_sent.connect(self.handle_debug_command)

        # Sinyal dari WaypointsPanel
        self.waypoints_panel.send_waypoints.connect(self.send_waypoints)
        self.waypoints_panel.send_waypoints.connect(self.map_view.update_waypoints)


    @Slot(str)
    def handle_mode_change(self, mode):
        self.api_client.send_command("CHANGE_MODE", payload=mode)

    @Slot()
    def emergency_stop(self):
        self.api_client.send_command("EMERGENCY_STOP")

    @Slot(str)
    def handle_manual_command(self, command):
        self.api_client.send_command("MANUAL_CONTROL", payload=command)

    @Slot(str)
    def handle_navigation_command(self, command):
        """Menerima perintah navigasi dan meneruskannya ke backend."""
        self.api_client.send_command("NAVIGATION", payload=command)

    @Slot(dict)
    def update_pid_constants(self, pid_data):
        self.api_client.send_command("UPDATE_PID", payload=pid_data)
        
    @Slot(dict)
    def update_servo_settings(self, servo_data):
        self.api_client.send_command("UPDATE_SERVO", payload=servo_data)
        
    @Slot(dict)
    def handle_connection_request(self, details):
        self.api_client.configure_and_connect(details)
        
    @Slot(str, object)
    def handle_debug_command(self, command_type, value):
        self.api_client.send_command("DEBUG", payload={"type": command_type, "value": value})

    @Slot(list)
    def send_waypoints(self, waypoints_list):
        self.api_client.send_command("SET_WAYPOINTS", payload=waypoints_list)
        
    @Slot(dict)
    def update_ui_from_state(self, data):
        mode = data.get("control_mode", "MANUAL")
        self.control_panel.set_mode(mode)

    def closeEvent(self, event):
        self.api_client.disconnect()
        self.video_view.stop_feed()
        super().closeEvent(event)
