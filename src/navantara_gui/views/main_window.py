# src/navantara_gui/views/main_window.py
import sys
import os
import logging

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
    QStatusBar,
    QScrollArea,
    QApplication,
    QSplitter,
)
from PySide6.QtCore import Slot, Qt, QTimer
from PySide6.QtGui import QPixmap

# --- [MODIFIKASI] Menghapus import MapView dan LogPanel ---
from navantara_gui.components.control_panel import ControlPanel
from navantara_gui.components.dashboard import Dashboard
from navantara_gui.components.settings_panel import SettingsPanel
from navantara_gui.components.video_view import VideoView

# MapView dihapus
from navantara_gui.components.header import Header
from navantara_gui.components.waypoints_panel import WaypointsPanel

# LogPanel dihapus
from navantara_gui.missions import get_lintasan_a, get_lintasan_b
from navantara_gui.api_client import ApiClient


class MainWindow(QMainWindow):
    def __init__(self, config):
        super().__init__()
        self.setWindowTitle("ASV Control System - Navantara Client (Lite Version)")
        self.config = config

        self.api_client = ApiClient(config=self.config)

        self.current_latitude = 0.0
        self.current_longitude = 0.0

        # [FIX 1] Set default internal mode ke AUTO agar konsisten dengan Backend
        self.current_control_mode = "AUTO"

        self.is_rc_override = False  # Lacak status override RC

        self.header = Header(config=self.config)
        self.control_panel = ControlPanel(config=self.config)
        self.system_status_panel = Dashboard(config=self.config)
        self.settings_panel = SettingsPanel(config=self.config)
        self.video_view = VideoView(config=self.config)

        # --- [MODIFIKASI] Menghapus inisialisasi MapView dan LogPanel ---
        # self.map_view = MapView(config=self.config)  <- Dihapus
        self.waypoints_panel = WaypointsPanel(config=self.config)
        # self.log_panel = LogPanel(config=self.config) <- Dihapus

        self.active_manual_keys = set()

        self.current_theme = "light"
        self.themes = {}
        self._load_themes()
        self._apply_theme(self.current_theme)

        self.setup_ui()
        self.connect_signals()

        # [FIX 2] Panggil set_mode("AUTO") saat startup
        # Ini akan memperbarui tombol di ControlPanel dan mengirim perintah ke Backend
        self.set_mode("AUTO")

        print("Memulai koneksi klien API ke server...")
        self.api_client.connect_to_server()

        # Load Lintasan A as default trajectory after allowing time for connection
        QTimer.singleShot(1500, lambda: self.load_predefined_mission("A"))

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
        # =======================================================
        # --- LEFT SIDEBAR: Tabbed Layout ---
        # =======================================================
        layout_sidebar_kiri = QVBoxLayout()
        layout_sidebar_kiri.setContentsMargins(4, 4, 4, 4)
        layout_sidebar_kiri.setSpacing(6)

        from PySide6.QtWidgets import QGroupBox, QHBoxLayout, QTabWidget

        # --- ALWAYS VISIBLE: Start Stream button ---
        stream_group = QGroupBox("Camera Stream")
        stream_layout = QHBoxLayout()
        stream_layout.addWidget(self.video_view.invert_button)
        stream_layout.addWidget(self.video_view.start_stop_button)
        stream_group.setLayout(stream_layout)
        layout_sidebar_kiri.addWidget(stream_group)

        # --- TABBED WIDGET ---
        self.left_tabs = QTabWidget()
        self.left_tabs.setStyleSheet(
            "QTabBar::tab { padding: 6px 12px; font-weight: bold; }"
        )

        # ─── Tab 1: 🎮 Manual Drive ───
        tab_manual = QWidget()
        tab_manual_layout = QVBoxLayout(tab_manual)
        tab_manual_layout.setContentsMargins(4, 8, 4, 4)

        from PySide6.QtWidgets import QGridLayout

        # Camera Capture - reparent from control_panel
        from PySide6.QtWidgets import QLabel

        capture_group = QGroupBox("Camera Capture")
        capture_layout = QGridLayout()
        capture_layout.addWidget(QLabel("<b>Surface (CAM1):</b>"), 0, 0, 1, 2)
        capture_layout.addWidget(self.control_panel.btn_surf_overlay, 1, 0)
        capture_layout.addWidget(self.control_panel.btn_surf_raw, 1, 1)
        capture_layout.addWidget(QLabel("<b>Underwater (CAM2):</b>"), 2, 0, 1, 2)
        capture_layout.addWidget(self.control_panel.btn_under_overlay, 3, 0)
        capture_layout.addWidget(self.control_panel.btn_under_raw, 3, 1)
        capture_group.setLayout(capture_layout)
        tab_manual_layout.addWidget(capture_group)

        tab_manual_layout.addStretch()
        self.left_tabs.addTab(tab_manual, "🎮 Manual Drive")

        # ─── Tab 2: 🤖 Auto Mission ───
        tab_auto = QWidget()
        tab_auto_layout = QVBoxLayout(tab_auto)
        tab_auto_layout.setContentsMargins(4, 8, 4, 4)



        ai_group = QGroupBox("AI Vision Mission Control")
        ai_layout = QVBoxLayout()

        # Reparent profil misi dari settings_panel
        for child in self.settings_panel.findChildren(QGroupBox):
            if child.title() in [
                "Misi Bola (Merah-Hijau)",
                "Misi Kotak (Biru-Hijau)",
            ]:
                ai_layout.addWidget(child)

        ai_group.setLayout(ai_layout)
        tab_auto_layout.addWidget(ai_group)

        # Reparent Docking Config dari settings_panel
        tab_auto_layout.addWidget(self.settings_panel.docking_view)

        tab_auto_layout.addStretch()
        self.left_tabs.addTab(tab_auto, "🤖 Auto Mission")

        # ─── Tab 3: ⚙️ System Tuning ───
        tab_tuning = QWidget()
        tab_tuning_layout = QVBoxLayout(tab_tuning)
        tab_tuning_layout.setContentsMargins(4, 8, 4, 4)

        # PID Controller Settings - reuse existing tab widget from settings_panel
        pid_group = QGroupBox("PID Controller Settings")
        pid_layout = QVBoxLayout()
        pid_layout.addWidget(self.settings_panel.tab_widget)
        pid_group.setLayout(pid_layout)
        tab_tuning_layout.addWidget(pid_group)

        tab_tuning_layout.addStretch()
        self.left_tabs.addTab(tab_tuning, "⚙️ System Tuning")

        # --- Add tabs to left sidebar ---
        layout_sidebar_kiri.addWidget(self.left_tabs, 1)

        widget_sidebar_kiri = QWidget()
        widget_sidebar_kiri.setLayout(layout_sidebar_kiri)

        scroll_area_kiri = QScrollArea()
        scroll_area_kiri.setWidget(widget_sidebar_kiri)
        scroll_area_kiri.setWidgetResizable(True)
        scroll_area_kiri.setFrameShape(QScrollArea.NoFrame)
        scroll_area_kiri.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # --- [MODIFIKASI] Tengah: Hapus QTabWidget, gunakan VideoView langsung ---
        # Sebelumnya: self.tab_tengah = QTabWidget() ...
        # Sekarang: VideoView menjadi widget utama di tengah

        # =======================================================
        # --- RIGHT SIDEBAR: Tabbed Top + Always-Visible Bottom ---
        # =======================================================
        layout_sidebar_kanan = QVBoxLayout()
        layout_sidebar_kanan.setContentsMargins(4, 4, 4, 4)
        layout_sidebar_kanan.setSpacing(6)

        # --- TABBED SECTION (Top) ---
        self.right_tabs = QTabWidget()
        self.right_tabs.setStyleSheet(
            "QTabBar::tab { padding: 6px 12px; font-weight: bold; }"
        )

        # ─── Tab 1: 📍 Route Planner ───
        tab_route = QWidget()
        tab_route_layout = QVBoxLayout(tab_route)
        tab_route_layout.setContentsMargins(4, 8, 4, 4)

        # Predefined Missions - reparent from waypoints_panel
        from PySide6.QtWidgets import QGroupBox, QFormLayout

        mission_box = QGroupBox("Predefined Missions")
        mission_layout_h = QHBoxLayout()
        mission_layout_h.addWidget(self.waypoints_panel.load_a_button)
        mission_layout_h.addWidget(self.waypoints_panel.load_b_button)
        mission_box.setLayout(mission_layout_h)
        tab_route_layout.addWidget(mission_box)

        # Manual Input - reparent from waypoints_panel
        input_form = QFormLayout()
        input_form.addRow("Latitude:", self.waypoints_panel.lat_input)
        input_form.addRow("Longitude:", self.waypoints_panel.lon_input)
        tab_route_layout.addLayout(input_form)

        # Waypoints List - reparent from waypoints_panel
        tab_route_layout.addWidget(self.waypoints_panel.waypoints_list, 1)

        # Add/Delete buttons - reparent from waypoints_panel
        btn_row = QHBoxLayout()
        btn_row.addWidget(self.waypoints_panel.add_manual_button)
        btn_row.addWidget(self.waypoints_panel.add_current_pos_button)
        btn_row.addWidget(self.waypoints_panel.delete_button)
        tab_route_layout.addLayout(btn_row)

        # Send All - reparent from waypoints_panel
        send_row = QHBoxLayout()
        send_row.addStretch()
        send_row.addWidget(self.waypoints_panel.send_all_button)
        tab_route_layout.addLayout(send_row)

        self.right_tabs.addTab(tab_route, "📍 Route Planner")

        # ─── Tab 2: ⚙️ WP Actions ───
        tab_wp_actions = QWidget()
        tab_wp_actions_layout = QVBoxLayout(tab_wp_actions)
        tab_wp_actions_layout.setContentsMargins(4, 8, 4, 4)

        # Misi Fotografi - reparent from waypoints_panel
        tab_wp_actions_layout.addWidget(self.waypoints_panel.photo_mission_widget)

        # Konfigurasi Inversi Servo - reparent from waypoints_panel
        inversion_box = QGroupBox("Waypoint Payload/Triggers (Inversi)")
        inversion_layout_h = QHBoxLayout()
        inversion_layout_h.addWidget(self.waypoints_panel.trigger_wp_input)
        inversion_layout_h.addWidget(self.waypoints_panel.set_trigger_btn)
        inversion_box.setLayout(inversion_layout_h)
        tab_wp_actions_layout.addWidget(inversion_box)

        tab_wp_actions_layout.addStretch()
        self.right_tabs.addTab(tab_wp_actions, "📸 WP Actions (Auto)")

        layout_sidebar_kanan.addWidget(self.right_tabs, 1)

        # --- ALWAYS VISIBLE SECTION (Bottom): Telemetry Data ---
        # Reparent sensor_display and status_monitor from system_status_panel (Dashboard)
        layout_sidebar_kanan.addWidget(self.system_status_panel.sensor_display)
        layout_sidebar_kanan.addWidget(self.system_status_panel.status_monitor)

        widget_sidebar_kanan = QWidget()
        widget_sidebar_kanan.setLayout(layout_sidebar_kanan)

        scroll_area_kanan = QScrollArea()
        scroll_area_kanan.setWidget(widget_sidebar_kanan)
        scroll_area_kanan.setWidgetResizable(True)
        scroll_area_kanan.setFrameShape(QScrollArea.NoFrame)
        scroll_area_kanan.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # --- Splitter Utama ---
        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.addWidget(scroll_area_kiri)
        main_splitter.addWidget(self.video_view)  # Menggunakan VideoView langsung
        main_splitter.addWidget(scroll_area_kanan)

        gui_settings = self.config.get("gui_settings", {})
        splitter_sizes = gui_settings.get("main_splitter_sizes", [350, 800, 350])
        main_splitter.setSizes(splitter_sizes)
        main_splitter.setCollapsible(0, False)
        main_splitter.setCollapsible(2, False)

        # --- Layout Keseluruhan ---
        layout_keseluruhan = QVBoxLayout()
        layout_keseluruhan.addWidget(self.header, 0)
        layout_keseluruhan.addWidget(main_splitter, 1)

        widget_pusat = QWidget()
        widget_pusat.setLayout(layout_keseluruhan)
        self.setCentralWidget(widget_pusat)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage(
            "Aplikasi Siap (Lite Mode). Menunggu koneksi ke backend..."
        )

    def connect_signals(self):
        """Menghubungkan semua sinyal dan slot antar komponen."""
        self.header.theme_changed_requested.connect(self.toggle_theme)

        self.video_view.toggle_camera_requested.connect(
            self.api_client.request_data_stream
        )

        self.api_client.log_received.connect(self.video_view.append_log)
        self.api_client.serial_connection_status.connect(
            self.header.update_connection_status
        )

        # --- [BARU] Hubungkan tombol "Invert Logic" ke perintah SWAP_CAMERAS ---
        self.video_view.inversion_changed.connect(
            lambda _: self.api_client.send_command("SWAP_CAMERAS", {})
        )

        self.settings_panel.debug_command_sent.connect(self.api_client.send_command)

        # --- [TAMBAHAN BARU] Koneksi PID, Servo, dan Thruster ---
        self.settings_panel.pid_updated.connect(
            lambda payload: self.api_client.send_command("UPDATE_PID", payload)
        )
        self.settings_panel.servo_settings_updated.connect(
            lambda payload: self.api_client.send_command("UPDATE_SERVO", payload)
        )
        self.settings_panel.manual_speed_changed.connect(
            lambda speed: self.api_client.send_command(
                "UPDATE_THRUSTER", {"speed": speed}
            )
        )
        # -----------------------------------

        self.settings_panel.vision_mission_updated.connect(
            lambda p: self.api_client.send_command("UPDATE_MISSION_CONFIG", p)
        )
        self.settings_panel.docking_config_updated.connect(
            lambda p: self.api_client.send_command("UPDATE_MISSION_CONFIG", p)
        )

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

        self.waypoints_panel.send_photo_mission.connect(
            lambda payload: self.api_client.send_command(
                "UPDATE_MISSION_CONFIG", payload
            )
        )

        self.waypoints_panel.update_inversion_trigger.connect(
            lambda p: self.api_client.send_command("UPDATE_INVERSION_TRIGGER", p)
        )

        # --- Koneksi Capture RAW/Overlay ---

        # 1. Surface Capture
        self.control_panel.surface_overlay_clicked.connect(
            lambda: self.api_client.send_command(
                "MANUAL_CAPTURE", {"type": "surface", "raw": False}
            )
        )
        self.control_panel.surface_raw_clicked.connect(
            lambda: self.api_client.send_command(
                "MANUAL_CAPTURE", {"type": "surface", "raw": True}
            )
        )

        # 2. Underwater Capture
        self.control_panel.underwater_overlay_clicked.connect(
            lambda: self.api_client.send_command(
                "MANUAL_CAPTURE", {"type": "underwater", "raw": False}
            )
        )
        self.control_panel.underwater_raw_clicked.connect(
            lambda: self.api_client.send_command(
                "MANUAL_CAPTURE", {"type": "underwater", "raw": True}
            )
        )

    def update_button_states(self):
        pass

    @Slot(dict)
    def on_data_updated(self, data):
        self.current_latitude = data.get("latitude", self.current_latitude)
        self.current_longitude = data.get("longitude", self.current_longitude)

        status_text = data.get("status", "")
        self.is_rc_override = "RC MANUAL OVERRIDE" in status_text.upper()

        # [FIX] Sinkronisasi mode dari ESP telemetry ke GUI buttons
        # Jika ESP melaporkan mode berbeda (misal RC switch digeser),
        # update tampilan GUI tanpa mengirim perintah balik ke backend
        esp_reported_mode = data.get("mode")
        if esp_reported_mode and esp_reported_mode in ("MANUAL", "AUTO"):
            if esp_reported_mode != self.current_control_mode:
                logging.info(
                    f"[GUI] Mode sync dari ESP: {self.current_control_mode} -> {esp_reported_mode}"
                )
                self.current_control_mode = esp_reported_mode

        # Perbarui semua panel
        self.system_status_panel.update_data(data)

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

        # --- [FIX] Sinkronkan tombol Start/Stop Stream dengan status koneksi ---
        # Saat terhubung, backend otomatis mengirim stream (initial_stream_request),
        # jadi tombol harus langsung menunjukkan "Stop Stream".
        if is_connected:
            self.video_view.is_camera_running = True
            self.video_view.update_ui_controls(True)
        else:
            self.video_view.is_camera_running = False
            self.video_view.update_ui_controls(False)
            self.video_view.label_video_1.setText("Backend terputus.")
            self.video_view.label_video_1.setPixmap(QPixmap())
            self.video_view.label_video_2.setText("Backend terputus.")
            self.video_view.label_video_2.setPixmap(QPixmap())

    @Slot()
    def on_request_manual_capture_surface(self):
        print("[GUI] Meminta capture 'surface'...")
        self.api_client.send_command("MANUAL_CAPTURE", {"type": "surface"})

    @Slot()
    def on_request_manual_capture_underwater(self):
        print("[GUI] Meminta capture 'underwater'...")
        self.api_client.send_command("MANUAL_CAPTURE", {"type": "underwater"})

    @Slot(str)
    def load_predefined_mission(self, mission_id):
        mission_data = None
        if mission_id == "A":
            mission_data = get_lintasan_a()
        elif mission_id == "B":
            mission_data = get_lintasan_b()
        else:
            logging.warning(f"ID Arena tidak dikenal: {mission_id}")
            return

        if mission_data:
            waypoints_list = mission_data.get("waypoints")
            arena = mission_data.get("arena")

            print(f"Memuat Lintasan {arena}...")

            if waypoints_list is not None:
                self.waypoints_panel.load_waypoints_to_list(waypoints_list)
                # Automatically send the waypoints to the backend
                self.waypoints_panel.send_all_waypoints()
            else:
                logging.error(f"Gagal memuat misi {mission_id}: format data salah.")

