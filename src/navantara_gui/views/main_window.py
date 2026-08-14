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
    QLabel,
)
from PySide6.QtCore import Slot, Qt

from navantara_gui.components.control_panel import ControlPanel
from navantara_gui.components.dashboard import Dashboard
from navantara_gui.components.settings_panel import SettingsPanel
from navantara_gui.components.video_view import VideoView
from navantara_gui.components.header import Header
from navantara_gui.components.waypoints_panel import WaypointsPanel

from navantara_gui.missions import get_lintasan_a, get_lintasan_b
from navantara_gui.api_client import ApiClient


class MainWindow(QMainWindow):
    def __init__(self, config):
        super().__init__()
        self.setWindowTitle("ASV Ground Control Station - Navantara")
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

        self.waypoints_panel = WaypointsPanel(config=self.config)

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

        # Load Lintasan A by default
        self.load_predefined_mission("A")

        print("Memulai koneksi klien API ke server...")
        self.api_client.connect_to_server()

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
        # --- Sidebar Kiri (Tetap) ---
        layout_sidebar_kiri = QVBoxLayout()
        layout_sidebar_kiri.setAlignment(Qt.AlignTop)  # Pastikan semua merapat ke atas

        # Tambahkan tombol kontrol stream ke dalam GroupBox
        from PySide6.QtWidgets import QHBoxLayout, QGroupBox

        video_ctrl_layout = QHBoxLayout()
        video_ctrl_layout.addWidget(self.video_view.invert_button)
        video_ctrl_layout.addWidget(self.video_view.start_stop_button)

        video_group = QGroupBox("Live Video Stream")
        video_group.setLayout(video_ctrl_layout)

        layout_sidebar_kiri.addWidget(video_group)
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

        # --- [MODIFIKASI] Tengah: Hapus QTabWidget, gunakan VideoView langsung ---
        # Sebelumnya: self.tab_tengah = QTabWidget() ...
        # Sekarang: VideoView menjadi widget utama di tengah

        # --- Sidebar Kanan ---

        # 1. Waypoints Panel (Bisa di-scroll)
        scroll_area_kanan = QScrollArea()
        scroll_area_kanan.setWidget(self.waypoints_panel)
        scroll_area_kanan.setWidgetResizable(True)
        scroll_area_kanan.setFrameShape(QScrollArea.NoFrame)
        scroll_area_kanan.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # 2. Kontainer Utama Sidebar Kanan
        layout_sidebar_kanan = QVBoxLayout()
        layout_sidebar_kanan.setContentsMargins(0, 0, 0, 0)
        layout_sidebar_kanan.addWidget(
            scroll_area_kanan, 1
        )  # Stretch factor 1 agar mengisi ruang sisa
        layout_sidebar_kanan.addWidget(
            self.system_status_panel, 0
        )  # Stretch factor 0 agar ukurannya fixed di bawah

        widget_sidebar_kanan = QWidget()
        widget_sidebar_kanan.setLayout(layout_sidebar_kanan)

        # --- Layout Tengah (Video & Fitur Masa Depan) ---
        layout_tengah = QVBoxLayout()
        layout_tengah.setAlignment(Qt.AlignTop)

        # Tetapkan tinggi minimum agar video tidak menyusut saat ditarik ke atas
        self.video_view.setMinimumHeight(450)

        layout_tengah.addWidget(self.video_view)
        layout_tengah.addStretch()  # Mendorong video ke atas, menyisakan ruang kosong di bawah

        widget_tengah = QWidget()
        widget_tengah.setLayout(layout_tengah)

        # --- Splitter Utama ---
        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.addWidget(scroll_area_kiri)
        main_splitter.addWidget(widget_tengah)  # Menggunakan Container Tengah
        main_splitter.addWidget(widget_sidebar_kanan)

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

        # --- Persistent Status Bar Labels ---
        self.backend_status_lbl = QLabel("GCS Backend Server: DISCONNECTED")
        self.backend_status_lbl.setStyleSheet(
            "font-weight: bold; color: red; margin-right: 20px;"
        )

        self.esp_status_lbl = QLabel("ASV Serial Link (ESP32): DISCONNECTED")
        self.esp_status_lbl.setStyleSheet("font-weight: bold; color: red;")

        self.status_bar.addWidget(self.backend_status_lbl)
        self.status_bar.addWidget(self.esp_status_lbl)
        self.status_bar.showMessage(
            "Aplikasi Siap (Lite Mode). Menunggu koneksi ke backend..."
        )

    def connect_signals(self):
        """Menghubungkan semua sinyal dan slot antar komponen."""
        self.header.theme_changed_requested.connect(self.toggle_theme)

        self.video_view.toggle_camera_requested.connect(
            self.api_client.request_data_stream
        )

        self.video_view.inversion_changed.connect(
            lambda is_swapped: self.api_client.send_command(
                "SWAP_CAMERAS", {"swapped": is_swapped}
            )
        )

        self.settings_panel.vision_speed_updated.connect(
            lambda val: self.api_client.send_command(
                "UPDATE_VISION_SPEED", {"pwm": val}
            )
        )

        # --- [TAMBAHAN BARU] Koneksi Motor Depan (Kiri & Kanan) ---
        self.settings_panel.vision_front_motor_updated.connect(
            lambda payload: self.api_client.send_command(
                "UPDATE_VISION_FRONT_MOTOR", payload
            )
        )
        # ----------------------------------------------------------

        self.settings_panel.vision_servo_updated.connect(
            lambda p: self.api_client.send_command("UPDATE_VISION_SERVO", p)
        )

        self.settings_panel.vision_distance_updated.connect(
            lambda d: self.api_client.send_command(
                "UPDATE_VISION_DISTANCE", {"distance": d}
            )
        )

        self.settings_panel.vision_model_updated.connect(
            lambda m: self.api_client.send_command("UPDATE_VISION_MODEL", {"model": m})
        )

        self.settings_panel.vision_wp_ranges_updated.connect(
            lambda p: self.api_client.send_command("UPDATE_VISION_WP_RANGES", p)
        )

        self.waypoints_panel.send_waypoints.connect(
            lambda wps: self.api_client.send_command("SET_WAYPOINTS", wps)
        )
        self.waypoints_panel.request_wp_sync.connect(
            lambda: self.api_client.send_command("REQUEST_WP_SYNC", {})
        )
        self.waypoints_panel.add_current_pos_requested.connect(self.on_add_current_pos)
        self.api_client.connection_status_changed.connect(
            self.on_connection_status_change
        )
        self.api_client.data_updated.connect(self.on_data_updated)

        # Koneksi untuk sinkronisasi waypoint 2-arah
        self.api_client.sync_waypoints_received.connect(
            self.waypoints_panel.sync_waypoints_from_backend
        )

        self.waypoints_panel.replace_with_live_gps_requested.connect(
            self.on_replace_live_gps
        )
        self.waypoints_panel.arm_replace_rc_requested.connect(self.on_arm_replace_rc)

        self.waypoints_panel.load_mission_requested.connect(
            self.load_predefined_mission
        )
        self.api_client.frame_cam1_updated.connect(self.video_view.update_frame_1)
        self.api_client.frame_cam2_updated.connect(self.video_view.update_frame_2)

        self.settings_panel.send_photo_mission.connect(
            lambda payload: self.api_client.send_command("SET_PHOTO_MISSION", payload)
        )
        self.settings_panel.send_portrait_config.connect(
            lambda payload: self.api_client.send_command("SET_PORTRAIT_CONFIG", payload)
        )
        self.settings_panel.send_dock_config.connect(
            lambda payload: self.api_client.send_command("SET_DOCK_CONFIG", payload)
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

    @Slot(str)
    def set_mode(self, mode):
        """Fungsi terpusat untuk mengubah mode operasi."""
        self.current_control_mode = mode
        self.api_client.send_command("CHANGE_MODE", {"mode": mode})

        self.update_button_states()
        self.setFocus()

    def update_button_states(self):
        # Tombol-tombol kontrol sudah dihapus. Method ini dibiarkan kosong
        # atau untuk keperluan tombol masa depan.
        pass

    @Slot(dict)
    def on_data_updated(self, data):
        # Mendukung baik key asli maupun short-key (minified)
        self.current_latitude = data.get(
            "lat", data.get("latitude", self.current_latitude)
        )
        self.current_longitude = data.get(
            "lon", data.get("longitude", self.current_longitude)
        )

        status_text = data.get("status", "")
        self.is_rc_override = "RC MANUAL OVERRIDE" in status_text.upper()

        # Update ESP32 status
        is_esp_connected = data.get("conn", data.get("is_connected_to_serial", False))
        if is_esp_connected:
            self.esp_status_lbl.setText("ASV Serial Link (ESP32): CONNECTED")
            self.esp_status_lbl.setStyleSheet("font-weight: bold; color: #2ecc71;")
        else:
            self.esp_status_lbl.setText("ASV Serial Link (ESP32): DISCONNECTED")
            self.esp_status_lbl.setStyleSheet("font-weight: bold; color: red;")

        # Perbarui semua panel
        self.system_status_panel.update_data(data)

        self.header.update_status(data)
        self.update_button_states()

    @Slot()
    def on_add_current_pos(self):
        self.waypoints_panel.add_waypoint_from_pos(
            self.current_latitude, self.current_longitude
        )

    @Slot(int)
    def on_replace_live_gps(self, index):
        if (
            self.current_latitude is not None
            and self.current_longitude is not None
            and self.current_latitude != 0.0
        ):
            # Ganti baris di tabel GUI
            waypoint_text = f"[{index}] Lat: {self.current_latitude:.6f}, Lon: {self.current_longitude:.6f}"
            item = self.waypoints_panel.waypoints_list.item(index)
            if item:
                item.setText(waypoint_text)
                self.waypoints_panel._emit_updated_waypoints()

                # Kirim sinyal spesifik REPLACE ke backend
                waypoint = {"lat": self.current_latitude, "lon": self.current_longitude}
                self.api_client.send_command(
                    "REPLACE_WAYPOINT", {"index": index, "waypoint": waypoint}
                )
                print(f"[GUI] Titik {index} diganti satuan via Live GPS.")
        else:
            print(
                "[GUI] Error: Posisi kapal belum valid untuk replace (GPS tidak lock)."
            )

    @Slot(int)
    def on_arm_replace_rc(self, index):
        self.api_client.send_command("ARM_REPLACE_WP", {"index": index})

    @Slot(bool, str)
    def on_connection_status_change(self, is_connected, message):
        # Update Backend status label
        if is_connected:
            self.backend_status_lbl.setText("GCS Backend Server: CONNECTED")
            self.backend_status_lbl.setStyleSheet(
                "font-weight: bold; color: #2ecc71; margin-right: 20px;"
            )
            # Sinkronisasi otomatis pengaturan GUI Windows ke Jetson Backend
            self.settings_panel.broadcast_all_settings()
        else:
            self.backend_status_lbl.setText("GCS Backend Server: DISCONNECTED")
            self.backend_status_lbl.setStyleSheet(
                "font-weight: bold; color: red; margin-right: 20px;"
            )

            # Jika backend putus, ESP32 pasti putus
            self.esp_status_lbl.setText("ASV Serial Link (ESP32): DISCONNECTED")
            self.esp_status_lbl.setStyleSheet("font-weight: bold; color: red;")

        self.status_bar.showMessage(message, 3000)  # Tampilkan pesan sementara 3 detik

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
            arena = mission_data.get("arena")
            print(f"Mengirim konfigurasi {arena} langsung ke sistem...")

            # Set current arena
            self.waypoints_panel.current_arena = arena
            # Langsung kirim perintah update arena ke backend tanpa menyentuh waypoints
            self.waypoints_panel.send_waypoints.emit({"arena": arena})

    # Keyboard events for manual drive removed
