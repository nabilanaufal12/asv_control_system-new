# gui/components/settings_panel.py
# --- VERSI FINAL: Dengan font tab yang lebih kecil untuk layout fleksibel ---

from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QTabWidget
from PySide6.QtCore import Signal
from PySide6.QtGui import QFont  # <-- Impor QFont

# Impor semua komponen tab
from .pid_view import PidView
from .servo_view import ServoView
from .connection_view import ConnectionView
from .debug_panel import DebugPanel
from .thruster_view import ThrusterView


class SettingsPanel(QGroupBox):
    """
    Panel pengaturan yang menggunakan Tab untuk mengorganisir
    berbagai macam pengaturan.
    """

    pid_updated = Signal(dict)
    servo_settings_updated = Signal(dict)
    connect_requested = Signal(dict)
    debug_command_sent = Signal(str, object)
    manual_speed_changed = Signal(int)

    def __init__(self, config, title="Settings"):
        super().__init__(title)

        self.config = config

        main_layout = QVBoxLayout()
        self.tab_widget = QTabWidget()

        # --- PERBAIKAN UTAMA DI SINI ---
        # 1. Buat objek font baru dengan ukuran lebih kecil
        tab_font = QFont()
        tab_font.setPointSize(9)  # Anda bisa sesuaikan ukuran ini (misalnya 8 atau 9)

        # 2. Terapkan font kecil tersebut ke tab bar
        self.tab_widget.setFont(tab_font)

        # 3. (Opsional tapi direkomendasikan) Buat tab bar bisa digulir
        self.tab_widget.tabBar().setUsesScrollButtons(True)
        # --- AKHIR PERBAIKAN ---

        # Inisialisasi semua tab seperti biasa
        self.pid_tab = PidView(config=self.config)
        self.servo_tab = ServoView(config=self.config)
        self.thruster_tab = ThrusterView(config=self.config)
        self.connection_tab = ConnectionView(config=self.config)
        self.debug_tab = DebugPanel(config=self.config)

        # Tambahkan semua tab ke widget tab
        self.tab_widget.addTab(self.pid_tab, "PID")
        self.tab_widget.addTab(self.servo_tab, "Servo")
        self.tab_widget.addTab(self.thruster_tab, "Thruster")
        self.tab_widget.addTab(self.connection_tab, "Connection")
        self.tab_widget.addTab(self.debug_tab, "Debug")

        main_layout.addWidget(self.tab_widget)
        self.setLayout(main_layout)

        # Teruskan sinyal dari setiap view (tidak ada perubahan di sini)
        self.pid_tab.pid_updated.connect(self.pid_updated.emit)
        self.servo_tab.servo_settings_updated.connect(self.servo_settings_updated.emit)
        self.connection_tab.connect_requested.connect(self.connect_requested.emit)
        self.debug_tab.debug_command_sent.connect(self.debug_command_sent.emit)
        self.thruster_tab.speed_changed.connect(self.manual_speed_changed.emit)
