# gui/components/settings_panel.py
# --- MODIFIKASI: Menerima dan meneruskan objek 'config' ke semua tab ---

from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QTabWidget
from PySide6.QtCore import Signal

# Impor semua komponen tab
from .pid_view import PidView
from .servo_view import ServoView
from .connection_view import ConnectionView
from .debug_panel import DebugPanel
from .thruster_view import ThrusterView  # <-- Impor ThrusterView


class SettingsPanel(QGroupBox):
    """
    Panel pengaturan yang menggunakan Tab untuk mengorganisir
    berbagai macam pengaturan.
    """

    # Teruskan sinyal dari setiap tab ke atas
    pid_updated = Signal(dict)
    servo_settings_updated = Signal(dict)
    connect_requested = Signal(dict)
    debug_command_sent = Signal(str, object)
    manual_speed_changed = Signal(int)  # <-- Sinyal baru

    # --- 1. UBAH TANDA TANGAN FUNGSI __init__ ---
    def __init__(self, config, title="Settings"):
        super().__init__(title)

        # --- 2. SIMPAN OBJEK KONFIGURASI ---
        self.config = config

        main_layout = QVBoxLayout()
        self.tab_widget = QTabWidget()

        # --- 3. TERUSKAN 'config' SAAT INISIALISASI SETIAP TAB ---
        # Catatan: PidView sekarang tidak memerlukan argumen title karena akan mengambilnya dari config
        self.pid_tab = PidView(config=self.config)
        self.servo_tab = ServoView(config=self.config)
        self.thruster_tab = ThrusterView(config=self.config)
        self.connection_tab = ConnectionView(config=self.config)
        self.debug_tab = DebugPanel(config=self.config)

        # Tambahkan semua tab ke widget tab
        self.tab_widget.addTab(self.pid_tab, "PID")
        self.tab_widget.addTab(self.servo_tab, "Servo")
        self.tab_widget.addTab(self.thruster_tab, "Thruster")  # <-- Tambahkan tab baru
        self.tab_widget.addTab(self.connection_tab, "Connection")
        self.tab_widget.addTab(self.debug_tab, "Debug")

        main_layout.addWidget(self.tab_widget)
        self.setLayout(main_layout)

        # Teruskan sinyal dari setiap view ke sinyal milik SettingsPanel
        self.pid_tab.pid_updated.connect(self.pid_updated.emit)
        self.servo_tab.servo_settings_updated.connect(self.servo_settings_updated.emit)
        self.connection_tab.connect_requested.connect(self.connect_requested.emit)
        self.debug_tab.debug_command_sent.connect(self.debug_command_sent.emit)
        self.thruster_tab.speed_changed.connect(
            self.manual_speed_changed.emit
        )  # <-- Teruskan sinyal kecepatan
