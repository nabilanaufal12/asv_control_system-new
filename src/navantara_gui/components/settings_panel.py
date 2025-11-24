# src/navantara_gui/components/settings_panel.py
from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QTabWidget, QLabel, QSlider
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

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

    # Sinyal-sinyal ini akan dipancarkan ke MainWindow
    pid_updated = Signal(dict)
    servo_settings_updated = Signal(dict)
    connect_requested = Signal(dict)
    debug_command_sent = Signal(str, object)
    manual_speed_changed = Signal(int)

    # [BARU] Sinyal untuk update kecepatan AI Vision
    vision_speed_updated = Signal(int)

    def __init__(self, config, title="Settings"):
        super().__init__(title)

        self.config = config

        main_layout = QVBoxLayout()

        # --- [BARU] BAGIAN KONTROL KECEPATAN AI ---
        ai_speed_group = QGroupBox("AI Vision Speed Control")
        ai_layout = QVBoxLayout()

        self.lbl_ai_speed = QLabel("AI PWM: 1500 (Default)")
        self.lbl_ai_speed.setAlignment(Qt.AlignCenter)

        self.slider_ai_speed = QSlider(Qt.Horizontal)
        self.slider_ai_speed.setRange(1300, 1800)
        self.slider_ai_speed.setValue(1500)
        self.slider_ai_speed.setTickPosition(QSlider.TicksBelow)
        self.slider_ai_speed.setTickInterval(50)

        ai_layout.addWidget(self.lbl_ai_speed)
        ai_layout.addWidget(self.slider_ai_speed)
        ai_speed_group.setLayout(ai_layout)

        # Tambahkan ke layout utama sebelum tab widget
        main_layout.addWidget(ai_speed_group)
        # ------------------------------------------

        self.tab_widget = QTabWidget()

        # Atur font agar lebih kecil dan rapi
        tab_font = QFont()
        tab_font.setPointSize(9)
        self.tab_widget.setFont(tab_font)
        self.tab_widget.tabBar().setUsesScrollButtons(True)

        # Inisialisasi semua widget tab
        self.pid_tab = PidView(config=self.config)
        self.servo_tab = ServoView(config=self.config)
        self.thruster_tab = ThrusterView(config=self.config)
        self.connection_tab = ConnectionView(config=self.config)
        self.debug_tab = DebugPanel(config=self.config)

        # Tambahkan semua tab ke widget tab utama
        self.tab_widget.addTab(self.pid_tab, "PID")
        self.tab_widget.addTab(self.servo_tab, "Servo")
        self.tab_widget.addTab(self.thruster_tab, "Thruster")
        self.tab_widget.addTab(self.connection_tab, "Connection")
        self.tab_widget.addTab(self.debug_tab, "Debug")

        main_layout.addWidget(self.tab_widget)
        self.setLayout(main_layout)

        # --- KONEKSI SINYAL INTERNAL ---
        self.pid_tab.pid_updated.connect(self.pid_updated.emit)
        self.servo_tab.servo_settings_updated.connect(self.servo_settings_updated.emit)
        self.connection_tab.connect_requested.connect(self.connect_requested.emit)
        self.debug_tab.debug_command_sent.connect(self.debug_command_sent.emit)
        self.thruster_tab.speed_changed.connect(self.manual_speed_changed.emit)

        # [BARU] Koneksi slider AI Speed
        self.slider_ai_speed.valueChanged.connect(self._on_ai_speed_changed)

    def _on_ai_speed_changed(self, value):
        self.lbl_ai_speed.setText(f"AI PWM: {value}")
        self.vision_speed_updated.emit(value)
