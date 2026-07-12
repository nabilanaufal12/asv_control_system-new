# src/navantara_gui/components/settings_panel.py
from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QTabWidget,
    QLabel,
    QSlider,
    QHBoxLayout,
    QSpinBox,
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

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
    vision_speed_updated = Signal(int)
    vision_front_motor_updated = Signal(dict) # [UBAH] Sekarang mengirim dict (Kiri & Kanan)
    vision_servo_updated = Signal(dict)
    vision_distance_updated = Signal(float)

    def __init__(self, config, title="Settings"):
        super().__init__(title)
        self.config = config
        main_layout = QVBoxLayout()

        # --- [BAGIAN KONTROL AI VISION & MISI] ---
        ai_control_group = QGroupBox("AI Vision & Mission Control")
        ai_layout = QVBoxLayout()

        # 1. Slider Kecepatan Motor Bawah (Utama)
        speed_layout = QHBoxLayout()
        self.lbl_ai_speed = QLabel("PWM Motor Utama: 1300")
        self.slider_ai_speed = QSlider(Qt.Horizontal)
        self.slider_ai_speed.setRange(1100, 1800)
        self.slider_ai_speed.setValue(1300)
        speed_layout.addWidget(self.lbl_ai_speed)
        speed_layout.addWidget(self.slider_ai_speed)

        # 1.5 [UBAH] 2 Slider Kecepatan Motor Depan (Kemudi Kiri & Kanan)
        front_speed_layout = QVBoxLayout()
        
        # --- Motor Depan Kiri ---
        left_front_layout = QHBoxLayout()
        self.lbl_front_left = QLabel("PWM Depan Kiri: 1650")
        self.slider_front_left = QSlider(Qt.Horizontal)
        self.slider_front_left.setRange(1500, 1900)
        self.slider_front_left.setValue(1650)
        left_front_layout.addWidget(self.lbl_front_left)
        left_front_layout.addWidget(self.slider_front_left)
        
        # --- Motor Depan Kanan ---
        right_front_layout = QHBoxLayout()
        self.lbl_front_right = QLabel("PWM Depan Kanan: 1650")
        self.slider_front_right = QSlider(Qt.Horizontal)
        self.slider_front_right.setRange(1500, 1900)
        self.slider_front_right.setValue(1650)
        right_front_layout.addWidget(self.lbl_front_right)
        right_front_layout.addWidget(self.slider_front_right)

        front_speed_layout.addLayout(left_front_layout)
        front_speed_layout.addLayout(right_front_layout)

        # 2. Input Servo Kiri & Kanan
        servo_layout = QHBoxLayout()
        self.spin_left = QSpinBox()
        self.spin_left.setRange(0, 90)
        self.spin_left.setValue(70)
        self.spin_left.setPrefix("Left: ")
        self.spin_left.setSuffix("°")
        
        self.spin_right = QSpinBox()
        self.spin_right.setRange(90, 180)
        self.spin_right.setValue(110)
        self.spin_right.setPrefix("Right: ")
        self.spin_right.setSuffix("°")

        servo_layout.addWidget(QLabel("Avoidance Angle:"))
        servo_layout.addWidget(self.spin_left)
        servo_layout.addWidget(self.spin_right)

        # 3. Input Jarak Obstacle (cm)
        dist_layout = QHBoxLayout()
        self.spin_obs_dist = QSpinBox()
        self.spin_obs_dist.setRange(0, 500)
        self.spin_obs_dist.setValue(165)
        self.spin_obs_dist.setSingleStep(10)
        self.spin_obs_dist.setPrefix("Trigger Dist: ")
        self.spin_obs_dist.setSuffix(" cm")
        dist_layout.addWidget(QLabel("AI Activation:"))
        dist_layout.addWidget(self.spin_obs_dist)

        # --- GABUNGKAN SEMUA KE AI LAYOUT ---
        ai_layout.addLayout(speed_layout)
        ai_layout.addLayout(front_speed_layout)
        ai_layout.addLayout(servo_layout)
        ai_layout.addLayout(dist_layout)

        ai_control_group.setLayout(ai_layout)
        main_layout.addWidget(ai_control_group)

        # --- TAB WIDGET ---
        self.tab_widget = QTabWidget()
        tab_font = QFont()
        tab_font.setPointSize(9)
        self.tab_widget.setFont(tab_font)
        self.tab_widget.tabBar().setUsesScrollButtons(True)

        self.pid_tab = PidView(config=self.config)
        self.servo_tab = ServoView(config=self.config)
        self.thruster_tab = ThrusterView(config=self.config)
        self.connection_tab = ConnectionView(config=self.config)
        self.debug_tab = DebugPanel(config=self.config)

        self.tab_widget.addTab(self.pid_tab, "PID")
        self.tab_widget.addTab(self.servo_tab, "Servo")
        self.tab_widget.addTab(self.thruster_tab, "Thruster")
        self.tab_widget.addTab(self.connection_tab, "Connection")
        self.tab_widget.addTab(self.debug_tab, "Debug")

        main_layout.addWidget(self.tab_widget)
        self.setLayout(main_layout)

        # --- KONEKSI SINYAL ---
        self.pid_tab.pid_updated.connect(self.pid_updated.emit)
        self.servo_tab.servo_settings_updated.connect(self.servo_settings_updated.emit)
        self.connection_tab.connect_requested.connect(self.connect_requested.emit)
        self.debug_tab.debug_command_sent.connect(self.debug_command_sent.emit)
        self.thruster_tab.speed_changed.connect(self.manual_speed_changed.emit)

        # Koneksi Kontrol AI
        self.slider_ai_speed.valueChanged.connect(self._on_ai_speed_changed)
        
        # [UBAH] Sambungkan kedua slider depan ke fungsi yang sama
        self.slider_front_left.valueChanged.connect(self._on_front_speed_changed)
        self.slider_front_right.valueChanged.connect(self._on_front_speed_changed)
        
        self.spin_left.valueChanged.connect(self._on_ai_servo_changed)
        self.spin_right.valueChanged.connect(self._on_ai_servo_changed)
        self.spin_obs_dist.valueChanged.connect(self._on_obs_dist_changed)

    # --- SLOT HANDLERS ---
    def _on_ai_speed_changed(self, value):
        self.lbl_ai_speed.setText(f"PWM Motor Utama: {value}")
        self.vision_speed_updated.emit(value)

    def _on_front_speed_changed(self):
        val_l = self.slider_front_left.value()
        val_r = self.slider_front_right.value()
        self.lbl_front_left.setText(f"PWM Depan Kiri: {val_l}")
        self.lbl_front_right.setText(f"PWM Depan Kanan: {val_r}")
        # Kirim payload data ke backend
        self.vision_front_motor_updated.emit({"left": val_l, "right": val_r})

    def _on_ai_servo_changed(self):
        payload = {"left": self.spin_left.value(), "right": self.spin_right.value()}
        self.vision_servo_updated.emit(payload)

    def _on_obs_dist_changed(self, value):
        self.vision_distance_updated.emit(float(value))