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
    vision_front_motor_updated = Signal(
        dict
    )  # [UBAH] Sekarang mengirim dict (Kiri & Kanan)
    vision_servo_updated = Signal(dict)
    vision_distance_updated = Signal(float)
    vision_mission_updated = Signal(dict)

    def __init__(self, config, title="Settings"):
        super().__init__(title)
        self.config = config
        main_layout = QVBoxLayout()

        # --- [BAGIAN KONTROL AI VISION & MISI] ---
        ai_control_group = QGroupBox("AI Vision & Mission Control")
        ai_layout = QVBoxLayout()

        # 1. Slider Kecepatan Motor Bawah (Utama)
        speed_layout = QHBoxLayout()
        self.lbl_ai_speed = QLabel("PWM Motor Utama:")
        self.slider_ai_speed = QSlider(Qt.Horizontal)
        self.slider_ai_speed.setRange(1100, 1800)
        self.slider_ai_speed.setValue(1300)

        self.spin_ai_speed = QSpinBox()
        self.spin_ai_speed.setRange(1100, 1800)
        self.spin_ai_speed.setValue(1300)
        self.spin_ai_speed.setFixedWidth(70)

        speed_layout.addWidget(self.lbl_ai_speed)
        speed_layout.addWidget(self.slider_ai_speed)
        speed_layout.addWidget(self.spin_ai_speed)

        # 1.5 [UBAH] 2 Slider Kecepatan Motor Depan (Kemudi Kiri & Kanan)
        front_speed_layout = QVBoxLayout()

        # --- Motor Depan Kiri ---
        left_front_layout = QHBoxLayout()
        self.lbl_front_left = QLabel("PWM Depan Kiri:")
        self.slider_front_left = QSlider(Qt.Horizontal)
        self.slider_front_left.setRange(1000, 1900)
        self.slider_front_left.setValue(1500)

        self.spin_front_left = QSpinBox()
        self.spin_front_left.setRange(1000, 1900)
        self.spin_front_left.setValue(1500)
        self.spin_front_left.setFixedWidth(70)

        left_front_layout.addWidget(self.lbl_front_left)
        left_front_layout.addWidget(self.slider_front_left)
        left_front_layout.addWidget(self.spin_front_left)

        # --- Motor Depan Kanan ---
        right_front_layout = QHBoxLayout()
        self.lbl_front_right = QLabel("PWM Depan Kanan:")
        self.slider_front_right = QSlider(Qt.Horizontal)
        self.slider_front_right.setRange(1000, 1900)
        self.slider_front_right.setValue(1500)

        self.spin_front_right = QSpinBox()
        self.spin_front_right.setRange(1000, 1900)
        self.spin_front_right.setValue(1500)
        self.spin_front_right.setFixedWidth(70)

        right_front_layout.addWidget(self.lbl_front_right)
        right_front_layout.addWidget(self.slider_front_right)
        right_front_layout.addWidget(self.spin_front_right)

        front_speed_layout.addLayout(left_front_layout)
        front_speed_layout.addLayout(right_front_layout)

        # 2. --- Profil Misi Bola (Merah-Hijau) ---
        bola_group = QGroupBox("Misi Bola (Merah-Hijau)")
        bola_layout = QVBoxLayout()

        bola_wp_row = QHBoxLayout()
        bola_wp_row.addWidget(QLabel("WP Range:"))
        self.spin_bola_wp_start = QSpinBox()
        self.spin_bola_wp_start.setRange(0, 30)
        self.spin_bola_wp_start.setValue(0)
        self.spin_bola_wp_start.setPrefix("Start: ")
        bola_wp_row.addWidget(self.spin_bola_wp_start)
        self.spin_bola_wp_end = QSpinBox()
        self.spin_bola_wp_end.setRange(0, 30)
        self.spin_bola_wp_end.setValue(11)
        self.spin_bola_wp_end.setPrefix("End: ")
        bola_wp_row.addWidget(self.spin_bola_wp_end)
        bola_layout.addLayout(bola_wp_row)

        bola_dist_row = QHBoxLayout()
        bola_dist_row.addWidget(QLabel("Trigger Dist:"))
        self.spin_bola_dist = QSpinBox()
        self.spin_bola_dist.setRange(0, 500)
        self.spin_bola_dist.setValue(165)
        self.spin_bola_dist.setSingleStep(10)
        self.spin_bola_dist.setSuffix(" cm")
        bola_dist_row.addWidget(self.spin_bola_dist)
        bola_layout.addLayout(bola_dist_row)

        bola_angle_row = QHBoxLayout()
        bola_angle_row.addWidget(QLabel("Avoid Angle:"))
        self.spin_bola_left = QSpinBox()
        self.spin_bola_left.setRange(0, 90)
        self.spin_bola_left.setValue(70)
        self.spin_bola_left.setPrefix("L: ")
        self.spin_bola_left.setSuffix("°")
        bola_angle_row.addWidget(self.spin_bola_left)
        self.spin_bola_right = QSpinBox()
        self.spin_bola_right.setRange(90, 180)
        self.spin_bola_right.setValue(110)
        self.spin_bola_right.setPrefix("R: ")
        self.spin_bola_right.setSuffix("°")
        bola_angle_row.addWidget(self.spin_bola_right)
        bola_layout.addLayout(bola_angle_row)

        bola_group.setLayout(bola_layout)

        # 3. --- Profil Misi Kotak (Biru-Hijau) ---
        kotak_group = QGroupBox("Misi Kotak (Biru-Hijau)")
        kotak_layout = QVBoxLayout()

        kotak_wp_row = QHBoxLayout()
        kotak_wp_row.addWidget(QLabel("WP Range:"))
        self.spin_kotak_wp_start = QSpinBox()
        self.spin_kotak_wp_start.setRange(0, 30)
        self.spin_kotak_wp_start.setValue(11)
        self.spin_kotak_wp_start.setPrefix("Start: ")
        kotak_wp_row.addWidget(self.spin_kotak_wp_start)
        self.spin_kotak_wp_end = QSpinBox()
        self.spin_kotak_wp_end.setRange(0, 30)
        self.spin_kotak_wp_end.setValue(15)
        self.spin_kotak_wp_end.setPrefix("End: ")
        kotak_wp_row.addWidget(self.spin_kotak_wp_end)
        kotak_layout.addLayout(kotak_wp_row)

        kotak_dist_row = QHBoxLayout()
        kotak_dist_row.addWidget(QLabel("Trigger Dist:"))
        self.spin_kotak_dist = QSpinBox()
        self.spin_kotak_dist.setRange(0, 500)
        self.spin_kotak_dist.setValue(165)
        self.spin_kotak_dist.setSingleStep(10)
        self.spin_kotak_dist.setSuffix(" cm")
        kotak_dist_row.addWidget(self.spin_kotak_dist)
        kotak_layout.addLayout(kotak_dist_row)

        kotak_angle_row = QHBoxLayout()
        kotak_angle_row.addWidget(QLabel("Avoid Angle:"))
        self.spin_kotak_left = QSpinBox()
        self.spin_kotak_left.setRange(0, 90)
        self.spin_kotak_left.setValue(70)
        self.spin_kotak_left.setPrefix("L: ")
        self.spin_kotak_left.setSuffix("°")
        kotak_angle_row.addWidget(self.spin_kotak_left)
        self.spin_kotak_right = QSpinBox()
        self.spin_kotak_right.setRange(90, 180)
        self.spin_kotak_right.setValue(110)
        self.spin_kotak_right.setPrefix("R: ")
        self.spin_kotak_right.setSuffix("°")
        kotak_angle_row.addWidget(self.spin_kotak_right)
        kotak_layout.addLayout(kotak_angle_row)

        kotak_group.setLayout(kotak_layout)

        # --- GABUNGKAN SEMUA KE AI LAYOUT ---
        ai_layout.addLayout(speed_layout)
        ai_layout.addLayout(front_speed_layout)
        ai_layout.addWidget(bola_group)
        ai_layout.addWidget(kotak_group)

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

        idx_servo = self.tab_widget.addTab(self.servo_tab, "Servo Trim / Range")
        self.tab_widget.setTabToolTip(
            idx_servo, "(Set hardware center and physical limits)"
        )

        idx_thruster = self.tab_widget.addTab(self.thruster_tab, "Thruster Calibration")
        self.tab_widget.setTabToolTip(
            idx_thruster, "(Set absolute ESC hardware limits)"
        )
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
        self.slider_ai_speed.valueChanged.connect(self.spin_ai_speed.setValue)
        self.spin_ai_speed.valueChanged.connect(self.slider_ai_speed.setValue)

        # [UBAH] Sambungkan kedua slider depan
        self.slider_front_left.valueChanged.connect(self._on_front_left_changed)
        self.slider_front_left.valueChanged.connect(self.spin_front_left.setValue)
        self.spin_front_left.valueChanged.connect(self.slider_front_left.setValue)

        self.slider_front_right.valueChanged.connect(self._on_front_right_changed)
        self.slider_front_right.valueChanged.connect(self.spin_front_right.setValue)
        self.spin_front_right.valueChanged.connect(self.slider_front_right.setValue)

        # --- [BARU] Koneksi Profil Misi Bola & Kotak ---
        mission_spinboxes = [
            self.spin_bola_wp_start,
            self.spin_bola_wp_end,
            self.spin_bola_dist,
            self.spin_bola_left,
            self.spin_bola_right,
            self.spin_kotak_wp_start,
            self.spin_kotak_wp_end,
            self.spin_kotak_dist,
            self.spin_kotak_left,
            self.spin_kotak_right,
        ]
        for sb in mission_spinboxes:
            sb.valueChanged.connect(self._on_mission_profile_changed)

    # --- SLOT HANDLERS ---
    def _on_ai_speed_changed(self, value):
        self.vision_speed_updated.emit(value)

    def _on_front_left_changed(self, value):
        # Kirim pasangan nilai saat ini ke backend
        right_val = self.slider_front_right.value()
        self.vision_front_motor_updated.emit({"left": value, "right": right_val})

    def _on_front_right_changed(self, value):
        left_val = self.slider_front_left.value()
        self.vision_front_motor_updated.emit({"left": left_val, "right": value})

    def _on_mission_profile_changed(self):
        """Kumpulkan semua data profil misi dan kirim ke backend."""
        payload = {
            "bola": {
                "wp_start": self.spin_bola_wp_start.value(),
                "wp_end": self.spin_bola_wp_end.value(),
                "trigger_dist": self.spin_bola_dist.value(),
                "angle_left": self.spin_bola_left.value(),
                "angle_right": self.spin_bola_right.value(),
            },
            "kotak": {
                "wp_start": self.spin_kotak_wp_start.value(),
                "wp_end": self.spin_kotak_wp_end.value(),
                "trigger_dist": self.spin_kotak_dist.value(),
                "angle_left": self.spin_kotak_left.value(),
                "angle_right": self.spin_kotak_right.value(),
            },
        }
        self.vision_mission_updated.emit(payload)
