# src/navantara_gui/components/settings_panel.py
from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QTabWidget,
    QLabel,
    QSlider,
    QSpinBox,
    QGridLayout,
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

from .pid_view import PidView
from .servo_view import ServoView
from .connection_view import ConnectionView
from .debug_panel import DebugPanel
from .thruster_view import ThrusterView
from .docking_view import DockingView


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
    vision_mission_updated = Signal(dict)
    vision_servo_updated = Signal(dict)
    vision_distance_updated = Signal(float)
    vision_mission_updated = Signal(dict)
    docking_config_updated = Signal(dict)

    def __init__(self, config, title="Settings"):
        super().__init__(title)
        self.config = config
        auto_missions = self.config.get("auto_missions", {})
        bola_cfg = auto_missions.get("vision_ball_red_green", {})
        kotak_cfg = auto_missions.get("vision_box_blue_green", {})

        main_layout = QVBoxLayout()

        # --- [BAGIAN KONTROL AI VISION & MISI] ---
        ai_control_group = QGroupBox("AI Vision & Mission Control")
        ai_layout = QVBoxLayout()

        # 1. --- Profil Misi Bola (Merah-Hijau) ---
        bola_group = QGroupBox("Misi Bola (Merah-Hijau)")
        bola_layout = QGridLayout()

        bola_layout.addWidget(QLabel("WP Range:"), 0, 0)
        self.spin_bola_wp_start = QSpinBox()
        self.spin_bola_wp_start.setRange(0, 30)
        self.spin_bola_wp_start.setValue(bola_cfg.get("wp_start", 0))
        self.spin_bola_wp_start.setPrefix("Start: ")
        bola_layout.addWidget(self.spin_bola_wp_start, 0, 1)
        self.spin_bola_wp_end = QSpinBox()
        self.spin_bola_wp_end.setRange(0, 30)
        self.spin_bola_wp_end.setValue(bola_cfg.get("wp_end", 11))
        self.spin_bola_wp_end.setPrefix("End: ")
        bola_layout.addWidget(self.spin_bola_wp_end, 0, 2)

        bola_layout.addWidget(QLabel("Trigger Dist:"), 1, 0)
        self.spin_bola_dist = QSpinBox()
        self.spin_bola_dist.setRange(0, 500)
        self.spin_bola_dist.setValue(bola_cfg.get("trigger_dist_cm", 165))
        self.spin_bola_dist.setSingleStep(10)
        self.spin_bola_dist.setSuffix(" cm")
        bola_layout.addWidget(self.spin_bola_dist, 1, 1, 1, 2)

        bola_layout.addWidget(QLabel("Avoid Angle:"), 2, 0)
        self.spin_bola_left = QSpinBox()
        self.spin_bola_left.setRange(0, 90)
        self.spin_bola_left.setValue(bola_cfg.get("avoid_angle_left", 70))
        self.spin_bola_left.setPrefix("L: ")
        self.spin_bola_left.setSuffix("°")
        bola_layout.addWidget(self.spin_bola_left, 2, 1)
        self.spin_bola_right = QSpinBox()
        self.spin_bola_right.setRange(90, 180)
        self.spin_bola_right.setValue(bola_cfg.get("avoid_angle_right", 110))
        self.spin_bola_right.setPrefix("R: ")
        self.spin_bola_right.setSuffix("°")
        bola_layout.addWidget(self.spin_bola_right, 2, 2)

        bola_layout.addWidget(QLabel("PWM Utama:"), 3, 0)
        self.slider_bola_pwm_utama = QSlider(Qt.Horizontal)
        self.slider_bola_pwm_utama.setRange(1000, 2000)
        self.slider_bola_pwm_utama.setValue(bola_cfg.get("pwm_utama", 1500))
        bola_layout.addWidget(self.slider_bola_pwm_utama, 3, 1)
        self.spin_bola_pwm_utama = QSpinBox()
        self.spin_bola_pwm_utama.setRange(1000, 2000)
        self.spin_bola_pwm_utama.setValue(bola_cfg.get("pwm_utama", 1500))
        bola_layout.addWidget(self.spin_bola_pwm_utama, 3, 2)

        bola_layout.addWidget(QLabel("PWM Depan Kiri:"), 4, 0)
        self.slider_bola_pwm_kiri = QSlider(Qt.Horizontal)
        self.slider_bola_pwm_kiri.setRange(1000, 2000)
        self.slider_bola_pwm_kiri.setValue(bola_cfg.get("pwm_depan_kiri", 1500))
        bola_layout.addWidget(self.slider_bola_pwm_kiri, 4, 1)
        self.spin_bola_pwm_kiri = QSpinBox()
        self.spin_bola_pwm_kiri.setRange(1000, 2000)
        self.spin_bola_pwm_kiri.setValue(bola_cfg.get("pwm_depan_kiri", 1500))
        bola_layout.addWidget(self.spin_bola_pwm_kiri, 4, 2)

        bola_layout.addWidget(QLabel("PWM Depan Kanan:"), 5, 0)
        self.slider_bola_pwm_kanan = QSlider(Qt.Horizontal)
        self.slider_bola_pwm_kanan.setRange(1000, 2000)
        self.slider_bola_pwm_kanan.setValue(bola_cfg.get("pwm_depan_kanan", 1500))
        bola_layout.addWidget(self.slider_bola_pwm_kanan, 5, 1)
        self.spin_bola_pwm_kanan = QSpinBox()
        self.spin_bola_pwm_kanan.setRange(1000, 2000)
        self.spin_bola_pwm_kanan.setValue(bola_cfg.get("pwm_depan_kanan", 1500))
        bola_layout.addWidget(self.spin_bola_pwm_kanan, 5, 2)

        bola_group.setLayout(bola_layout)

        # 2. --- Profil Misi Kotak (Biru-Hijau) ---
        kotak_group = QGroupBox("Misi Kotak (Biru-Hijau)")
        kotak_layout = QGridLayout()

        kotak_layout.addWidget(QLabel("WP Range:"), 0, 0)
        self.spin_kotak_wp_start = QSpinBox()
        self.spin_kotak_wp_start.setRange(0, 30)
        self.spin_kotak_wp_start.setValue(kotak_cfg.get("wp_start", 11))
        self.spin_kotak_wp_start.setPrefix("Start: ")
        kotak_layout.addWidget(self.spin_kotak_wp_start, 0, 1)
        self.spin_kotak_wp_end = QSpinBox()
        self.spin_kotak_wp_end.setRange(0, 30)
        self.spin_kotak_wp_end.setValue(kotak_cfg.get("wp_end", 14))
        self.spin_kotak_wp_end.setPrefix("End: ")
        kotak_layout.addWidget(self.spin_kotak_wp_end, 0, 2)

        kotak_layout.addWidget(QLabel("Trigger Dist:"), 1, 0)
        self.spin_kotak_dist = QSpinBox()
        self.spin_kotak_dist.setRange(0, 500)
        self.spin_kotak_dist.setValue(kotak_cfg.get("trigger_dist_cm", 165))
        self.spin_kotak_dist.setSingleStep(10)
        self.spin_kotak_dist.setSuffix(" cm")
        kotak_layout.addWidget(self.spin_kotak_dist, 1, 1, 1, 2)

        kotak_layout.addWidget(QLabel("Avoid Angle:"), 2, 0)
        self.spin_kotak_left = QSpinBox()
        self.spin_kotak_left.setRange(0, 90)
        self.spin_kotak_left.setValue(kotak_cfg.get("avoid_angle_left", 70))
        self.spin_kotak_left.setPrefix("L: ")
        self.spin_kotak_left.setSuffix("°")
        kotak_layout.addWidget(self.spin_kotak_left, 2, 1)
        self.spin_kotak_right = QSpinBox()
        self.spin_kotak_right.setRange(90, 180)
        self.spin_kotak_right.setValue(kotak_cfg.get("avoid_angle_right", 110))
        self.spin_kotak_right.setPrefix("R: ")
        self.spin_kotak_right.setSuffix("°")
        kotak_layout.addWidget(self.spin_kotak_right, 2, 2)

        kotak_layout.addWidget(QLabel("PWM Utama:"), 3, 0)
        self.slider_kotak_pwm_utama = QSlider(Qt.Horizontal)
        self.slider_kotak_pwm_utama.setRange(1000, 2000)
        self.slider_kotak_pwm_utama.setValue(kotak_cfg.get("pwm_utama", 1500))
        kotak_layout.addWidget(self.slider_kotak_pwm_utama, 3, 1)
        self.spin_kotak_pwm_utama = QSpinBox()
        self.spin_kotak_pwm_utama.setRange(1000, 2000)
        self.spin_kotak_pwm_utama.setValue(kotak_cfg.get("pwm_utama", 1500))
        kotak_layout.addWidget(self.spin_kotak_pwm_utama, 3, 2)

        kotak_layout.addWidget(QLabel("PWM Depan Kiri:"), 4, 0)
        self.slider_kotak_pwm_kiri = QSlider(Qt.Horizontal)
        self.slider_kotak_pwm_kiri.setRange(1000, 2000)
        self.slider_kotak_pwm_kiri.setValue(kotak_cfg.get("pwm_depan_kiri", 1500))
        kotak_layout.addWidget(self.slider_kotak_pwm_kiri, 4, 1)
        self.spin_kotak_pwm_kiri = QSpinBox()
        self.spin_kotak_pwm_kiri.setRange(1000, 2000)
        self.spin_kotak_pwm_kiri.setValue(kotak_cfg.get("pwm_depan_kiri", 1500))
        kotak_layout.addWidget(self.spin_kotak_pwm_kiri, 4, 2)

        kotak_layout.addWidget(QLabel("PWM Depan Kanan:"), 5, 0)
        self.slider_kotak_pwm_kanan = QSlider(Qt.Horizontal)
        self.slider_kotak_pwm_kanan.setRange(1000, 2000)
        self.slider_kotak_pwm_kanan.setValue(kotak_cfg.get("pwm_depan_kanan", 1500))
        kotak_layout.addWidget(self.slider_kotak_pwm_kanan, 5, 1)
        self.spin_kotak_pwm_kanan = QSpinBox()
        self.spin_kotak_pwm_kanan.setRange(1000, 2000)
        self.spin_kotak_pwm_kanan.setValue(kotak_cfg.get("pwm_depan_kanan", 1500))
        kotak_layout.addWidget(self.spin_kotak_pwm_kanan, 5, 2)

        kotak_group.setLayout(kotak_layout)

        # --- GABUNGKAN SEMUA KE AI LAYOUT ---
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
        self.docking_view = DockingView(config=self.config)

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
        self.docking_view.docking_config_updated.connect(
            self.docking_config_updated.emit
        )

        # --- [BARU] Koneksi Profil Misi Bola & Kotak ---
        self.slider_bola_pwm_utama.valueChanged.connect(
            self.spin_bola_pwm_utama.setValue
        )
        self.spin_bola_pwm_utama.valueChanged.connect(
            self.slider_bola_pwm_utama.setValue
        )

        self.slider_bola_pwm_kiri.valueChanged.connect(self.spin_bola_pwm_kiri.setValue)
        self.spin_bola_pwm_kiri.valueChanged.connect(self.slider_bola_pwm_kiri.setValue)

        self.slider_bola_pwm_kanan.valueChanged.connect(
            self.spin_bola_pwm_kanan.setValue
        )
        self.spin_bola_pwm_kanan.valueChanged.connect(
            self.slider_bola_pwm_kanan.setValue
        )

        self.slider_kotak_pwm_utama.valueChanged.connect(
            self.spin_kotak_pwm_utama.setValue
        )
        self.spin_kotak_pwm_utama.valueChanged.connect(
            self.slider_kotak_pwm_utama.setValue
        )

        self.slider_kotak_pwm_kiri.valueChanged.connect(
            self.spin_kotak_pwm_kiri.setValue
        )
        self.spin_kotak_pwm_kiri.valueChanged.connect(
            self.slider_kotak_pwm_kiri.setValue
        )

        self.slider_kotak_pwm_kanan.valueChanged.connect(
            self.spin_kotak_pwm_kanan.setValue
        )
        self.spin_kotak_pwm_kanan.valueChanged.connect(
            self.slider_kotak_pwm_kanan.setValue
        )

        mission_spinboxes = [
            self.spin_bola_wp_start,
            self.spin_bola_wp_end,
            self.spin_bola_dist,
            self.spin_bola_left,
            self.spin_bola_right,
            self.spin_bola_pwm_utama,
            self.spin_bola_pwm_kiri,
            self.spin_bola_pwm_kanan,
            self.spin_kotak_wp_start,
            self.spin_kotak_wp_end,
            self.spin_kotak_dist,
            self.spin_kotak_left,
            self.spin_kotak_right,
            self.spin_kotak_pwm_utama,
            self.spin_kotak_pwm_kiri,
            self.spin_kotak_pwm_kanan,
        ]
        for sb in mission_spinboxes:
            sb.valueChanged.connect(self._on_mission_profile_changed)

    # --- SLOT HANDLERS ---
    def _on_mission_profile_changed(self):
        """Kumpulkan semua data profil misi dan kirim ke backend."""
        payload = {
            "vision_ball_red_green": {
                "wp_start": self.spin_bola_wp_start.value(),
                "wp_end": self.spin_bola_wp_end.value(),
                "trigger_dist_cm": self.spin_bola_dist.value(),
                "avoid_angle_left": self.spin_bola_left.value(),
                "avoid_angle_right": self.spin_bola_right.value(),
                "pwm_utama": self.spin_bola_pwm_utama.value(),
                "pwm_depan_kiri": self.spin_bola_pwm_kiri.value(),
                "pwm_depan_kanan": self.spin_bola_pwm_kanan.value(),
            },
            "vision_box_blue_green": {
                "wp_start": self.spin_kotak_wp_start.value(),
                "wp_end": self.spin_kotak_wp_end.value(),
                "trigger_dist_cm": self.spin_kotak_dist.value(),
                "avoid_angle_left": self.spin_kotak_left.value(),
                "avoid_angle_right": self.spin_kotak_right.value(),
                "pwm_utama": self.spin_kotak_pwm_utama.value(),
                "pwm_depan_kiri": self.spin_kotak_pwm_kiri.value(),
                "pwm_depan_kanan": self.spin_kotak_pwm_kanan.value(),
            },
        }
        self.vision_mission_updated.emit(payload)
