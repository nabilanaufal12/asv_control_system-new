# src/navantara_gui/components/settings_panel.py
from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QLabel,
    QSlider,
    QHBoxLayout,
    QFormLayout,
    QSpinBox,
    QComboBox,
)
from PySide6.QtCore import Signal, Qt


class SettingsPanel(QGroupBox):
    """
    Panel pengaturan AI Vision.
    """

    vision_speed_updated = Signal(int)
    vision_front_motor_updated = Signal(
        dict
    )  # [UBAH] Sekarang mengirim dict (Kiri & Kanan)
    vision_servo_updated = Signal(dict)
    vision_distance_updated = Signal(float)
    vision_model_updated = Signal(str)
    vision_wp_ranges_updated = Signal(dict)

    def __init__(self, config, title="System Configuration"):
        super().__init__(title)
        self.config = config
        main_layout = QVBoxLayout()

        # --- [BAGIAN KONTROL AI VISION & MISI] ---
        ai_control_group = QGroupBox("AI Engine & Thruster Parameters")
        ai_layout = QFormLayout()

        # 1. Slider Kecepatan Motor Bawah (Utama)
        speed_widget_layout = QHBoxLayout()
        self.spin_ai_speed = QSpinBox()
        self.spin_ai_speed.setRange(1000, 2000)
        self.spin_ai_speed.setValue(1300)
        self.slider_ai_speed = QSlider(Qt.Horizontal)
        self.slider_ai_speed.setRange(1000, 2000)
        self.slider_ai_speed.setValue(1300)
        speed_widget_layout.addWidget(self.slider_ai_speed)
        speed_widget_layout.addWidget(self.spin_ai_speed)
        ai_layout.addRow("PWM Motor Utama:", speed_widget_layout)

        # --- Motor Depan Kiri ---
        left_front_widget_layout = QHBoxLayout()
        self.spin_front_left = QSpinBox()
        self.spin_front_left.setRange(1000, 2000)
        self.spin_front_left.setValue(1500)
        self.slider_front_left = QSlider(Qt.Horizontal)
        self.slider_front_left.setRange(1000, 2000)
        self.slider_front_left.setValue(1500)
        left_front_widget_layout.addWidget(self.slider_front_left)
        left_front_widget_layout.addWidget(self.spin_front_left)
        ai_layout.addRow("PWM Depan Kiri:", left_front_widget_layout)

        # --- Motor Depan Kanan ---
        right_front_widget_layout = QHBoxLayout()
        self.spin_front_right = QSpinBox()
        self.spin_front_right.setRange(1000, 2000)
        self.spin_front_right.setValue(1500)
        self.slider_front_right = QSlider(Qt.Horizontal)
        self.slider_front_right.setRange(1000, 2000)
        self.slider_front_right.setValue(1500)
        right_front_widget_layout.addWidget(self.slider_front_right)
        right_front_widget_layout.addWidget(self.spin_front_right)
        ai_layout.addRow("PWM Depan Kanan:", right_front_widget_layout)

        # 2. Input Servo Kiri & Kanan
        servo_widget_layout = QHBoxLayout()
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

        servo_widget_layout.addWidget(self.spin_left)
        servo_widget_layout.addWidget(self.spin_right)
        ai_layout.addRow("Avoidance Angle:", servo_widget_layout)

        # 3. Input Jarak Obstacle (cm)
        self.spin_obs_dist = QSpinBox()
        self.spin_obs_dist.setRange(0, 500)
        self.spin_obs_dist.setValue(165)
        self.spin_obs_dist.setSingleStep(10)
        self.spin_obs_dist.setPrefix("Trigger Dist: ")
        self.spin_obs_dist.setSuffix(" cm")
        ai_layout.addRow("AI Activation:", self.spin_obs_dist)

        # 4. Pilihan Model AI
        self.combo_model = QComboBox()
        self.combo_model.addItems(
            ["Auto", "best.engine", "best100.engine", "best.pt", "best100.pt"]
        )
        ai_layout.addRow("AI Model:", self.combo_model)

        ai_control_group.setLayout(ai_layout)
        main_layout.addWidget(ai_control_group)

        # --- [BAGIAN WP RANGES] ---
        wp_ranges_group = QGroupBox("Vision Waypoint Ranges")
        wp_ranges_layout = QFormLayout()

        # Bola (Avoidance)
        bola_layout = QHBoxLayout()
        self.spin_wp_bola_start = QSpinBox()
        self.spin_wp_bola_start.setRange(0, 100)
        self.spin_wp_bola_end = QSpinBox()
        self.spin_wp_bola_end.setRange(0, 100)
        bola_layout.addWidget(QLabel("Start:"))
        bola_layout.addWidget(self.spin_wp_bola_start)
        bola_layout.addWidget(QLabel("End:"))
        bola_layout.addWidget(self.spin_wp_bola_end)
        wp_ranges_layout.addRow("Avoidance (Bola):", bola_layout)

        # Kotak Biru (Underwater)
        kotak_biru_layout = QHBoxLayout()
        self.spin_wp_biru_start = QSpinBox()
        self.spin_wp_biru_start.setRange(0, 100)
        self.spin_wp_biru_end = QSpinBox()
        self.spin_wp_biru_end.setRange(0, 100)
        kotak_biru_layout.addWidget(QLabel("Start:"))
        kotak_biru_layout.addWidget(self.spin_wp_biru_start)
        kotak_biru_layout.addWidget(QLabel("End:"))
        kotak_biru_layout.addWidget(self.spin_wp_biru_end)
        wp_ranges_layout.addRow("Track UW (K. Biru):", kotak_biru_layout)

        # Kotak Hijau (Surface)
        kotak_hijau_layout = QHBoxLayout()
        self.spin_wp_hijau_start = QSpinBox()
        self.spin_wp_hijau_start.setRange(0, 100)
        self.spin_wp_hijau_end = QSpinBox()
        self.spin_wp_hijau_end.setRange(0, 100)
        kotak_hijau_layout.addWidget(QLabel("Start:"))
        kotak_hijau_layout.addWidget(self.spin_wp_hijau_start)
        kotak_hijau_layout.addWidget(QLabel("End:"))
        kotak_hijau_layout.addWidget(self.spin_wp_hijau_end)
        wp_ranges_layout.addRow("Track Sfc (K. Hijau):", kotak_hijau_layout)

        wp_ranges_group.setLayout(wp_ranges_layout)
        main_layout.addWidget(wp_ranges_group)

        # Set default values from config if available
        self._load_wp_ranges_from_config()

        self.setLayout(main_layout)

        # --- KONEKSI SINYAL ---
        # Sinkronisasi SpinBox dan Slider
        self.slider_ai_speed.valueChanged.connect(self.spin_ai_speed.setValue)
        self.spin_ai_speed.valueChanged.connect(self.slider_ai_speed.setValue)

        self.slider_front_left.valueChanged.connect(self.spin_front_left.setValue)
        self.spin_front_left.valueChanged.connect(self.slider_front_left.setValue)

        self.slider_front_right.valueChanged.connect(self.spin_front_right.setValue)
        self.spin_front_right.valueChanged.connect(self.slider_front_right.setValue)

        # Koneksi Kontrol AI
        self.slider_ai_speed.valueChanged.connect(self._on_ai_speed_changed)

        # [UBAH] Sambungkan kedua slider depan ke fungsi yang sama
        self.slider_front_left.valueChanged.connect(self._on_front_speed_changed)
        self.slider_front_right.valueChanged.connect(self._on_front_speed_changed)

        self.spin_left.valueChanged.connect(self._on_ai_servo_changed)
        self.spin_right.valueChanged.connect(self._on_ai_servo_changed)
        self.spin_obs_dist.valueChanged.connect(self._on_obs_dist_changed)
        self.combo_model.currentTextChanged.connect(self._on_model_changed)

        # Koneksi wp ranges
        self.spin_wp_bola_start.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_bola_end.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_biru_start.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_biru_end.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_hijau_start.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_hijau_end.valueChanged.connect(self._on_wp_ranges_changed)

    def _load_wp_ranges_from_config(self):
        vision_cfg = self.config.get("vision", {})
        rb = vision_cfg.get("wp_range_bola", [0, 10])
        rbb = vision_cfg.get("wp_range_kotak_biru", [11, 12])
        rbh = vision_cfg.get("wp_range_kotak_hijau", [13, 14])

        # Supaya tidak men-trigger event terus menerus saat set
        self.spin_wp_bola_start.blockSignals(True)
        self.spin_wp_bola_end.blockSignals(True)
        self.spin_wp_biru_start.blockSignals(True)
        self.spin_wp_biru_end.blockSignals(True)
        self.spin_wp_hijau_start.blockSignals(True)
        self.spin_wp_hijau_end.blockSignals(True)

        self.spin_wp_bola_start.setValue(rb[0])
        self.spin_wp_bola_end.setValue(rb[1])
        self.spin_wp_biru_start.setValue(rbb[0])
        self.spin_wp_biru_end.setValue(rbb[1])
        self.spin_wp_hijau_start.setValue(rbh[0])
        self.spin_wp_hijau_end.setValue(rbh[1])

        self.spin_wp_bola_start.blockSignals(False)
        self.spin_wp_bola_end.blockSignals(False)
        self.spin_wp_biru_start.blockSignals(False)
        self.spin_wp_biru_end.blockSignals(False)
        self.spin_wp_hijau_start.blockSignals(False)
        self.spin_wp_hijau_end.blockSignals(False)

    # --- SLOT ACTIONS ---
    def _on_model_changed(self, text):
        self.vision_model_updated.emit(text)

    # --- SLOT HANDLERS ---
    def _on_ai_speed_changed(self, value):
        self.vision_speed_updated.emit(value)

    def _on_front_speed_changed(self):
        val_l = self.slider_front_left.value()
        val_r = self.slider_front_right.value()
        # Kirim payload data ke backend
        self.vision_front_motor_updated.emit({"left": val_l, "right": val_r})

    def _on_ai_servo_changed(self):
        payload = {"left": self.spin_left.value(), "right": self.spin_right.value()}
        self.vision_servo_updated.emit(payload)

    def _on_obs_dist_changed(self, value):
        self.vision_distance_updated.emit(float(value))

    def _on_wp_ranges_changed(self):
        payload = {
            "wp_range_bola": [
                self.spin_wp_bola_start.value(),
                self.spin_wp_bola_end.value(),
            ],
            "wp_range_kotak_biru": [
                self.spin_wp_biru_start.value(),
                self.spin_wp_biru_end.value(),
            ],
            "wp_range_kotak_hijau": [
                self.spin_wp_hijau_start.value(),
                self.spin_wp_hijau_end.value(),
            ],
        }
        self.vision_wp_ranges_updated.emit(payload)
