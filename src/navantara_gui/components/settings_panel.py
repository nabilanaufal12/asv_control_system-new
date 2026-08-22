# src/navantara_gui/components/settings_panel.py
from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QLabel,
    QHBoxLayout,
    QGridLayout,
    QComboBox,
    QPushButton,
    QMessageBox,
    QSpinBox,
    QDoubleSpinBox,
    QCheckBox,
)
from PySide6.QtCore import Signal, Qt
import os
import json


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

    # [TAMBAHAN BARU] Sinyal untuk Photo Mission Segments
    send_photo_mission = Signal(dict)
    send_portrait_config = Signal(dict)
    send_dock_config = Signal(dict)
    send_dock_enabled = Signal(dict)
    send_box_avoidance_config = Signal(dict)

    def __init__(self, config, title="System Configuration"):
        super().__init__(title)
        self.config = config
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(6, 8, 6, 8)
        main_layout.setSpacing(8)

        # Helper untuk styling QGridLayout agar konsisten di semua group
        def setup_grid_layout():
            grid = QGridLayout()
            grid.setHorizontalSpacing(8)
            grid.setVerticalSpacing(6)
            grid.setContentsMargins(8, 10, 8, 8)
            grid.setColumnStretch(0, 0)
            grid.setColumnStretch(1, 1)
            grid.setColumnStretch(2, 0)
            grid.setColumnStretch(3, 1)
            return grid

        # =====================================================================
        # 1. AI Engine & Thruster Parameters (Misi Bola Merah - Hijau)
        # =====================================================================
        ai_control_group = QGroupBox("AI Engine Thruster Parameters")
        ai_grid = setup_grid_layout()

        self.spin_ai_speed = QSpinBox()
        self.spin_ai_speed.setRange(1000, 2000)
        self.spin_ai_speed.setValue(1500)

        self.spin_front = QSpinBox()
        self.spin_front.setRange(1000, 2000)
        self.spin_front.setValue(1800)

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

        self.spin_obs_dist = QSpinBox()
        self.spin_obs_dist.setRange(0, 500)
        self.spin_obs_dist.setValue(165)
        self.spin_obs_dist.setSingleStep(10)
        self.spin_obs_dist.setSuffix(" cm")

        self.combo_model = QComboBox()
        self.combo_model.addItems(
            ["Auto", "best.engine", "best100.engine", "best.pt", "best100.pt"]
        )

        ai_grid.addWidget(QLabel("Speed (PWM):"), 0, 0)
        ai_grid.addWidget(self.spin_ai_speed, 0, 1)
        ai_grid.addWidget(QLabel("Front Motor:"), 0, 2)
        ai_grid.addWidget(self.spin_front, 0, 3)

        ai_grid.addWidget(QLabel("Avoid Angle:"), 1, 0)
        ai_grid.addWidget(self.spin_left, 1, 1)
        ai_grid.addWidget(QLabel("Avoid Angle:"), 1, 2)
        ai_grid.addWidget(self.spin_right, 1, 3)

        ai_grid.addWidget(QLabel("Obstacle Dist:"), 2, 0)
        ai_grid.addWidget(self.spin_obs_dist, 2, 1)
        ai_grid.addWidget(QLabel("Model:"), 2, 2)
        ai_grid.addWidget(self.combo_model, 2, 3)

        ai_control_group.setLayout(ai_grid)
        main_layout.addWidget(ai_control_group)

        # =====================================================================
        # 2. Box Detection Settings (Misi Kotak Biru - Hijau)
        # =====================================================================
        box_avoid_group = QGroupBox("Box Detection Settings")
        box_grid = setup_grid_layout()

        self.spin_box_speed = QSpinBox()
        self.spin_box_speed.setRange(1000, 2000)
        self.spin_box_speed.setValue(1500)

        self.spin_box_front = QSpinBox()
        self.spin_box_front.setRange(1000, 2000)
        self.spin_box_front.setValue(1800)

        self.spin_box_left = QSpinBox()
        self.spin_box_left.setRange(0, 90)
        self.spin_box_left.setValue(70)
        self.spin_box_left.setPrefix("Left: ")
        self.spin_box_left.setSuffix("°")

        self.spin_box_right = QSpinBox()
        self.spin_box_right.setRange(90, 180)
        self.spin_box_right.setValue(110)
        self.spin_box_right.setPrefix("Right: ")
        self.spin_box_right.setSuffix("°")

        self.spin_box_dist = QSpinBox()
        self.spin_box_dist.setRange(0, 500)
        self.spin_box_dist.setValue(165)
        self.spin_box_dist.setSuffix(" cm")

        box_grid.addWidget(QLabel("Speed (PWM):"), 0, 0)
        box_grid.addWidget(self.spin_box_speed, 0, 1)
        box_grid.addWidget(QLabel("Front Motor:"), 0, 2)
        box_grid.addWidget(self.spin_box_front, 0, 3)

        box_grid.addWidget(QLabel("Avoid Angle:"), 1, 0)
        box_grid.addWidget(self.spin_box_left, 1, 1)
        box_grid.addWidget(QLabel("Avoid Angle:"), 1, 2)
        box_grid.addWidget(self.spin_box_right, 1, 3)

        box_grid.addWidget(QLabel("Obstacle Dist:"), 2, 0)
        box_grid.addWidget(self.spin_box_dist, 2, 1)

        box_avoid_group.setLayout(box_grid)
        main_layout.addWidget(box_avoid_group)

        # =====================================================================
        # 3. Vision Target WP Ranges
        # =====================================================================
        wp_ranges_group = QGroupBox("Vision Target WP Ranges")
        wp_grid = setup_grid_layout()

        self.spin_wp_bola_start = QSpinBox()
        self.spin_wp_bola_start.setRange(0, 100)
        self.spin_wp_bola_start.setPrefix("Start: ")

        self.spin_wp_bola_end = QSpinBox()
        self.spin_wp_bola_end.setRange(0, 100)
        self.spin_wp_bola_end.setPrefix("End: ")

        self.spin_wp_biru_start = QSpinBox()
        self.spin_wp_biru_start.setRange(0, 100)
        self.spin_wp_biru_start.setPrefix("Start: ")

        self.spin_wp_biru_end = QSpinBox()
        self.spin_wp_biru_end.setRange(0, 100)
        self.spin_wp_biru_end.setPrefix("End: ")

        self.spin_wp_hijau_start = QSpinBox()
        self.spin_wp_hijau_start.setRange(0, 100)
        self.spin_wp_hijau_start.setPrefix("Start: ")

        self.spin_wp_hijau_end = QSpinBox()
        self.spin_wp_hijau_end.setRange(0, 100)
        self.spin_wp_hijau_end.setPrefix("End: ")

        wp_grid.addWidget(QLabel("Bola Avoid:"), 0, 0)
        wp_grid.addWidget(self.spin_wp_bola_start, 0, 1)
        wp_grid.addWidget(QLabel("—"), 0, 2, alignment=Qt.AlignCenter)
        wp_grid.addWidget(self.spin_wp_bola_end, 0, 3)

        wp_grid.addWidget(QLabel("Track UW (Biru):"), 1, 0)
        wp_grid.addWidget(self.spin_wp_biru_start, 1, 1)
        wp_grid.addWidget(QLabel("—"), 1, 2, alignment=Qt.AlignCenter)
        wp_grid.addWidget(self.spin_wp_biru_end, 1, 3)

        wp_grid.addWidget(QLabel("Track Sfc (Hijau):"), 2, 0)
        wp_grid.addWidget(self.spin_wp_hijau_start, 2, 1)
        wp_grid.addWidget(QLabel("—"), 2, 2, alignment=Qt.AlignCenter)
        wp_grid.addWidget(self.spin_wp_hijau_end, 2, 3)

        wp_ranges_group.setLayout(wp_grid)
        main_layout.addWidget(wp_ranges_group)

        # =====================================================================
        # 4. Photo Mission Settings
        # =====================================================================
        photo_mission_box = QGroupBox("Photo Mission Settings")
        photo_grid = setup_grid_layout()

        self.surf_wp1_input = QSpinBox()
        self.surf_wp1_input.setRange(0, 100)
        self.surf_wp1_input.setValue(13)
        self.surf_wp1_input.setPrefix("Start: ")

        self.surf_wp2_input = QSpinBox()
        self.surf_wp2_input.setRange(0, 100)
        self.surf_wp2_input.setValue(14)
        self.surf_wp2_input.setPrefix("End: ")

        self.under_wp1_input = QSpinBox()
        self.under_wp1_input.setRange(0, 100)
        self.under_wp1_input.setValue(11)
        self.under_wp1_input.setPrefix("Start: ")

        self.under_wp2_input = QSpinBox()
        self.under_wp2_input.setRange(0, 100)
        self.under_wp2_input.setValue(12)
        self.under_wp2_input.setPrefix("End: ")

        self.photo_count_input = QSpinBox()
        self.photo_count_input.setRange(1, 100)
        self.photo_count_input.setValue(10)

        self.photo_interval_input = QDoubleSpinBox()
        self.photo_interval_input.setRange(0.5, 10.0)
        self.photo_interval_input.setSingleStep(0.1)
        self.photo_interval_input.setValue(0.8)
        self.photo_interval_input.setSuffix(" s")

        self.spin_portrait_speed = QSpinBox()
        self.spin_portrait_speed.setRange(1000, 2000)
        self.spin_portrait_speed.setValue(1600)

        self.spin_portrait_rev_speed = QSpinBox()
        self.spin_portrait_rev_speed.setRange(1000, 2000)
        self.spin_portrait_rev_speed.setValue(1400)

        self.spin_portrait_stop = QSpinBox()
        self.spin_portrait_stop.setRange(1, 15)
        self.spin_portrait_stop.setValue(3)
        self.spin_portrait_stop.setSuffix(" s")

        self.spin_portrait_reverse = QSpinBox()
        self.spin_portrait_reverse.setRange(1, 15)
        self.spin_portrait_reverse.setValue(2)
        self.spin_portrait_reverse.setSuffix(" s")

        # Row 0: Surface WP
        photo_grid.addWidget(QLabel("Surface WP:"), 0, 0)
        photo_grid.addWidget(self.surf_wp1_input, 0, 1)
        photo_grid.addWidget(QLabel("—"), 0, 2, alignment=Qt.AlignCenter)
        photo_grid.addWidget(self.surf_wp2_input, 0, 3)

        # Row 1: Underwater WP
        photo_grid.addWidget(QLabel("Underwater WP:"), 1, 0)
        photo_grid.addWidget(self.under_wp1_input, 1, 1)
        photo_grid.addWidget(QLabel("—"), 1, 2, alignment=Qt.AlignCenter)
        photo_grid.addWidget(self.under_wp2_input, 1, 3)

        # Row 2: Photo Config
        photo_grid.addWidget(QLabel("Max Foto:"), 2, 0)
        photo_grid.addWidget(self.photo_count_input, 2, 1)
        photo_grid.addWidget(QLabel("Interval:"), 2, 2)
        photo_grid.addWidget(self.photo_interval_input, 2, 3)

        # Row 3: Speed (Portrait & Reverse)
        photo_grid.addWidget(QLabel("Speed Portrait:"), 3, 0)
        photo_grid.addWidget(self.spin_portrait_speed, 3, 1)
        photo_grid.addWidget(QLabel("Speed Reverse:"), 3, 2)
        photo_grid.addWidget(self.spin_portrait_rev_speed, 3, 3)

        # Row 4: Duration (Stop & Reverse)
        photo_grid.addWidget(QLabel("Durasi Stop:"), 4, 0)
        photo_grid.addWidget(self.spin_portrait_stop, 4, 1)
        photo_grid.addWidget(QLabel("Durasi Rev:"), 4, 2)
        photo_grid.addWidget(self.spin_portrait_reverse, 4, 3)

        photo_mission_box.setLayout(photo_grid)
        main_layout.addWidget(photo_mission_box)

        # =====================================================================
        # 5. Docking Settings
        # =====================================================================
        docking_mission_box = QGroupBox("Docking Settings")
        dock_grid = setup_grid_layout()

        self.chk_dock_enable = QCheckBox("Enable Docking Mission")
        self.chk_dock_enable.setChecked(True)

        self.spin_dock_motor = QSpinBox()
        self.spin_dock_motor.setRange(1000, 2000)
        self.spin_dock_motor.setValue(
            self.config.get("docking_defaults", {}).get("motor_utama_pwm", 1200)
        )

        self.spin_dock_front = QSpinBox()
        self.spin_dock_front.setRange(1000, 2000)
        self.spin_dock_front.setValue(
            self.config.get("docking_defaults", {}).get("motor_depan_pwm", 1400)
        )

        self.spin_dock_left = QSpinBox()
        self.spin_dock_left.setRange(0, 90)
        self.spin_dock_left.setValue(
            self.config.get("docking_defaults", {}).get("servo_left", 0)
        )
        self.spin_dock_left.setPrefix("Left: ")
        self.spin_dock_left.setSuffix("°")

        self.spin_dock_right = QSpinBox()
        self.spin_dock_right.setRange(90, 180)
        self.spin_dock_right.setValue(
            self.config.get("docking_defaults", {}).get("servo_right", 180)
        )
        self.spin_dock_right.setPrefix("Right: ")
        self.spin_dock_right.setSuffix("°")

        self.spin_dock_charge = QSpinBox()
        self.spin_dock_charge.setRange(1, 15)
        self.spin_dock_charge.setValue(
            self.config.get("docking_defaults", {}).get("charge_duration_ms", 3000)
            // 1000
        )
        self.spin_dock_charge.setSuffix(" s")

        dock_grid.addWidget(self.chk_dock_enable, 0, 0, 1, 4)

        dock_grid.addWidget(QLabel("Speed (PWM):"), 1, 0)
        dock_grid.addWidget(self.spin_dock_motor, 1, 1)
        dock_grid.addWidget(QLabel("Front Motor:"), 1, 2)
        dock_grid.addWidget(self.spin_dock_front, 1, 3)

        dock_grid.addWidget(QLabel("Swing Angle:"), 2, 0)
        dock_grid.addWidget(self.spin_dock_left, 2, 1)
        dock_grid.addWidget(QLabel("Swing Angle:"), 2, 2)
        dock_grid.addWidget(self.spin_dock_right, 2, 3)

        dock_grid.addWidget(QLabel("Durasi Charge:"), 3, 0)
        dock_grid.addWidget(self.spin_dock_charge, 3, 1)

        docking_mission_box.setLayout(dock_grid)
        main_layout.addWidget(docking_mission_box)

        # --- Tombol Simpan Default ---
        self.save_default_button = QPushButton("Save as Default Config")
        self.save_default_button.setStyleSheet(
            "background-color: #4CAF50; color: white; font-weight: bold; margin-top: 6px; padding: 6px;"
        )
        main_layout.addWidget(self.save_default_button)

        # --------------------------------------------------------------------

        # Set default values from config if available
        self._load_defaults_from_config()

        # [MODIFIKASI] Sembunyikan panah atas-bawah (buttons) di semua QSpinBox agar polos seperti QLineEdit
        for spin_box in self.findChildren(QSpinBox):
            spin_box.setButtonSymbols(QSpinBox.NoButtons)

        self.setLayout(main_layout)

        # --- KONEKSI SINYAL ---
        # Event Handler AI Engine
        self.spin_ai_speed.valueChanged.connect(self._on_ai_speed_changed)
        self.spin_front.valueChanged.connect(self._on_front_speed_changed)
        self.spin_left.valueChanged.connect(self._on_ai_servo_changed)
        self.spin_right.valueChanged.connect(self._on_ai_servo_changed)
        self.spin_obs_dist.valueChanged.connect(self._on_obs_dist_changed)
        self.combo_model.currentTextChanged.connect(self._on_model_changed)
        
        # Event Handler Box Avoidance Settings
        self.spin_box_dist.valueChanged.connect(self._on_box_avoidance_changed)
        self.spin_box_left.valueChanged.connect(self._on_box_avoidance_changed)
        self.spin_box_right.valueChanged.connect(self._on_box_avoidance_changed)
        self.spin_box_speed.valueChanged.connect(self._on_box_avoidance_changed)
        self.spin_box_front.valueChanged.connect(self._on_box_avoidance_changed)

        # Event Handler Photo Mission Segments (Auto Update)
        self.surf_wp1_input.valueChanged.connect(self._on_set_photo_mission)
        self.surf_wp2_input.valueChanged.connect(self._on_set_photo_mission)
        self.under_wp1_input.valueChanged.connect(self._on_set_photo_mission)
        self.under_wp2_input.valueChanged.connect(self._on_set_photo_mission)
        self.photo_count_input.valueChanged.connect(self._on_set_photo_mission)
        self.photo_interval_input.valueChanged.connect(self._on_set_photo_mission)

        self.spin_portrait_speed.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_rev_speed.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_stop.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_reverse.valueChanged.connect(self._on_set_photo_mission)

        self.spin_dock_motor.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_front.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_left.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_right.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_charge.valueChanged.connect(self._on_set_dock_config)
        self.chk_dock_enable.stateChanged.connect(self._on_dock_enable_changed)

        self.save_default_button.clicked.connect(self._save_defaults_to_config)

        # Event Handler Update Ranges
        self.spin_wp_bola_start.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_bola_end.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_biru_start.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_biru_end.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_hijau_start.valueChanged.connect(self._on_wp_ranges_changed)
        self.spin_wp_hijau_end.valueChanged.connect(self._on_wp_ranges_changed)

    def _load_defaults_from_config(self):
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

        # Load GUI Defaults jika ada
        gui_cfg = self.config.get("gui_settings", {})
        defs = gui_cfg.get("panel_defaults", {})

        if "ai_speed" in defs:
            self.spin_ai_speed.setValue(defs["ai_speed"])
        if "dock_motor" in defs:
            self.spin_dock_motor.setValue(defs["dock_motor"])
        if "dock_front" in defs:
            self.spin_dock_front.setValue(defs["dock_front"])
        if "dock_left" in defs:
            self.spin_dock_left.setValue(defs["dock_left"])
        if "dock_right" in defs:
            self.spin_dock_right.setValue(defs["dock_right"])
        if "dock_charge" in defs:
            self.spin_dock_charge.setValue(defs["dock_charge"])
        if "front_motor" in defs:
            self.spin_front.setValue(defs["front_motor"])
        if "servo_left" in defs:
            self.spin_left.setValue(defs["servo_left"])
        if "servo_right" in defs:
            self.spin_right.setValue(defs["servo_right"])
        if "obs_dist" in defs:
            self.spin_obs_dist.setValue(defs["obs_dist"])
            
        if "box_obs_dist" in defs:
            self.spin_box_dist.setValue(defs["box_obs_dist"])
        if "box_servo_left" in defs:
            self.spin_box_left.setValue(defs["box_servo_left"])
        if "box_servo_right" in defs:
            self.spin_box_right.setValue(defs["box_servo_right"])
        if "box_speed" in defs:
            self.spin_box_speed.setValue(defs["box_speed"])
        if "box_front_motor" in defs:
            self.spin_box_front.setValue(defs["box_front_motor"])
        elif "box_pwm" in defs:
            self.spin_box_front.setValue(defs["box_pwm"])

        if "photo_surf1" in defs:
            self.surf_wp1_input.setValue(int(defs["photo_surf1"]))
        if "photo_surf2" in defs:
            self.surf_wp2_input.setValue(int(defs["photo_surf2"]))
        if "photo_under1" in defs:
            self.under_wp1_input.setValue(int(defs["photo_under1"]))
        if "photo_under2" in defs:
            self.under_wp2_input.setValue(int(defs["photo_under2"]))
        if "photo_count" in defs:
            self.photo_count_input.setValue(int(defs["photo_count"]))
            self.photo_interval_input.setValue(float(defs.get("photo_interval", 2.0)))

        if "portrait_speed" in defs:
            self.spin_portrait_speed.setValue(defs["portrait_speed"])
        if "portrait_rev_speed" in defs:
            self.spin_portrait_rev_speed.setValue(defs["portrait_rev_speed"])
        if "portrait_stop" in defs:
            self.spin_portrait_stop.setValue(defs["portrait_stop"])
        if "portrait_reverse" in defs:
            self.spin_portrait_reverse.setValue(defs["portrait_reverse"])

    def _save_defaults_to_config(self):
        # Update config dict
        if "vision" not in self.config:
            self.config["vision"] = {}
        self.config["vision"]["wp_range_bola"] = [
            self.spin_wp_bola_start.value(),
            self.spin_wp_bola_end.value(),
        ]
        self.config["vision"]["wp_range_kotak_biru"] = [
            self.spin_wp_biru_start.value(),
            self.spin_wp_biru_end.value(),
        ]
        self.config["vision"]["wp_range_kotak_hijau"] = [
            self.spin_wp_hijau_start.value(),
            self.spin_wp_hijau_end.value(),
        ]

        if "gui_settings" not in self.config:
            self.config["gui_settings"] = {}

        self.config["gui_settings"]["panel_defaults"] = {
            "ai_speed": self.spin_ai_speed.value(),
            "front_motor": self.spin_front.value(),
            "servo_left": self.spin_left.value(),
            "servo_right": self.spin_right.value(),
            "obs_dist": self.spin_obs_dist.value(),
            "box_obs_dist": self.spin_box_dist.value(),
            "box_servo_left": self.spin_box_left.value(),
            "box_servo_right": self.spin_box_right.value(),
            "box_speed": self.spin_box_speed.value(),
            "box_front_motor": self.spin_box_front.value(),
            "box_pwm": self.spin_box_front.value(),
            "photo_surf1": self.surf_wp1_input.value(),
            "photo_surf2": self.surf_wp2_input.value(),
            "photo_under1": self.under_wp1_input.value(),
            "photo_under2": self.under_wp2_input.value(),
            "photo_count": self.photo_count_input.value(),
            "photo_interval": self.photo_interval_input.value(),
            "portrait_speed": self.spin_portrait_speed.value(),
            "portrait_rev_speed": self.spin_portrait_rev_speed.value(),
            "portrait_stop": self.spin_portrait_stop.value(),
            "portrait_reverse": self.spin_portrait_reverse.value(),
            "dock_motor": self.spin_dock_motor.value(),
            "dock_front": self.spin_dock_front.value(),
            "dock_left": self.spin_dock_left.value(),
            "dock_right": self.spin_dock_right.value(),
            "dock_charge": self.spin_dock_charge.value(),
        }

        if "docking_defaults" not in self.config:
            self.config["docking_defaults"] = {}
        self.config["docking_defaults"][
            "motor_utama_pwm"
        ] = self.spin_dock_motor.value()
        self.config["docking_defaults"][
            "motor_depan_pwm"
        ] = self.spin_dock_front.value()
        self.config["docking_defaults"][
            "servo_left"
        ] = self.spin_dock_left.value()
        self.config["docking_defaults"][
            "servo_right"
        ] = self.spin_dock_right.value()
        self.config["docking_defaults"]["charge_duration_ms"] = (
            self.spin_dock_charge.value() * 1000
        )

        # Write to file
        try:
            gui_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(gui_dir)))
            config_path = os.path.join(project_root, "config", "config.json")
            with open(config_path, "w") as f:
                json.dump(self.config, f, indent=2)
            print("[SettingsPanel] Konfigurasi berhasil disimpan secara permanen!")
            QMessageBox.information(
                self, "Berhasil", "Pengaturan default berhasil disimpan!"
            )
        except Exception as e:
            print(f"[SettingsPanel] Gagal menyimpan config: {e}")
            QMessageBox.critical(self, "Error", f"Gagal menyimpan: {e}")

    def broadcast_all_settings(self):
        """Kirim semua setting ke backend secara serentak (berguna saat baru connect)."""
        print("[SettingsPanel] Melakukan broadcast semua pengaturan ke backend...")
        self._on_model_changed(self.combo_model.currentText())
        self._on_ai_speed_changed(self.spin_ai_speed.value())
        self._on_front_speed_changed()
        self._on_ai_servo_changed()
        self._on_obs_dist_changed(self.spin_obs_dist.value())
        self._on_wp_ranges_changed()
        self._on_set_photo_mission()
        self._on_set_dock_config()

    # --- SLOT ACTIONS ---
    def _on_model_changed(self, text):
        self.vision_model_updated.emit(text)

    # --- SLOT HANDLERS ---
    def _on_ai_speed_changed(self, value):
        self.vision_speed_updated.emit(value)

    def _on_front_speed_changed(self):
        val = self.spin_front.value()
        # Kirim payload data ke backend
        self.vision_front_motor_updated.emit({"pwm": val})

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
        # Payload sudah sesuai format yang diminta asv_handler (UPDATE_VISION_WP_RANGES)
        self.vision_wp_ranges_updated.emit(payload)

    def _on_set_photo_mission(self):
        """Handler untuk set misi foto segmen."""
        try:
            surf1 = self.surf_wp1_input.value()
            surf2 = self.surf_wp2_input.value()
            under1 = self.under_wp1_input.value()
            under2 = self.under_wp2_input.value()
            count = self.photo_count_input.value()

            if count <= 0:
                print("[SettingsPanel] Error: Jumlah Foto harus > 0.")
                return

            payload = {
                "surf_wp1": surf1,
                "surf_wp2": surf2,
                "under_wp1": under1,
                "under_wp2": under2,
                "count": count,
            }
            self.send_photo_mission.emit(payload)
            print(f"[SettingsPanel] Sinyal send_photo_mission dipancarkan: {payload}")

            # Kirim juga konfigurasi portrait
            portrait_payload = {
                "speed": self.spin_portrait_speed.value(),
                "stop_ms": self.spin_portrait_stop.value() * 1000,  # detik -> ms
                "reverse_ms": self.spin_portrait_reverse.value() * 1000,  # detik -> ms
                "reverse_speed": self.spin_portrait_rev_speed.value(),
            }
            self.send_portrait_config.emit(portrait_payload)
            print(
                f"[SettingsPanel] Sinyal send_portrait_config dipancarkan: {portrait_payload}"
            )
        except Exception as e:
            print(f"[SettingsPanel] Error parsing photo mission WP: {e}")

    def _on_set_dock_config(self):
        try:
            payload = {
                "motor_utama_pwm": self.spin_dock_motor.value(),
                "motor_depan_pwm": self.spin_dock_front.value(),
                "servo_left": self.spin_dock_left.value(),
                "servo_right": self.spin_dock_right.value(),
                "charge_duration_ms": self.spin_dock_charge.value() * 1000,
            }
            self.send_dock_config.emit(payload)
            print(f"[SettingsPanel] Sinyal send_dock_config dipancarkan: {payload}")
        except Exception as e:
            print(f"[SettingsPanel] Error parsing dock config: {e}")
        except ValueError:
            print("[SettingsPanel] Error: Input indeks / jumlah foto tidak valid.")

    def _on_dock_enable_changed(self, state):
        try:
            is_enabled = (state == Qt.Checked)
            payload = {"enabled": is_enabled}
            self.send_dock_enabled.emit(payload)
            print(f"[SettingsPanel] Sinyal send_dock_enabled dipancarkan: {payload}")
        except Exception as e:
            print(f"[SettingsPanel] Error emitting dock enable: {e}")

    def _on_box_avoidance_changed(self):
        payload = {
            "speed": self.spin_box_speed.value(),
            "front_motor": self.spin_box_front.value(),
            "distance": self.spin_box_dist.value(),
            "left": self.spin_box_left.value(),
            "right": self.spin_box_right.value(),
            "pwm": self.spin_box_front.value(),
        }
        self.send_box_avoidance_config.emit(payload)
        
    def broadcast_all_settings(self):
        self._on_ai_speed_changed(self.spin_ai_speed.value())
        self._on_front_speed_changed()
        self._on_ai_servo_changed()
        self._on_obs_dist_changed(self.spin_obs_dist.value())
        self._on_model_changed(self.combo_model.currentText())
        self._on_wp_ranges_changed()
        self._on_set_photo_mission()
        self._on_set_dock_config()
        self._on_dock_enable_changed(self.chk_dock_enable.checkState())
        self._on_box_avoidance_changed()
