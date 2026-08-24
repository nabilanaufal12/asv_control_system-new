# src/navantara_gui/components/settings_panel.py
import json
import os
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QSpinBox,
    QDoubleSpinBox,
    QVBoxLayout,
    QCheckBox,
    QFrame,
)


class SettingsPanel(QGroupBox):
    """
    Panel konfigurasi misi ASV NAVANTARA (Misi 1: Bola, Misi 2: Foto, Misi 3: Docking).
    """

    vision_speed_updated = Signal(int)
    vision_front_motor_updated = Signal(dict)
    vision_servo_updated = Signal(dict)
    vision_distance_updated = Signal(float)
    vision_model_updated = Signal(str)
    vision_wp_ranges_updated = Signal(dict)

    send_photo_mission = Signal(dict)
    send_portrait_config = Signal(dict)
    send_dock_config = Signal(dict)
    send_dock_enabled = Signal(dict)
    send_box_avoidance_config = Signal(dict)

    def __init__(self, config, title="System Configuration"):
        super().__init__(title)
        self.config = config
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        def setup_grid_layout():
            grid = QGridLayout()
            grid.setHorizontalSpacing(6)
            grid.setVerticalSpacing(4)
            grid.setContentsMargins(6, 6, 6, 6)
            grid.setColumnStretch(0, 0)
            grid.setColumnStretch(1, 1)
            grid.setColumnStretch(2, 0)
            grid.setColumnStretch(3, 1)
            return grid

        def create_sub_header(text):
            label = QLabel(f"• {text}")
            label.setStyleSheet("font-weight: bold; color: #0088cc; margin-top: 3px;")
            return label

        # =====================================================================
        # 1. MISI 1: RINTANGAN BOLA (Red & Green Buoys)
        # =====================================================================
        misi1_group = QGroupBox("Misi 1: Rintangan Bola (Red/Green Buoy)")
        misi1_grid = setup_grid_layout()

        self.spin_wp_bola_start = QSpinBox()
        self.spin_wp_bola_start.setRange(0, 100)
        self.spin_wp_bola_start.setPrefix("Start: ")

        self.spin_wp_bola_end = QSpinBox()
        self.spin_wp_bola_end.setRange(0, 100)
        self.spin_wp_bola_end.setPrefix("End: ")

        self.spin_ai_speed = QSpinBox()
        self.spin_ai_speed.setRange(1000, 2000)
        self.spin_ai_speed.setValue(1500)

        self.spin_front = QSpinBox()
        self.spin_front.setRange(1000, 2000)
        self.spin_front.setValue(1800)

        self.spin_left = QSpinBox()
        self.spin_left.setRange(0, 180)
        self.spin_left.setValue(70)
        self.spin_left.setPrefix("Left: ")
        self.spin_left.setSuffix("°")

        self.spin_right = QSpinBox()
        self.spin_right.setRange(0, 180)
        self.spin_right.setValue(110)
        self.spin_right.setPrefix("Right: ")
        self.spin_right.setSuffix("°")

        self.spin_obs_dist = QSpinBox()
        self.spin_obs_dist.setRange(0, 500)
        self.spin_obs_dist.setValue(150)
        self.spin_obs_dist.setSingleStep(10)
        self.spin_obs_dist.setSuffix(" cm")

        self.combo_model = QComboBox()
        self.combo_model.addItems(
            ["Auto", "best.engine", "best100.engine", "best.pt", "best100.pt"]
        )

        # Row 0: WP Range Bola
        misi1_grid.addWidget(QLabel("WP Bola:"), 0, 0)
        misi1_grid.addWidget(self.spin_wp_bola_start, 0, 1)
        misi1_grid.addWidget(QLabel("—"), 0, 2, alignment=Qt.AlignCenter)
        misi1_grid.addWidget(self.spin_wp_bola_end, 0, 3)

        # Row 1: Kecepatan
        misi1_grid.addWidget(QLabel("Speed:"), 1, 0)
        misi1_grid.addWidget(self.spin_ai_speed, 1, 1)
        misi1_grid.addWidget(QLabel("Front:"), 1, 2)
        misi1_grid.addWidget(self.spin_front, 1, 3)

        # Row 2: Kemudi
        misi1_grid.addWidget(QLabel("Avoid L:"), 2, 0)
        misi1_grid.addWidget(self.spin_left, 2, 1)
        misi1_grid.addWidget(QLabel("Avoid R:"), 2, 2)
        misi1_grid.addWidget(self.spin_right, 2, 3)

        # Row 3: Trigger Jarak & Model
        misi1_grid.addWidget(QLabel("Obstacle:"), 3, 0)
        misi1_grid.addWidget(self.spin_obs_dist, 3, 1)
        misi1_grid.addWidget(QLabel("Model:"), 3, 2)
        misi1_grid.addWidget(self.combo_model, 3, 3)

        misi1_group.setLayout(misi1_grid)
        main_layout.addWidget(misi1_group)

        # =====================================================================
        # 2. MISI 2: FOTOGRAFI KOTAK (Bawah Air & Permukaan)
        # =====================================================================
        misi2_group = QGroupBox("Misi 2: Fotografi Kotak (UW & Surface)")
        misi2_grid = setup_grid_layout()

        # Input Waypoint (Single Source of Truth untuk Vision & Auto-Photo)
        self.under_wp1_input = QSpinBox()
        self.under_wp1_input.setRange(0, 100)
        self.under_wp1_input.setValue(11)
        self.under_wp1_input.setPrefix("Start: ")

        self.under_wp2_input = QSpinBox()
        self.under_wp2_input.setRange(0, 100)
        self.under_wp2_input.setValue(12)
        self.under_wp2_input.setPrefix("End: ")

        self.surf_wp1_input = QSpinBox()
        self.surf_wp1_input.setRange(0, 100)
        self.surf_wp1_input.setValue(13)
        self.surf_wp1_input.setPrefix("Start: ")

        self.surf_wp2_input = QSpinBox()
        self.surf_wp2_input.setRange(0, 100)
        self.surf_wp2_input.setValue(14)
        self.surf_wp2_input.setPrefix("End: ")

        # Aliases agar kompatibel 100% jika ada modul yang memanggil spin_wp_biru/hijau
        self.spin_wp_biru_start = self.under_wp1_input
        self.spin_wp_biru_end = self.under_wp2_input
        self.spin_wp_hijau_start = self.surf_wp1_input
        self.spin_wp_hijau_end = self.surf_wp2_input

        # Jarak Deteksi Kotak (Zona Tracking vs Avoidance)
        self.spin_box_track_dist = QSpinBox()
        self.spin_box_track_dist.setRange(0, 500)
        self.spin_box_track_dist.setValue(165)
        self.spin_box_track_dist.setSuffix(" cm")

        self.spin_box_avoid_dist = QSpinBox()
        self.spin_box_avoid_dist.setRange(0, 500)
        self.spin_box_avoid_dist.setValue(100)
        self.spin_box_avoid_dist.setSuffix(" cm")

        # Manuver Pemotretan di Titik 12 / 14
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

        self.photo_count_input = QSpinBox()
        self.photo_count_input.setRange(1, 100)
        self.photo_count_input.setValue(10)

        self.photo_interval_input = QDoubleSpinBox()
        self.photo_interval_input.setRange(0.1, 10.0)
        self.photo_interval_input.setSingleStep(0.1)
        self.photo_interval_input.setValue(0.8)
        self.photo_interval_input.setSuffix(" s")

        # Manuver Menghindar Kotak (Pasca Foto)
        self.spin_box_speed = QSpinBox()
        self.spin_box_speed.setRange(1000, 2000)
        self.spin_box_speed.setValue(1500)

        self.spin_box_front = QSpinBox()
        self.spin_box_front.setRange(1000, 2000)
        self.spin_box_front.setValue(1800)

        self.spin_box_left = QSpinBox()
        self.spin_box_left.setRange(0, 180)
        self.spin_box_left.setValue(70)
        self.spin_box_left.setPrefix("Left: ")
        self.spin_box_left.setSuffix("°")

        self.spin_box_right = QSpinBox()
        self.spin_box_right.setRange(0, 180)
        self.spin_box_right.setValue(110)
        self.spin_box_right.setPrefix("Right: ")
        self.spin_box_right.setSuffix("°")

        # --- Penyusunan Layout Misi 2 ---
        row = 0
        # Section A: Target WP Foto
        misi2_grid.addWidget(create_sub_header("Target Waypoint Foto:"), row, 0, 1, 4)
        row += 1
        misi2_grid.addWidget(QLabel("Kotak UW:"), row, 0)
        misi2_grid.addWidget(self.under_wp1_input, row, 1)
        misi2_grid.addWidget(QLabel("—"), row, 2, alignment=Qt.AlignCenter)
        misi2_grid.addWidget(self.under_wp2_input, row, 3)
        row += 1
        misi2_grid.addWidget(QLabel("Kotak Sfc:"), row, 0)
        misi2_grid.addWidget(self.surf_wp1_input, row, 1)
        misi2_grid.addWidget(QLabel("—"), row, 2, alignment=Qt.AlignCenter)
        misi2_grid.addWidget(self.surf_wp2_input, row, 3)
        row += 1

        # Section B: Jarak Deteksi Kotak
        misi2_grid.addWidget(create_sub_header("Jarak Deteksi Kotak:"), row, 0, 1, 4)
        row += 1
        misi2_grid.addWidget(QLabel("Track Dist:"), row, 0)
        misi2_grid.addWidget(self.spin_box_track_dist, row, 1)
        misi2_grid.addWidget(QLabel("Avoid Dist:"), row, 2)
        misi2_grid.addWidget(self.spin_box_avoid_dist, row, 3)
        row += 1

        # Section C: Manuver Pemotretan
        misi2_grid.addWidget(create_sub_header("Manuver Pemotretan (Titik Foto):"), row, 0, 1, 4)
        row += 1
        misi2_grid.addWidget(QLabel("Spd Maju:"), row, 0)
        misi2_grid.addWidget(self.spin_portrait_speed, row, 1)
        misi2_grid.addWidget(QLabel("Spd Mundur:"), row, 2)
        misi2_grid.addWidget(self.spin_portrait_rev_speed, row, 3)
        row += 1
        misi2_grid.addWidget(QLabel("Waktu Diam:"), row, 0)
        misi2_grid.addWidget(self.spin_portrait_stop, row, 1)
        misi2_grid.addWidget(QLabel("Waktu Mundur:"), row, 2)
        misi2_grid.addWidget(self.spin_portrait_reverse, row, 3)
        row += 1
        misi2_grid.addWidget(QLabel("Target Foto:"), row, 0)
        misi2_grid.addWidget(self.photo_count_input, row, 1)
        misi2_grid.addWidget(QLabel("Interval:"), row, 2)
        misi2_grid.addWidget(self.photo_interval_input, row, 3)
        row += 1

        # Section D: Manuver Menghindar Kotak
        misi2_grid.addWidget(create_sub_header("Manuver Menghindar (Pasca Foto):"), row, 0, 1, 4)
        row += 1
        misi2_grid.addWidget(QLabel("Avoid Spd:"), row, 0)
        misi2_grid.addWidget(self.spin_box_speed, row, 1)
        misi2_grid.addWidget(QLabel("Avoid Front:"), row, 2)
        misi2_grid.addWidget(self.spin_box_front, row, 3)
        row += 1
        misi2_grid.addWidget(QLabel("Avoid L:"), row, 0)
        misi2_grid.addWidget(self.spin_box_left, row, 1)
        misi2_grid.addWidget(QLabel("Avoid R:"), row, 2)
        misi2_grid.addWidget(self.spin_box_right, row, 3)

        misi2_group.setLayout(misi2_grid)
        main_layout.addWidget(misi2_group)

        # =====================================================================
        # 3. MISI 3: DOCKING AKHIR (WP Terakhir)
        # =====================================================================
        misi3_group = QGroupBox("Misi 3: Docking Akhir (Last WP)")
        dock_grid = setup_grid_layout()

        self.chk_dock_enable = QCheckBox("Aktifkan Misi Docking")
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
        self.spin_dock_left.setRange(0, 180)
        self.spin_dock_left.setValue(
            self.config.get("docking_defaults", {}).get("servo_left", 180)
        )
        self.spin_dock_left.setPrefix("Left: ")
        self.spin_dock_left.setSuffix("°")

        self.spin_dock_right = QSpinBox()
        self.spin_dock_right.setRange(0, 180)
        self.spin_dock_right.setValue(
            self.config.get("docking_defaults", {}).get("servo_right", 0)
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

        dock_grid.addWidget(QLabel("Speed:"), 1, 0)
        dock_grid.addWidget(self.spin_dock_motor, 1, 1)
        dock_grid.addWidget(QLabel("Front:"), 1, 2)
        dock_grid.addWidget(self.spin_dock_front, 1, 3)

        dock_grid.addWidget(QLabel("Swing L:"), 2, 0)
        dock_grid.addWidget(self.spin_dock_left, 2, 1)
        dock_grid.addWidget(QLabel("Swing R:"), 2, 2)
        dock_grid.addWidget(self.spin_dock_right, 2, 3)

        dock_grid.addWidget(QLabel("Charge Time:"), 3, 0)
        dock_grid.addWidget(self.spin_dock_charge, 3, 1)

        misi3_group.setLayout(dock_grid)
        main_layout.addWidget(misi3_group)

        # --- Tombol Simpan Default ---
        self.save_default_button = QPushButton("💾 Save as Default Config")
        self.save_default_button.setStyleSheet(
            "background-color: #2e7d32; color: white; font-weight: bold; margin-top: 4px; padding: 6px; border-radius: 4px;"
        )
        main_layout.addWidget(self.save_default_button)

        # Set default values from config if available
        self._load_defaults_from_config()

        # Sembunyikan tombol panah spinbox agar tampilan bersih
        for spin_box in self.findChildren(QSpinBox):
            spin_box.setButtonSymbols(QSpinBox.NoButtons)
        for spin_box in self.findChildren(QDoubleSpinBox):
            spin_box.setButtonSymbols(QDoubleSpinBox.NoButtons)

        self.setLayout(main_layout)

        # =====================================================================
        # KONEKSI SINYAL EVENT
        # =====================================================================
        # Event Handler Misi 1 (Bola)
        self.spin_ai_speed.valueChanged.connect(self._on_ai_speed_changed)
        self.spin_front.valueChanged.connect(self._on_front_speed_changed)
        self.spin_left.valueChanged.connect(self._on_ai_servo_changed)
        self.spin_right.valueChanged.connect(self._on_ai_servo_changed)
        self.spin_obs_dist.valueChanged.connect(self._on_obs_dist_changed)
        self.combo_model.currentTextChanged.connect(self._on_model_changed)
        self.spin_wp_bola_start.valueChanged.connect(self._on_unified_wp_changed)
        self.spin_wp_bola_end.valueChanged.connect(self._on_unified_wp_changed)

        # Event Handler Misi 2 (Fotografi Kotak & Avoidance)
        self.under_wp1_input.valueChanged.connect(self._on_unified_wp_changed)
        self.under_wp2_input.valueChanged.connect(self._on_unified_wp_changed)
        self.surf_wp1_input.valueChanged.connect(self._on_unified_wp_changed)
        self.surf_wp2_input.valueChanged.connect(self._on_unified_wp_changed)

        self.spin_box_track_dist.valueChanged.connect(self._on_box_avoidance_changed)
        self.spin_box_avoid_dist.valueChanged.connect(self._on_box_avoidance_changed)
        self.spin_box_left.valueChanged.connect(self._on_box_avoidance_changed)
        self.spin_box_right.valueChanged.connect(self._on_box_avoidance_changed)
        self.spin_box_speed.valueChanged.connect(self._on_box_avoidance_changed)
        self.spin_box_front.valueChanged.connect(self._on_box_avoidance_changed)

        self.photo_count_input.valueChanged.connect(self._on_set_photo_mission)
        self.photo_interval_input.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_speed.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_rev_speed.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_stop.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_reverse.valueChanged.connect(self._on_set_photo_mission)

        # Event Handler Misi 3 (Docking)
        self.spin_dock_motor.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_front.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_left.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_right.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_charge.valueChanged.connect(self._on_set_dock_config)
        self.chk_dock_enable.stateChanged.connect(self._on_dock_enable_changed)

        self.save_default_button.clicked.connect(self._save_defaults_to_config)

    def _load_defaults_from_config(self):
        vision_cfg = self.config.get("vision", {})
        rb = vision_cfg.get("wp_range_bola", [0, 10])
        rbb = vision_cfg.get("wp_range_kotak_biru", [11, 12])
        rbh = vision_cfg.get("wp_range_kotak_hijau", [13, 14])

        self.spin_wp_bola_start.blockSignals(True)
        self.spin_wp_bola_end.blockSignals(True)
        self.under_wp1_input.blockSignals(True)
        self.under_wp2_input.blockSignals(True)
        self.surf_wp1_input.blockSignals(True)
        self.surf_wp2_input.blockSignals(True)

        self.spin_wp_bola_start.setValue(rb[0])
        self.spin_wp_bola_end.setValue(rb[1])
        self.under_wp1_input.setValue(rbb[0])
        self.under_wp2_input.setValue(rbb[1])
        self.surf_wp1_input.setValue(rbh[0])
        self.surf_wp2_input.setValue(rbh[1])

        self.spin_wp_bola_start.blockSignals(False)
        self.spin_wp_bola_end.blockSignals(False)
        self.under_wp1_input.blockSignals(False)
        self.under_wp2_input.blockSignals(False)
        self.surf_wp1_input.blockSignals(False)
        self.surf_wp2_input.blockSignals(False)

        # Load GUI Defaults jika ada
        gui_cfg = self.config.get("gui_settings", {})
        defs = gui_cfg.get("panel_defaults", {})

        if "ai_speed" in defs:
            self.spin_ai_speed.setValue(defs["ai_speed"])
        if "front_motor" in defs:
            self.spin_front.setValue(defs["front_motor"])
        if "servo_left" in defs:
            self.spin_left.setValue(defs["servo_left"])
        if "servo_right" in defs:
            self.spin_right.setValue(defs["servo_right"])
        if "obs_dist" in defs:
            self.spin_obs_dist.setValue(defs["obs_dist"])

        if "box_track_dist" in defs:
            self.spin_box_track_dist.setValue(defs["box_track_dist"])
        elif "box_obs_dist" in defs:
            self.spin_box_track_dist.setValue(defs["box_obs_dist"])

        if "box_avoid_dist" in defs:
            self.spin_box_avoid_dist.setValue(defs["box_avoid_dist"])
        elif "obs_dist" in defs:
            self.spin_box_avoid_dist.setValue(defs.get("box_avoid_dist", 100))

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
        if "photo_interval" in defs:
            self.photo_interval_input.setValue(float(defs["photo_interval"]))

        if "portrait_speed" in defs:
            self.spin_portrait_speed.setValue(defs["portrait_speed"])
        if "portrait_rev_speed" in defs:
            self.spin_portrait_rev_speed.setValue(defs["portrait_rev_speed"])
        if "portrait_stop" in defs:
            self.spin_portrait_stop.setValue(defs["portrait_stop"])
        if "portrait_reverse" in defs:
            self.spin_portrait_reverse.setValue(defs["portrait_reverse"])

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

    def _save_defaults_to_config(self):
        if "vision" not in self.config:
            self.config["vision"] = {}
        self.config["vision"]["wp_range_bola"] = [
            self.spin_wp_bola_start.value(),
            self.spin_wp_bola_end.value(),
        ]
        self.config["vision"]["wp_range_kotak_biru"] = [
            self.under_wp1_input.value(),
            self.under_wp2_input.value(),
        ]
        self.config["vision"]["wp_range_kotak_hijau"] = [
            self.surf_wp1_input.value(),
            self.surf_wp2_input.value(),
        ]

        if "gui_settings" not in self.config:
            self.config["gui_settings"] = {}

        self.config["gui_settings"]["panel_defaults"] = {
            "ai_speed": self.spin_ai_speed.value(),
            "front_motor": self.spin_front.value(),
            "servo_left": self.spin_left.value(),
            "servo_right": self.spin_right.value(),
            "obs_dist": self.spin_obs_dist.value(),
            "box_obs_dist": self.spin_box_avoid_dist.value(),
            "box_track_dist": self.spin_box_track_dist.value(),
            "box_avoid_dist": self.spin_box_avoid_dist.value(),
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
        self.config["docking_defaults"]["motor_utama_pwm"] = self.spin_dock_motor.value()
        self.config["docking_defaults"]["motor_depan_pwm"] = self.spin_dock_front.value()
        self.config["docking_defaults"]["servo_left"] = self.spin_dock_left.value()
        self.config["docking_defaults"]["servo_right"] = self.spin_dock_right.value()
        self.config["docking_defaults"]["charge_duration_ms"] = (
            self.spin_dock_charge.value() * 1000
        )

        try:
            gui_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(gui_dir)))
            config_path = os.path.join(project_root, "config", "config.json")
            with open(config_path, "w") as f:
                json.dump(self.config, f, indent=2)
            print(f"[SettingsPanel] Default konfigurasi berhasil disimpan ke {config_path}")
        except Exception as e:
            print(f"[SettingsPanel] Gagal menyimpan konfigurasi default: {e}")

    def broadcast_all_settings(self):
        """Kirim semua setting ke backend secara serentak (berguna saat baru connect)."""
        print("[SettingsPanel] Melakukan broadcast semua pengaturan ke backend...")
        self._on_model_changed(self.combo_model.currentText())
        self._on_ai_speed_changed(self.spin_ai_speed.value())
        self._on_front_speed_changed()
        self._on_ai_servo_changed()
        self._on_obs_dist_changed(self.spin_obs_dist.value())
        self._on_unified_wp_changed()
        self._on_box_avoidance_changed()
        self._on_set_photo_mission()
        self._on_set_dock_config()
        self._on_dock_enable_changed(self.chk_dock_enable.checkState())

    # --- SLOT HANDLERS ---
    def _on_model_changed(self, text):
        self.vision_model_updated.emit(text)

    def _on_ai_speed_changed(self, value):
        self.vision_speed_updated.emit(value)

    def _on_front_speed_changed(self):
        val = self.spin_front.value()
        self.vision_front_motor_updated.emit({"pwm": val})

    def _on_ai_servo_changed(self):
        payload = {"left": self.spin_left.value(), "right": self.spin_right.value()}
        self.vision_servo_updated.emit(payload)

    def _on_obs_dist_changed(self, value):
        self.vision_distance_updated.emit(float(value))

    def _on_unified_wp_changed(self):
        """
        Handler terpadu: menyinkronkan filter target Vision dan trigger Auto-Photo sekaligus.
        """
        wp_ranges_payload = {
            "wp_range_bola": [
                self.spin_wp_bola_start.value(),
                self.spin_wp_bola_end.value(),
            ],
            "wp_range_kotak_biru": [
                self.under_wp1_input.value(),
                self.under_wp2_input.value(),
            ],
            "wp_range_kotak_hijau": [
                self.surf_wp1_input.value(),
                self.surf_wp2_input.value(),
            ],
        }
        self.vision_wp_ranges_updated.emit(wp_ranges_payload)
        self._on_set_photo_mission()

    def _on_wp_ranges_changed(self):
        self._on_unified_wp_changed()

    def _on_set_photo_mission(self):
        """Handler untuk set misi foto segmen dan parameter manuver portrait."""
        try:
            surf1 = self.surf_wp1_input.value()
            surf2 = self.surf_wp2_input.value()
            under1 = self.under_wp1_input.value()
            under2 = self.under_wp2_input.value()
            count = self.photo_count_input.value()

            if count <= 0:
                return

            payload = {
                "surf_wp1": surf1,
                "surf_wp2": surf2,
                "under_wp1": under1,
                "under_wp2": under2,
                "count": count,
            }
            self.send_photo_mission.emit(payload)

            port_speed = self.spin_portrait_speed.value()
            rev_speed = self.spin_portrait_rev_speed.value()
            stop_sec = self.spin_portrait_stop.value()
            rev_sec = self.spin_portrait_reverse.value()

            portrait_payload = {
                "speed": port_speed,
                "stop_ms": int(stop_sec * 1000),
                "reverse_ms": int(rev_sec * 1000),
                "reverse_speed": rev_speed,
                "interval": self.photo_interval_input.value(),
            }
            self.send_portrait_config.emit(portrait_payload)
        except Exception as e:
            print(f"[SettingsPanel] Error setting photo mission: {e}")

    def _on_set_dock_config(self):
        try:
            m_utama = self.spin_dock_motor.value()
            m_depan = self.spin_dock_front.value()
            s_left = self.spin_dock_left.value()
            s_right = self.spin_dock_right.value()
            charge_sec = self.spin_dock_charge.value()

            payload = {
                "motor_utama_pwm": m_utama,
                "motor_depan_pwm": m_depan,
                "servo_left": s_left,
                "servo_right": s_right,
                "charge_duration_ms": int(charge_sec * 1000),
            }
            self.send_dock_config.emit(payload)
        except Exception as e:
            print(f"[SettingsPanel] Error emitting dock config: {e}")

    def _on_dock_enable_changed(self, state):
        try:
            is_enabled = bool(state == Qt.Checked or state == 2 or state is True)
            payload = {"enabled": is_enabled}
            self.send_dock_enabled.emit(payload)
            print(f"[SettingsPanel] Sinyal send_dock_enabled dipancarkan: {payload}")
        except Exception as e:
            print(f"[SettingsPanel] Error emitting dock enable: {e}")

    def _on_box_avoidance_changed(self):
        payload = {
            "track_dist": self.spin_box_track_dist.value(),
            "avoid_dist": self.spin_box_avoid_dist.value(),
            "distance": self.spin_box_track_dist.value(),
            "safety_dist": self.spin_box_avoid_dist.value(),
            "speed": self.spin_box_speed.value(),
            "front_motor": self.spin_box_front.value(),
            "left": self.spin_box_left.value(),
            "right": self.spin_box_right.value(),
            "pwm": self.spin_box_front.value(),
        }
        self.send_box_avoidance_config.emit(payload)
