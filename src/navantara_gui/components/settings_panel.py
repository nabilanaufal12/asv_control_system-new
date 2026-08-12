# src/navantara_gui/components/settings_panel.py
from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QLabel,
    QSlider,
    QHBoxLayout,
    QComboBox,
    QLineEdit,
    QPushButton,
    QMessageBox,
    QFormLayout,
    QSpinBox,
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QIntValidator
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

    def __init__(self, config, title="System Configuration"):
        super().__init__(title)
        self.config = config
        main_layout = QVBoxLayout()

        # --- [BAGIAN KONTROL AI VISION & MISI] ---
        ai_control_group = QGroupBox("AI Engine & Thruster Parameters")
        ai_layout = QVBoxLayout()

        # Row 1: PWM Utama & PWM Depan
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("PWM Utama:"))
        self.spin_ai_speed = QSpinBox()
        self.spin_ai_speed.setRange(1000, 2000)
        self.spin_ai_speed.setValue(1300)
        row1.addWidget(self.spin_ai_speed)
        
        row1.addWidget(QLabel("PWM Depan:"))
        self.spin_front = QSpinBox()
        self.spin_front.setRange(1000, 2000)
        self.spin_front.setValue(1500)
        row1.addWidget(self.spin_front)
        
        ai_layout.addLayout(row1)

        # Row 2: Avoidance Angle (Left, Right)
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Avoidance Angle:"))
        self.spin_left = QSpinBox()
        self.spin_left.setRange(0, 90)
        self.spin_left.setValue(70)
        self.spin_left.setPrefix("Left: ")
        self.spin_left.setSuffix("°")
        row2.addWidget(self.spin_left)

        self.spin_right = QSpinBox()
        self.spin_right.setRange(90, 180)
        self.spin_right.setValue(110)
        self.spin_right.setPrefix("Right: ")
        self.spin_right.setSuffix("°")
        row2.addWidget(self.spin_right)
        
        ai_layout.addLayout(row2)

        # Row 3: AI Activation & AI Model
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("AI Activation:"))
        self.spin_obs_dist = QSpinBox()
        self.spin_obs_dist.setRange(0, 500)
        self.spin_obs_dist.setValue(165)
        self.spin_obs_dist.setSingleStep(10)
        self.spin_obs_dist.setSuffix(" cm")
        row3.addWidget(self.spin_obs_dist)

        row3.addWidget(QLabel("AI Model:"))
        self.combo_model = QComboBox()
        self.combo_model.addItems(["Auto", "best.engine", "best100.engine", "best.pt", "best100.pt"])
        row3.addWidget(self.combo_model)
        
        ai_layout.addLayout(row3)

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

        # --- [TAMBAHAN BARU] Photo Mission Segments (Pindahan dari Kanan) ---
        photo_mission_box = QGroupBox("Photography Mission Segments")
        photo_mission_layout = QVBoxLayout()

        photo_form_layout = QFormLayout()
        self.surf_wp1_input = QSpinBox()
        self.surf_wp1_input.setRange(0, 100)
        self.surf_wp1_input.setValue(13)

        self.surf_wp2_input = QSpinBox()
        self.surf_wp2_input.setRange(0, 100)
        self.surf_wp2_input.setValue(14)

        self.under_wp1_input = QSpinBox()
        self.under_wp1_input.setRange(0, 100)
        self.under_wp1_input.setValue(11)

        self.under_wp2_input = QSpinBox()
        self.under_wp2_input.setRange(0, 100)
        self.under_wp2_input.setValue(12)

        self.photo_count_input = QSpinBox()
        self.photo_count_input.setRange(1, 100)
        self.photo_count_input.setValue(5)

        # Surface WP
        photo_surf_layout = QHBoxLayout()
        photo_surf_layout.addWidget(QLabel("Start:"))
        photo_surf_layout.addWidget(self.surf_wp1_input)
        photo_surf_layout.addWidget(QLabel("End:"))
        photo_surf_layout.addWidget(self.surf_wp2_input)
        photo_form_layout.addRow("Surface WP:", photo_surf_layout)

        # Underwater WP
        photo_under_layout = QHBoxLayout()
        photo_under_layout.addWidget(QLabel("Start:"))
        photo_under_layout.addWidget(self.under_wp1_input)
        photo_under_layout.addWidget(QLabel("End:"))
        photo_under_layout.addWidget(self.under_wp2_input)
        photo_form_layout.addRow("Underwater WP:", photo_under_layout)
        photo_form_layout.addRow("Max Foto / Area:", self.photo_count_input)

        # --- Portrait Config Inputs ---
        self.spin_portrait_speed = QSpinBox()
        self.spin_portrait_speed.setRange(1000, 2000)
        self.spin_portrait_speed.setValue(1200)

        self.spin_portrait_rev_speed = QSpinBox()
        self.spin_portrait_rev_speed.setRange(1000, 2000)
        self.spin_portrait_rev_speed.setValue(1400)

        photo_speed_layout = QHBoxLayout()
        photo_speed_layout.addWidget(QLabel("Portrait:"))
        photo_speed_layout.addWidget(self.spin_portrait_speed)
        photo_speed_layout.addWidget(QLabel("Reverse:"))
        photo_speed_layout.addWidget(self.spin_portrait_rev_speed)
        photo_form_layout.addRow("Speed (PWM):", photo_speed_layout)

        self.spin_portrait_stop = QSpinBox()
        self.spin_portrait_stop.setRange(1, 15)
        self.spin_portrait_stop.setValue(3)

        self.spin_portrait_reverse = QSpinBox()
        self.spin_portrait_reverse.setRange(1, 15)
        self.spin_portrait_reverse.setValue(2)

        photo_duration_layout = QHBoxLayout()
        photo_duration_layout.addWidget(QLabel("Stop:"))
        photo_duration_layout.addWidget(self.spin_portrait_stop)
        photo_duration_layout.addWidget(QLabel("Reverse:"))
        photo_duration_layout.addWidget(self.spin_portrait_reverse)
        photo_form_layout.addRow("Durasi (Detik):", photo_duration_layout)

        photo_mission_layout.addLayout(photo_form_layout)
        photo_mission_box.setLayout(photo_mission_layout)

        main_layout.addWidget(photo_mission_box)

        # --- [TAMBAHAN BARU] Docking Mission Settings ---
        docking_mission_box = QGroupBox("Docking Mission Settings")
        docking_layout = QVBoxLayout()

        dock_row1 = QHBoxLayout()
        dock_row1.addWidget(QLabel("PWM Utama:"))
        self.spin_dock_motor = QSpinBox()
        self.spin_dock_motor.setRange(1000, 2000)
        self.spin_dock_motor.setValue(self.config.get("docking_defaults", {}).get("motor_utama_pwm", 1200))
        dock_row1.addWidget(self.spin_dock_motor)

        dock_row1.addWidget(QLabel("Depan:"))
        self.spin_dock_depan = QSpinBox()
        self.spin_dock_depan.setRange(1000, 2000)
        self.spin_dock_depan.setValue(self.config.get("docking_defaults", {}).get("motor_depan_pwm", 1400))
        dock_row1.addWidget(self.spin_dock_depan)

        docking_layout.addLayout(dock_row1)

        dock_row2 = QHBoxLayout()
        dock_row2.addWidget(QLabel("Durasi (s):"))
        self.spin_dock_charge = QSpinBox()
        self.spin_dock_charge.setRange(1, 10)
        self.spin_dock_charge.setValue(self.config.get("docking_defaults", {}).get("charge_duration_ms", 3000) // 1000)
        dock_row2.addWidget(self.spin_dock_charge)

        dock_row2.addWidget(QLabel("Toleransi (°):"))
        self.spin_dock_tol = QSpinBox()
        self.spin_dock_tol.setRange(1, 90)
        self.spin_dock_tol.setValue(self.config.get("docking_defaults", {}).get("heading_tolerance_deg", 5))
        dock_row2.addWidget(self.spin_dock_tol)

        docking_layout.addLayout(dock_row2)

        docking_mission_box.setLayout(docking_layout)
        main_layout.addWidget(docking_mission_box)

        # --- Tombol Simpan Default ---
        self.save_default_button = QPushButton("Save as Default Config")
        self.save_default_button.setStyleSheet(
            "background-color: #4CAF50; color: white; font-weight: bold; margin-top: 10px; padding: 5px;"
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

        # Event Handler Photo Mission Segments (Auto Update)
        self.surf_wp1_input.valueChanged.connect(self._on_set_photo_mission)
        self.surf_wp2_input.valueChanged.connect(self._on_set_photo_mission)
        self.under_wp1_input.valueChanged.connect(self._on_set_photo_mission)
        self.under_wp2_input.valueChanged.connect(self._on_set_photo_mission)
        self.photo_count_input.valueChanged.connect(self._on_set_photo_mission)

        self.spin_portrait_speed.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_rev_speed.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_stop.valueChanged.connect(self._on_set_photo_mission)
        self.spin_portrait_reverse.valueChanged.connect(self._on_set_photo_mission)
        
        self.spin_dock_motor.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_depan.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_charge.valueChanged.connect(self._on_set_dock_config)
        self.spin_dock_tol.valueChanged.connect(self._on_set_dock_config)

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
        if "front_motor" in defs:
            self.spin_front.setValue(defs["front_motor"])
        if "servo_left" in defs:
            self.spin_left.setValue(defs["servo_left"])
        if "servo_right" in defs:
            self.spin_right.setValue(defs["servo_right"])
        if "obs_dist" in defs:
            self.spin_obs_dist.setValue(defs["obs_dist"])

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
            "photo_surf1": self.surf_wp1_input.value(),
            "photo_surf2": self.surf_wp2_input.value(),
            "photo_under1": self.under_wp1_input.value(),
            "photo_under2": self.under_wp2_input.value(),
            "photo_count": self.photo_count_input.value(),
            "portrait_speed": self.spin_portrait_speed.value(),
            "portrait_rev_speed": self.spin_portrait_rev_speed.value(),
            "portrait_stop": self.spin_portrait_stop.value(),
            "portrait_reverse": self.spin_portrait_reverse.value(),
        }

        if "docking_defaults" not in self.config:
            self.config["docking_defaults"] = {}
        self.config["docking_defaults"]["motor_utama_pwm"] = self.spin_dock_motor.value()
        self.config["docking_defaults"]["motor_depan_pwm"] = self.spin_dock_depan.value()
        self.config["docking_defaults"]["charge_duration_ms"] = self.spin_dock_charge.value() * 1000
        self.config["docking_defaults"]["heading_tolerance_deg"] = self.spin_dock_tol.value()

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
                "motor_depan_pwm": self.spin_dock_depan.value(),
                "charge_duration_ms": self.spin_dock_charge.value() * 1000,
                "heading_tolerance_deg": self.spin_dock_tol.value()
            }
            self.send_dock_config.emit(payload)
            print(f"[SettingsPanel] Sinyal send_dock_config dipancarkan: {payload}")
        except Exception as e:
            print(f"[SettingsPanel] Error parsing dock config: {e}")
        except ValueError:
            print("[SettingsPanel] Error: Input indeks / jumlah foto tidak valid.")
