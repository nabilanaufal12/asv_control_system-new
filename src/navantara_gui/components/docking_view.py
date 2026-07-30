import os
import json
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QGridLayout,
    QLabel,
    QSpinBox,
    QPushButton,
    QMessageBox
)
from PySide6.QtCore import Qt, Signal

class DockingView(QWidget):
    docking_config_updated = Signal(dict)

    def __init__(self, config):
        super().__init__()
        self.config = config
        
        main_layout = QVBoxLayout()
        
        # --- Arena A Group ---
        arena_a_group = QGroupBox("Arena A (Left Docking)")
        layout_a = QGridLayout()
        
        self.spin_a_servo = self._create_spinbox(0, 180, 45, "°")
        self.spin_a_dur = self._create_spinbox(0, 60, 20, " s")
        self.spin_a_r_pwm = self._create_spinbox(1000, 2000, 1100)
        self.spin_a_r_dir = self._create_spinbox(1000, 2000, 2000)
        self.spin_a_fl_pwm = self._create_spinbox(1000, 2000, 1200)
        self.spin_a_fl_dir = self._create_spinbox(1000, 2000, 2000)
        self.spin_a_fr_pwm = self._create_spinbox(1000, 2000, 1200)
        self.spin_a_fr_dir = self._create_spinbox(1000, 2000, 1000)

        self._populate_grid(layout_a, self.spin_a_servo, self.spin_a_dur, self.spin_a_r_pwm, self.spin_a_r_dir, 
                            self.spin_a_fl_pwm, self.spin_a_fl_dir, self.spin_a_fr_pwm, self.spin_a_fr_dir)
        arena_a_group.setLayout(layout_a)

        # --- Arena B Group ---
        arena_b_group = QGroupBox("Arena B (Right Docking)")
        layout_b = QGridLayout()
        
        self.spin_b_servo = self._create_spinbox(0, 180, 135, "°")
        self.spin_b_dur = self._create_spinbox(0, 60, 20, " s")
        self.spin_b_r_pwm = self._create_spinbox(1000, 2000, 1100)
        self.spin_b_r_dir = self._create_spinbox(1000, 2000, 2000)
        self.spin_b_fl_pwm = self._create_spinbox(1000, 2000, 1200)
        self.spin_b_fl_dir = self._create_spinbox(1000, 2000, 1000)
        self.spin_b_fr_pwm = self._create_spinbox(1000, 2000, 1200)
        self.spin_b_fr_dir = self._create_spinbox(1000, 2000, 2000)

        self._populate_grid(layout_b, self.spin_b_servo, self.spin_b_dur, self.spin_b_r_pwm, self.spin_b_r_dir, 
                            self.spin_b_fl_pwm, self.spin_b_fl_dir, self.spin_b_fr_pwm, self.spin_b_fr_dir)
        arena_b_group.setLayout(layout_b)

        # --- Save Button ---
        self.btn_save = QPushButton("Save Docking Config")
        self.btn_save.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 10px;")
        self.btn_save.clicked.connect(self.save_config)

        main_layout.addWidget(arena_a_group)
        main_layout.addWidget(arena_b_group)
        main_layout.addWidget(self.btn_save)
        main_layout.addStretch()
        
        self.setLayout(main_layout)
        
        # Load Existing Data
        self.load_config()

    def _create_spinbox(self, minimum, maximum, default, suffix=""):
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(default)
        if suffix:
            spin.setSuffix(suffix)
        return spin

    def _populate_grid(self, layout, servo, dur, r_pwm, r_dir, fl_pwm, fl_dir, fr_pwm, fr_dir):
        layout.addWidget(QLabel("Servo Angle:"), 0, 0)
        layout.addWidget(servo, 0, 1)
        layout.addWidget(QLabel("Duration:"), 0, 2)
        layout.addWidget(dur, 0, 3)
        
        layout.addWidget(QLabel("Rear PWM/Dir:"), 1, 0)
        layout.addWidget(r_pwm, 1, 1)
        layout.addWidget(r_dir, 1, 2)
        
        layout.addWidget(QLabel("Front L PWM/Dir:"), 2, 0)
        layout.addWidget(fl_pwm, 2, 1)
        layout.addWidget(fl_dir, 2, 2)
        
        layout.addWidget(QLabel("Front R PWM/Dir:"), 3, 0)
        layout.addWidget(fr_pwm, 3, 1)
        layout.addWidget(fr_dir, 3, 2)

    def load_config(self):
        try:
            dock_cfg = self.config.get("auto_missions", {}).get("docking", {})
            
            # Arena A
            a = dock_cfg.get("Arena_A", {})
            self.spin_a_servo.setValue(a.get("servo_angle", 45))
            self.spin_a_dur.setValue(int(a.get("duration_sec", 20)))
            self.spin_a_r_pwm.setValue(a.get("rear_pwm", 1100))
            # Di config JSON versi awal belum ada rear_dir, gunakan fallback 2000
            self.spin_a_r_dir.setValue(a.get("rear_dir", 2000)) 
            self.spin_a_fl_pwm.setValue(a.get("front_left_pwm", 1200))
            self.spin_a_fl_dir.setValue(a.get("front_left_dir", 2000))
            self.spin_a_fr_pwm.setValue(a.get("front_right_pwm", 1200))
            self.spin_a_fr_dir.setValue(a.get("front_right_dir", 1000))

            # Arena B
            b = dock_cfg.get("Arena_B", {})
            self.spin_b_servo.setValue(b.get("servo_angle", 135))
            self.spin_b_dur.setValue(int(b.get("duration_sec", 20)))
            self.spin_b_r_pwm.setValue(b.get("rear_pwm", 1100))
            self.spin_b_r_dir.setValue(b.get("rear_dir", 2000))
            self.spin_b_fl_pwm.setValue(b.get("front_left_pwm", 1200))
            self.spin_b_fl_dir.setValue(b.get("front_left_dir", 1000))
            self.spin_b_fr_pwm.setValue(b.get("front_right_pwm", 1200))
            self.spin_b_fr_dir.setValue(b.get("front_right_dir", 2000))
        except Exception as e:
            print("Error loading config:", e)

    def save_config(self):
        try:
            payload = {
                "docking": {
                    "enabled": True,
                    "trigger_wp_index": 17,
                    "Arena_A": {
                        "servo_angle": self.spin_a_servo.value(),
                        "duration_sec": float(self.spin_a_dur.value()),
                        "rear_pwm": self.spin_a_r_pwm.value(),
                        "rear_dir": self.spin_a_r_dir.value(),
                        "front_left_pwm": self.spin_a_fl_pwm.value(),
                        "front_left_dir": self.spin_a_fl_dir.value(),
                        "front_right_pwm": self.spin_a_fr_pwm.value(),
                        "front_right_dir": self.spin_a_fr_dir.value(),
                    },
                    "Arena_B": {
                        "servo_angle": self.spin_b_servo.value(),
                        "duration_sec": float(self.spin_b_dur.value()),
                        "rear_pwm": self.spin_b_r_pwm.value(),
                        "rear_dir": self.spin_b_r_dir.value(),
                        "front_left_pwm": self.spin_b_fl_pwm.value(),
                        "front_left_dir": self.spin_b_fl_dir.value(),
                        "front_right_pwm": self.spin_b_fr_pwm.value(),
                        "front_right_dir": self.spin_b_fr_dir.value(),
                    }
                }
            }
                
            self.docking_config_updated.emit(payload)
                
            QMessageBox.information(self, "Success", "Docking Config Sent to Backend!")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save config: {str(e)}")
