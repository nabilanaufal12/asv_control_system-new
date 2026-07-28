# gui/components/thruster_view.py
# --- MODIFIKASI: Menerima objek 'config' ---

from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QSlider,
    QSpinBox,
)
from PySide6.QtCore import Signal, Qt


class ThrusterView(QWidget):
    """
    Widget yang berisi slider untuk mengatur kecepatan manual.
    """

    # Sinyal yang membawa nilai kecepatan baru
    speed_changed = Signal(int)

    # --- 1. UBAH TANDA TANGAN FUNGSI __init__ ---
    def __init__(self, config):
        super().__init__()

        # --- 2. SIMPAN OBJEK KONFIGURASI ---
        self.config = config

        main_layout = QVBoxLayout(self)

        settings_group = QGroupBox("Manual Speed Setting")
        group_layout = QHBoxLayout()

        # Display kecepatan
        self.speed_label = QLabel("PWM Thruster:")

        # Slider kecepatan
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1000, 2000)

        # SpinBox kecepatan
        self.speed_spinbox = QSpinBox()
        self.speed_spinbox.setRange(1000, 2000)
        self.speed_spinbox.setFixedWidth(70)

        group_layout.addWidget(self.speed_label)
        group_layout.addWidget(self.speed_slider)
        group_layout.addWidget(self.speed_spinbox)
        settings_group.setLayout(group_layout)

        main_layout.addWidget(settings_group)
        main_layout.addStretch()

        # Hubungkan slider dan spinbox
        self.speed_slider.valueChanged.connect(self.speed_spinbox.setValue)
        self.speed_spinbox.valueChanged.connect(self.speed_slider.setValue)
        self.speed_slider.valueChanged.connect(self.speed_changed.emit)

        # Set default awal dari config jika ada, atau 1500
        actuator_config = self.config.get("actuators", {})
        pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
        self.speed_slider.setValue(pwm_stop)
