# gui/components/thruster_view.py
# --- MODIFIKASI: Menerima objek 'config' ---

from PySide6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QLabel, QSlider
from PySide6.QtCore import Signal, Qt


class ThrusterView(QWidget):
    """
    Widget yang berisi slider untuk mengatur kecepatan manual.
    """

    # Sinyal yang membawa nilai kecepatan baru (0-100)
    speed_changed = Signal(int)

    # --- 1. UBAH TANDA TANGAN FUNGSI __init__ ---
    def __init__(self, config):
        super().__init__()

        # --- 2. SIMPAN OBJEK KONFIGURASI ---
        self.config = config

        main_layout = QVBoxLayout(self)

        settings_group = QGroupBox("Manual Speed Setting")
        group_layout = QVBoxLayout()

        # Display kecepatan
        self.speed_label = QLabel("Speed: 0% | PWM: 1500")
        self.speed_label.setAlignment(Qt.AlignCenter)

        # Slider kecepatan
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(0)

        group_layout.addWidget(self.speed_label)
        group_layout.addWidget(self.speed_slider)
        settings_group.setLayout(group_layout)

        main_layout.addWidget(settings_group)
        main_layout.addStretch()

        # Hubungkan slider ke fungsi
        self.speed_slider.valueChanged.connect(self.update_speed_label)
        self.speed_slider.valueChanged.connect(self.speed_changed.emit)

        # Panggil sekali di awal untuk mengeset label dengan nilai dari config
        self.update_speed_label(0)

    def update_speed_label(self, value):
        """Mengupdate label kecepatan berdasarkan nilai slider."""
        # --- 3. MENGGUNAKAN NILAI DARI config.json ---
        actuator_config = self.config.get("actuators", {})
        pwm_stop = actuator_config.get("motor_pwm_stop", 1500)

        # Asumsi rentang PWM adalah 500 (misal, dari 1500 ke 2000)
        pwm_value = pwm_stop + (value * 5)
        self.speed_label.setText(f"Speed: {value}% | PWM: {int(pwm_value)}")
