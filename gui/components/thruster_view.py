# gui/components/thruster_view.py
# Komponen untuk mengatur kecepatan manual thruster.

from PySide6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QLabel, QSlider
from PySide6.QtCore import Signal, Qt

class ThrusterView(QWidget):
    """
    Widget yang berisi slider untuk mengatur kecepatan manual.
    """
    # Sinyal yang membawa nilai kecepatan baru (0-100)
    speed_changed = Signal(int)

    def __init__(self):
        super().__init__()

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

    def update_speed_label(self, value):
        """Mengupdate label kecepatan berdasarkan nilai slider."""
        # Asumsi PWM 1500 (netral) hingga 2000 (maks)
        pwm_value = 1500 + (value * 5)
        self.speed_label.setText(f"Speed: {value}% | PWM: {pwm_value}")

