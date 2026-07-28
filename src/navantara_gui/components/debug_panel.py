# gui/components/debug_panel.py
# --- MODIFIKASI: Menerima objek 'config' ---

from PySide6.QtWidgets import (
    QGroupBox,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QFormLayout,
    QLineEdit,
    QDoubleSpinBox,
    QLabel,
)
from PySide6.QtCore import Signal


class DebugPanel(QWidget):
    """
    Widget yang berisi berbagai alat untuk debugging dan kalibrasi,
    diambil dari fungsionalitas GUI lama.
    """

    debug_command_sent = Signal(str, object)

    # --- 1. UBAH TANDA TANGAN FUNGSI __init__ ---
    def __init__(self, config):
        super().__init__()

        # --- 2. SIMPAN OBJEK KONFIGURASI ---
        self.config = config

        main_layout = QVBoxLayout(self)

        # === Panel Kontrol AI & Motor ===
        ai_control_group = QGroupBox("Raw Motor Diagnostics")
        ai_layout = QVBoxLayout()

        ai_buttons_layout = QHBoxLayout()
        self.counter_plus_btn = QPushButton("Step PWM (+)")
        self.counter_min_btn = QPushButton("Step PWM (-)")
        self.inverse_btn = QPushButton("Invert Motor Polarity")
        self.inverse_btn.setCheckable(True)
        ai_buttons_layout.addWidget(self.counter_plus_btn)
        ai_buttons_layout.addWidget(self.counter_min_btn)
        ai_buttons_layout.addWidget(self.inverse_btn)

        motor_buttons_layout = QHBoxLayout()
        self.motor_set_btn = QPushButton("Send Raw PWM")
        self.reset_btn = QPushButton("Kill/Reset Motors")
        motor_buttons_layout.addWidget(self.motor_set_btn)
        motor_buttons_layout.addWidget(self.reset_btn)

        ai_layout.addLayout(ai_buttons_layout)
        ai_layout.addLayout(motor_buttons_layout)
        ai_control_group.setLayout(ai_layout)

        # === Panel Input Data Spesifik ===
        specific_data_group = QGroupBox("Sensor Spoofing (Simulasi Data)")
        form_layout = QFormLayout()

        self.azimuth_input = QLineEdit()
        self.lat_direction_input = QLineEdit()
        self.long_direction_input = QLineEdit()

        form_layout.addRow("Azimuth:", self.azimuth_input)
        form_layout.addRow("Lat Direction:", self.lat_direction_input)
        form_layout.addRow("Long Direction:", self.long_direction_input)

        self.send_specific_data_btn = QPushButton("Inject Fake Sensor Data")
        form_layout.addRow(self.send_specific_data_btn)

        specific_data_group.setLayout(form_layout)

        # === [BARU] Panel Waypoint Simulator & Debugger ===
        wp_sim_group = QGroupBox("Waypoint Simulator & Debugger")
        wp_sim_layout = QVBoxLayout()

        # Total WP (Override nav_dist_to_wp)
        dist_layout = QHBoxLayout()
        dist_layout.addWidget(QLabel("Total Waypoints (Max):"))
        self.spin_sim_dist = QDoubleSpinBox()
        self.spin_sim_dist.setRange(1.0, 9999.0)
        self.spin_sim_dist.setDecimals(1)
        self.spin_sim_dist.setValue(18.0)  # Default 18 titik (0-17)
        dist_layout.addWidget(self.spin_sim_dist)

        self.btn_send_dist = QPushButton("Set Total WP")
        dist_layout.addWidget(self.btn_send_dist)
        wp_sim_layout.addLayout(dist_layout)

        # WP Index Override
        self.current_debug_wp = 0
        self.lbl_debug_wp = QLabel(self._get_wp_label_text())
        self.lbl_debug_wp.setStyleSheet("font-weight: bold;")
        wp_sim_layout.addWidget(self.lbl_debug_wp)

        wp_btn_layout = QHBoxLayout()
        self.btn_prev_wp = QPushButton("[-] Prev WP")
        self.btn_reset_wp = QPushButton("[Reset] WP 0")
        self.btn_next_wp = QPushButton("[+] Next WP")
        wp_btn_layout.addWidget(self.btn_prev_wp)
        wp_btn_layout.addWidget(self.btn_reset_wp)
        wp_btn_layout.addWidget(self.btn_next_wp)
        wp_sim_layout.addLayout(wp_btn_layout)

        wp_sim_group.setLayout(wp_sim_layout)

        # Tambahkan semua ke layout utama
        main_layout.addWidget(wp_sim_group)
        main_layout.addWidget(ai_control_group)
        main_layout.addWidget(specific_data_group)
        main_layout.addStretch()

        # Hubungkan tombol WP Simulator
        self.spin_sim_dist.valueChanged.connect(
            lambda: self.lbl_debug_wp.setText(self._get_wp_label_text())
        )
        self.btn_send_dist.clicked.connect(self._send_sim_dist)
        self.btn_prev_wp.clicked.connect(lambda: self._update_debug_wp(-1))
        self.btn_next_wp.clicked.connect(lambda: self._update_debug_wp(1))
        self.btn_reset_wp.clicked.connect(lambda: self._update_debug_wp(0, reset=True))

        # Hubungkan tombol ke fungsi yang memancarkan sinyal
        self.counter_plus_btn.clicked.connect(
            lambda: self.debug_command_sent.emit("DEBUG_WP_COUNTER", {"action": "INC"})
        )
        self.counter_min_btn.clicked.connect(
            lambda: self.debug_command_sent.emit("DEBUG_WP_COUNTER", {"action": "DEC"})
        )
        self.reset_btn.clicked.connect(
            lambda: self.debug_command_sent.emit(
                "DEBUG_WP_COUNTER", {"action": "RESET"}
            )
        )

        # Tombol lain (jika masih diperlukan, bisa dihubungkan nanti)
        self.inverse_btn.toggled.connect(
            lambda is_checked: self.debug_command_sent.emit(
                "SET_INVERSION", {"inverted": is_checked}
            )
        )
        self.motor_set_btn.clicked.connect(
            lambda: print("Tombol Motor Set ditekan (belum terhubung)")
        )

        self.send_specific_data_btn.clicked.connect(self.send_data)

    def send_data(self):
        """Mengumpulkan data dari input dan mengirimkannya."""
        data = {
            "azimuth": self.azimuth_input.text(),
            "lat_direction": self.lat_direction_input.text(),
            "long_direction": self.long_direction_input.text(),
        }
        self.debug_command_sent.emit("SPECIFIC_DATA", data)

    # --- SLOT HANDLERS WP SIMULATOR ---
    def _get_wp_label_text(self):
        max_wp = max(0, int(self.spin_sim_dist.value()) - 1)
        return f"Target WP: {self.current_debug_wp} / {max_wp}"

    def _send_sim_dist(self):
        dist = self.spin_sim_dist.value()
        self.debug_command_sent.emit("DEBUG_COMMAND", {"set_dist_to_wp": dist})

    def _update_debug_wp(self, delta, reset=False):
        max_wp = max(0, int(self.spin_sim_dist.value()) - 1)
        if reset:
            self.current_debug_wp = 0
        else:
            self.current_debug_wp += delta
            if self.current_debug_wp < 0:
                self.current_debug_wp = 0
            elif self.current_debug_wp > max_wp:
                self.current_debug_wp = max_wp

        self.lbl_debug_wp.setText(self._get_wp_label_text())
        self.debug_command_sent.emit(
            "DEBUG_COMMAND", {"set_wp_index": self.current_debug_wp}
        )
