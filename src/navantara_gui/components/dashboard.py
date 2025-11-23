# src/navantara_gui/components/dashboard.py
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QFrame,
)
from PySide6.QtCore import Qt


class Dashboard(QWidget):
    def __init__(self, config=None):
        super().__init__()
        self.config = config
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        # --- Status Cards ---
        status_layout = QHBoxLayout()
        self.status_card = self.create_card("Status", "DISCONNECTED")
        self.mode_card = self.create_card("Mode", "MANUAL")
        self.battery_card = self.create_card("Battery", "0.0 V")
        self.gps_card = self.create_card("GPS Sats", "0")

        status_layout.addWidget(self.status_card)
        status_layout.addWidget(self.mode_card)
        status_layout.addWidget(self.battery_card)
        status_layout.addWidget(self.gps_card)
        main_layout.addLayout(status_layout)

        # --- Navigation Info ---
        nav_grid = QGridLayout()

        self.lat_label = self.create_info_label("Latitude", "0.000000")
        self.lon_label = self.create_info_label("Longitude", "0.000000")
        self.heading_label = self.create_info_label("Heading", "0.0°")
        self.speed_label = self.create_info_label("Speed", "0.0 knots")

        self.wp_idx_label = self.create_info_label("Next WP", "-")
        self.dist_label = self.create_info_label("Dist to WP", "0.0 m")
        self.brg_label = self.create_info_label("Tgt Bearing", "0.0°")
        self.err_label = self.create_info_label("Hdg Error", "0.0°")

        nav_grid.addWidget(self.lat_label, 0, 0)
        nav_grid.addWidget(self.lon_label, 0, 1)
        nav_grid.addWidget(self.heading_label, 1, 0)
        nav_grid.addWidget(self.speed_label, 1, 1)

        nav_grid.addWidget(self.create_separator(), 2, 0, 1, 2)

        nav_grid.addWidget(self.wp_idx_label, 3, 0)
        nav_grid.addWidget(self.dist_label, 3, 1)
        nav_grid.addWidget(self.brg_label, 4, 0)
        nav_grid.addWidget(self.err_label, 4, 1)

        main_layout.addLayout(nav_grid)
        main_layout.addStretch()

    def create_card(self, title, value):
        frame = QFrame()
        frame.setStyleSheet(
            "background-color: #2d2d2d; border-radius: 5px; padding: 5px;"
        )
        layout = QVBoxLayout(frame)
        title_lbl = QLabel(title)
        title_lbl.setStyleSheet("color: #aaaaaa; font-size: 10px;")
        val_lbl = QLabel(value)
        val_lbl.setStyleSheet("color: white; font-size: 14px; font-weight: bold;")
        val_lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_lbl)
        layout.addWidget(val_lbl)
        frame.value_label = val_lbl
        return frame

    def create_info_label(self, title, value):
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        t = QLabel(f"{title}:")
        t.setStyleSheet("color: #aaaaaa;")
        v = QLabel(value)
        v.setStyleSheet("color: white; font-weight: bold;")
        v.setAlignment(Qt.AlignRight)
        layout.addWidget(t)
        layout.addWidget(v)
        widget.value_label = v
        return widget

    def create_separator(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: #444;")
        return line

    def update_data(self, data):
        """
        Update GUI dengan data telemetri.
        Mendukung key 'SHORT' (Optimized) dan 'LONG' (Legacy).
        """

        # 1. Helper untuk ambil data (Coba Short Key dulu, lalu Long Key, lalu Default)
        def get_val(short_k, long_k, default):
            return data.get(short_k, data.get(long_k, default))

        # 2. Ekstrak Data
        lat = get_val("lat", "latitude", 0.0)
        lon = get_val("lon", "longitude", 0.0)
        hdg = get_val("hdg", "heading", 0.0)
        spd = get_val("sog", "speed", 0.0)
        bat = get_val("bat", "battery_voltage", 0.0)
        sats = get_val("sat", "nav_gps_sats", 0)

        status = get_val("sts", "status", "DISCONNECTED")
        mode = get_val("mode", "control_mode", "MANUAL")

        # Waypoint Info
        wp_idx = get_val("wp_idx", "nav_target_wp_index", 0)
        wp_dst = get_val("wp_dst", "nav_dist_to_wp", 0.0)
        tgt_brg = get_val("tgt_brg", "nav_target_bearing", 0.0)
        err_hdg = get_val("err_hdg", "nav_heading_error", 0.0)

        # 3. Update UI Labels
        self.status_card.value_label.setText(str(status))
        self.mode_card.value_label.setText(str(mode))
        self.battery_card.value_label.setText(f"{float(bat):.1f} V")
        self.gps_card.value_label.setText(str(sats))

        self.lat_label.value_label.setText(f"{float(lat):.6f}")
        self.lon_label.value_label.setText(f"{float(lon):.6f}")
        self.heading_label.value_label.setText(f"{float(hdg):.1f}°")
        self.speed_label.value_label.setText(f"{float(spd):.1f} kn")

        self.wp_idx_label.value_label.setText(str(wp_idx))
        self.dist_label.value_label.setText(f"{float(wp_dst):.1f} m")
        self.brg_label.value_label.setText(f"{float(tgt_brg):.0f}°")
        self.err_label.value_label.setText(f"{float(err_hdg):.1f}°")
