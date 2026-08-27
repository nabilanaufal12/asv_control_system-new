# src/navantara_gui/components/dashboard.py
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QGridLayout,
    QFrame,
)

# [FIX] Baris import Qt dihapus karena tidak digunakan di implementasi baru
# from PySide6.QtCore import Qt


class NavigationStatusMonitor(QGroupBox):
    """
    Widget ringkas dengan layout Grid 2 Kolom untuk memantau
    6 metrik vital operasi ASV.
    """

    def __init__(self):
        super().__init__("Navigation & System Status")

        # Gunakan Grid Layout:
        # Kolom 0: Label Kiri, Kolom 1: Nilai Kiri
        # Kolom 2: Label Kanan, Kolom 3: Nilai Kanan
        self.layout = QGridLayout()
        self.layout.setVerticalSpacing(5)
        self.layout.setHorizontalSpacing(15)

        # --- Baris 1 ---
        # Kiri: Mode Kendali
        self.lbl_mode = QLabel("Mode:")
        self.val_mode = QLabel("UNKNOWN")
        self.val_mode.setStyleSheet("font-weight: bold; font-size: 14px;")

        # Kanan: GPS Sats
        self.lbl_sats = QLabel("GPS Sats:")
        self.val_sats = QLabel("0")

        # --- Baris 2 ---
        # Kiri: Target WP
        self.lbl_wp_idx = QLabel("Target WP:")
        self.val_wp_idx = QLabel("-")

        # Kanan: Jarak WP
        self.lbl_wp_dist = QLabel("Jarak WP:")
        self.val_wp_dist = QLabel("0.0 m")

        # --- Baris 3 ---
        # Kiri: Error Heading
        self.lbl_err_hdg = QLabel("Error Hdg:")
        self.val_err_hdg = QLabel("0.0°")
        self.val_err_hdg.setStyleSheet("font-weight: bold;")

        # Kanan: Vision Status
        self.lbl_vision = QLabel("Vision AI:")
        self.val_vision = QLabel("OFF")
        # --- Baris 4 ---
        # Kiri: XTE (Cross-Track Error)
        self.lbl_xte = QLabel("XTE (Dev):")
        self.val_xte = QLabel("0.0 m")
        self.val_xte.setStyleSheet("font-weight: bold;")

        # Kanan: Target Bearing
        self.lbl_tgt_brg = QLabel("Tgt Bearing:")
        self.val_tgt_brg = QLabel("0.0°")

        # Menambahkan ke Layout Grid (row, col)
        # Kolom Kiri (0, 1)
        self.layout.addWidget(self.lbl_mode, 0, 0)
        self.layout.addWidget(self.val_mode, 0, 1)

        self.layout.addWidget(self.lbl_wp_idx, 1, 0)
        self.layout.addWidget(self.val_wp_idx, 1, 1)

        self.layout.addWidget(self.lbl_err_hdg, 2, 0)
        self.layout.addWidget(self.val_err_hdg, 2, 1)

        self.layout.addWidget(self.lbl_xte, 3, 0)
        self.layout.addWidget(self.val_xte, 3, 1)

        # Separator vertikal
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)
        self.layout.addWidget(line, 0, 2, 4, 1)  # Span 4 baris

        # Kolom Kanan (3, 4) -- index 2 dipakai separator
        self.layout.addWidget(self.lbl_sats, 0, 3)
        self.layout.addWidget(self.val_sats, 0, 4)

        self.layout.addWidget(self.lbl_wp_dist, 1, 3)
        self.layout.addWidget(self.val_wp_dist, 1, 4)

        self.layout.addWidget(self.lbl_vision, 2, 3)
        self.layout.addWidget(self.val_vision, 2, 4)

        self.layout.addWidget(self.lbl_tgt_brg, 3, 3)
        self.layout.addWidget(self.val_tgt_brg, 3, 4)

        self.setLayout(self.layout)

    def update_status(self, data):
        """
        Memperbarui UI dengan logika fallback untuk key minification.
        """
        # 1. MODE KENDALI
        mode = data.get("control_mode", data.get("mode", "MANUAL"))
        self.val_mode.setText(str(mode).upper())

        if mode == "AUTO":
            self.val_mode.setStyleSheet(
                "font-weight: bold; color: #2ecc71; font-size: 14px;"
            )
        elif mode == "MANUAL":
            self.val_mode.setStyleSheet(
                "font-weight: bold; color: #e67e22; font-size: 14px;"
            )
        else:
            self.val_mode.setStyleSheet(
                "font-weight: bold; color: red; font-size: 14px;"
            )

        # 2. GPS SATS
        sats = data.get("nav_gps_sats", data.get("sat", 0))
        self.val_sats.setText(str(sats))
        if int(sats) < 4:
            self.val_sats.setStyleSheet("font-weight: bold; color: red;")
        else:
            self.val_sats.setStyleSheet("font-weight: bold; color: #2ecc71;")

        # 3. TARGET WP
        wp_idx = data.get(
            "current_waypoint_index", data.get("cur_wp", data.get("wp_idx", "-"))
        )
        wp_tot_raw = data.get("nav_esp_total_wp", data.get("wp_tot", 0))
        try:
            wp_tot_int = int(wp_tot_raw)
            wp_idx_int = int(wp_idx)
            max_idx = wp_tot_int - 1 if wp_tot_int > 0 else 0
            if wp_tot_int > 0 and wp_idx_int >= wp_tot_int:
                # Semua WP tercapai (counter == dataIndex di ESP32)
                self.val_wp_idx.setText(f"DONE ({wp_tot_int} WP)")
            else:
                self.val_wp_idx.setText(f"#{wp_idx_int} / {max_idx}")
        except (ValueError, TypeError):
            self.val_wp_idx.setText(f"#{wp_idx} / ?")

        # 4. JARAK WP
        dist = data.get("nav_dist_to_wp", data.get("wp_dst", 0.0))
        self.val_wp_dist.setText(f"{float(dist):.1f} m")

        # 5. ERROR HEADING
        err = data.get("nav_heading_error", data.get("err_hdg", 0.0))
        self.val_err_hdg.setText(f"{float(err):.1f}°")

        # 6. TARGET BEARING
        tgt_brg = data.get("nav_target_bearing", data.get("tgt_brg", 0.0))
        self.val_tgt_brg.setText(f"{float(tgt_brg):.1f}°")

        if abs(float(err)) > 20.0:
            self.val_err_hdg.setStyleSheet("font-weight: bold; color: #e74c3c;")
        else:
            self.val_err_hdg.setStyleSheet("font-weight: bold; color: #3498db;")

        # 6. VISION ACTIVE
        vis_data = data.get("vision_target", data.get("vis", {}))
        is_vis_active = vis_data.get("active", False)

        if is_vis_active:
            self.val_vision.setText("ACTIVE")
            self.val_vision.setStyleSheet("font-weight: bold; color: #2ecc71;")
        else:
            self.val_vision.setText("IDLE")
            self.val_vision.setStyleSheet("color: gray;")

        # 7. XTE
        xte = data.get("nav_xte_m", data.get("xte", 0.0))
        self.val_xte.setText(f"{float(xte):.2f} m")


class SensorDisplay(QGroupBox):
    """
    Menampilkan data telemetri sensor (Heading, COG, Speed, Lat, Lon, Fix/Hz).
    Didesain berdampingan dengan NavigationStatusMonitor di bawah Photo Capture Configuration.
    """

    def __init__(self):
        super().__init__("Telemetry Sensor Data")
        self.layout = QGridLayout()
        self.layout.setVerticalSpacing(5)
        self.layout.setHorizontalSpacing(15)
        self.layout.setContentsMargins(10, 10, 10, 10)

        # --- Baris 0 ---
        # Kiri: Heading
        self.lbl_hdg = QLabel("Heading:")
        self.val_heading = QLabel("90.0°")
        self.val_heading.setStyleSheet("font-size: 14px; font-weight: bold;")
        # Kanan: COG
        self.lbl_cog = QLabel("COG:")
        self.val_cog = QLabel("0.0°")
        self.val_cog.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: #3498db;"
        )

        # --- Baris 1 ---
        # Kiri: Speed
        self.lbl_spd = QLabel("Speed:")
        self.val_speed = QLabel("0.0 km/h")
        self.val_speed.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: #2ecc71;"
        )
        # Kanan: GPS Fix / Rate
        self.lbl_fix = QLabel("GPS Fix:")
        self.val_fix = QLabel("NO FIX")
        self.val_fix.setStyleSheet("font-weight: bold; color: #e67e22;")

        # --- Baris 2 ---
        # Kiri: Latitude
        self.lbl_lat = QLabel("Latitude:")
        self.val_lat = QLabel("0.000000")
        self.val_lat.setStyleSheet("font-weight: bold;")
        # Kanan: Longitude
        self.lbl_lon = QLabel("Longitude:")
        self.val_lon = QLabel("0.000000")
        self.val_lon.setStyleSheet("font-weight: bold;")

        # Baris 0
        self.layout.addWidget(self.lbl_hdg, 0, 0)
        self.layout.addWidget(self.val_heading, 0, 1)
        self.layout.addWidget(self.lbl_cog, 0, 2)
        self.layout.addWidget(self.val_cog, 0, 3)

        # Baris 1
        self.layout.addWidget(self.lbl_spd, 1, 0)
        self.layout.addWidget(self.val_speed, 1, 1)
        self.layout.addWidget(self.lbl_fix, 1, 2)
        self.layout.addWidget(self.val_fix, 1, 3)

        # Baris 2
        self.layout.addWidget(self.lbl_lat, 2, 0)
        self.layout.addWidget(self.val_lat, 2, 1)
        self.layout.addWidget(self.lbl_lon, 2, 2)
        self.layout.addWidget(self.val_lon, 2, 3)

        self.setLayout(self.layout)

    def update_sensor(self, data):
        hdg = data.get("heading", data.get("hdg", 0.0))
        cog_val = data.get("cog", 0.0)
        lat = data.get("latitude", data.get("lat", 0.0))
        lon = data.get("longitude", data.get("lon", 0.0))
        spd = data.get("speed", data.get("sog", 0.0))
        fix_val = data.get("esp_status", data.get("fix", "NO FIX"))
        hz_val = data.get("gps_hz", data.get("hz", 0.0))

        self.val_heading.setText(f"{float(hdg):.1f}°")
        self.val_cog.setText(f"{float(cog_val):.1f}°")
        self.val_lat.setText(f"{float(lat):.6f}")
        self.val_lon.setText(f"{float(lon):.6f}")
        self.val_speed.setText(f"{float(spd):.1f} km/h")

        if hz_val and float(hz_val) > 0:
            self.val_fix.setText(f"{fix_val} ({float(hz_val):.1f} Hz)")
        else:
            self.val_fix.setText(f"{fix_val}")

        if "FIX" in str(fix_val).upper() and "NO" not in str(fix_val).upper():
            self.val_fix.setStyleSheet("font-weight: bold; color: #2ecc71;")
        else:
            self.val_fix.setStyleSheet("font-weight: bold; color: #e67e22;")


class Dashboard(QWidget):
    """
    Panel utama Dashboard yang menggabungkan SensorDisplay
    dan NavigationStatusMonitor.
    """

    def __init__(self, config):
        super().__init__()
        self.config = config

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(5, 5, 5, 5)
        self.main_layout.setSpacing(10)

        self.sensor_display = SensorDisplay()
        self.status_monitor = NavigationStatusMonitor()
        self.main_layout.addWidget(self.sensor_display)
        self.main_layout.addWidget(self.status_monitor)

    def update_data(self, data):
        if not data:
            return
        self.sensor_display.update_sensor(data)
        self.status_monitor.update_status(data)
