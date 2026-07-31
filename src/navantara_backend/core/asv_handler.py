# src/navantara_backend/core/asv_handler.py
import threading
import time
import re
import numpy as np
import json
import logging
from dataclasses import dataclass, asdict, field

from navantara_backend.services.serial_service import SerialHandler

# [FIX] Hapus 'run_navigation_logic' dan 'haversine_distance' yang tidak dipakai
from navantara_backend.core.navigation import PIDController
from navantara_backend.core.kalman_filter import SimpleEKF
from navantara_backend.core.mission_logger import MissionLogger

# --- [OPTIMASI KEY MINIFICATION: MAPPING DICTIONARY] ---
# Format: "Nama Atribut Class AsvState" -> "Kode Pendek JSON"
TELEMETRY_KEY_MAP = {
    "latitude": "lat",
    "longitude": "lon",
    "heading": "hdg",
    "speed": "sog",  # Speed Over Ground
    "battery_voltage": "bat",
    "status": "sts",
    "control_mode": "mode",
    "active_arena": "ar",
    "inverse_servo": "inv",
    # Navigation & Waypoints
    "waypoints": "wps",
    "current_waypoint_index": "cur_wp",
    "nav_target_wp_index": "wp_idx",
    "nav_dist_to_wp": "wp_dst",
    "nav_heading_error": "err_hdg",
    "nav_target_bearing": "tgt_brg",
    "nav_gps_sats": "sat",
    # Actuators
    "nav_servo_cmd": "srv",
    "nav_motor_cmd": "mot",
    "manual_servo_cmd": "m_srv",
    "manual_motor_cmd": "m_mot",
    # Debug & Misc
    "mission_time": "time",
    "rc_channels": "rc",
    "is_connected_to_serial": "conn",
    "use_dummy_counter": "dum",
    "debug_waypoint_counter": "dbg_cnt",
    "vision_target": "vis",
    "is_docking": "dock",
    "esp_status": "esp_sts",
    "current_race_id": "r_id",
}
# --- [AKHIR OPTIMASI] ---


@dataclass
class AsvState:
    control_mode: str = "AUTO"
    current_race_id: int = 1
    latitude: float = -6.9180
    longitude: float = 107.6185
    heading: float = 90.0
    cog: float = 0.0
    speed: float = 0.0
    battery_voltage: float = 12.5
    status: str = "DISCONNECTED"
    mission_time: str = "00:00:00"
    waypoints: list = field(default_factory=list)
    current_waypoint_index: int = 0
    is_connected_to_serial: bool = False
    gyro_z: float = 0.0
    accel_x: float = 0.0
    rc_channels: list = field(default_factory=lambda: [1500] * 6)
    nav_target_wp_index: int = 0
    nav_dist_to_wp: float = 0.0  # coba 9999.0
    nav_target_bearing: float = 0.0
    nav_heading_error: float = 0.0
    nav_servo_cmd: int = 90
    nav_motor_cmd: int = 1500
    nav_gps_sats: int = 0
    manual_servo_cmd: int = 90
    manual_motor_cmd: int = 1500
    inversion_trigger_wp: int = 8  # Wp 6
    inverse_servo: bool = False
    active_arena: str = "B"
    debug_waypoint_counter: int = 0
    use_dummy_counter: bool = False
    debug_mode_enabled: bool = False
    esp_status: str = None
    vision_target: dict = field(default_factory=lambda: {"active": False})
    gate_target: dict = field(default_factory=lambda: {"active": False})
    avoidance_direction: str = None
    is_avoiding: bool = False
    gate_context: dict = field(default_factory=lambda: {"last_gate_config": None})
    recovering_from_avoidance: bool = False
    last_avoidance_time: float = 0.0
    last_pixel_error: float = 0.0
    resume_waypoint_on_clear: bool = False
    photo_mission_qty_requested: int = 0
    photo_mission_qty_taken_blue_surface: int = 0
    photo_mission_qty_taken_blue_underwater: int = 0
    photo_mission_qty_taken_green_surface: int = 0
    photo_mission_qty_taken_green_underwater: int = 0

    # Loitering State
    is_loitering: bool = False
    loiter_start_time: float = 0.0

    # Reverse Maneuver State
    is_reversing: bool = False
    reverse_start_time: float = 0.0

    # --- [BARU] Docking State ---
    is_docking: bool = False
    is_docking_completed: bool = False
    docking_start_time: float = 0.0

    # AI Photo Centering State
    is_photo_centering: bool = False
    photo_pixel_error: float = 0.0


class AsvHandler:
    # [FIX YEL-02] Konstanta kelas untuk menggantikan magic numbers
    RC_MODE_SWITCH_THRESHOLD = 1500
    PWM_NEUTRAL = 1500
    FRONT_MOTOR_STOP = 1000
    AI_WP_RELEASE_DISTANCE_M = 1.5
    LOOP_INTERVAL_SEC = 0.02

    def __init__(self, config, socketio):
        self.config = config
        self.mission_config = self.config.get("auto_missions", {})
        self.socketio = socketio

        # --- [NEW] Route Python Logs to GUI ---
        class SocketIOLogHandler(logging.Handler):
            def __init__(self, sio):
                super().__init__()
                self.sio = sio

            def emit(self, record):
                try:
                    msg = self.format(record)
                    self.sio.emit("log_message", {"text": msg})
                except Exception:
                    pass

        self.gui_log_handler = SocketIOLogHandler(self.socketio)
        formatter = logging.Formatter("%(levelname)s: %(message)s")
        self.gui_log_handler.setFormatter(formatter)
        logging.getLogger().addHandler(self.gui_log_handler)
        # --------------------------------------
        self.serial_handler = SerialHandler(config)
        self.running = True
        self.state_lock = threading.Lock()
        self.is_streaming_to_gui = False
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 5.0

        # Inisialisasi state default
        self.current_state = AsvState()

        # Dictionary ini menyimpan value terakhir berdasarkan NAMA ASLI (long key)
        # agar logika deteksi perubahan (delta) tetap konsisten.
        self.last_emitted_state = {}

        pid_config = self.config.get("navigation", {}).get("heading_pid", {})
        self.pid_controller = PIDController(
            Kp=pid_config.get("kp", 1.2),
            Ki=pid_config.get("ki", 0.1),
            Kd=pid_config.get("kd", 0.05),
        )
        self.ekf = SimpleEKF(np.zeros(5), np.eye(5) * 0.1)
        self.last_ekf_update_time = time.time()
        self.use_dummy_serial = self.config.get("serial_connection", {}).get(
            "use_dummy_serial", False
        )

        # [FIX] Scan filesystem untuk menentukan race_id awal, buat sesi race baru
        initial_race_id = self._scan_next_race_id()
        if initial_race_id < 1:
            initial_race_id = 1
        self.current_state.current_race_id = initial_race_id

        self.logger = MissionLogger(race_id=initial_race_id)
        self.is_logging_csv = False
        self.custom_csv_headers = ["Day", "Date", "Time", "GPS", "SOG", "COG", "HDG"]

        # [FIX] Track previous ESP mode untuk edge-triggered race creation
        self._previous_esp_mode = None

        self.logger.log_event("AsvHandler diinisialisasi.")
        logging.info(
            f"[AsvHandler] Handler diinisialisasi. Initial Race ID: {initial_race_id}"
        )
        self.initiate_auto_connection()

    def initiate_auto_connection(self):
        """Mencoba koneksi serial otomatis ke ESP32 atau fallback ke mode dummy."""
        serial_cfg = self.config.get("serial_connection", {})
        force_port = serial_cfg.get("force_serial_port")
        baud_rate = serial_cfg.get("default_baud_rate", 115200)

        if force_port:
            self.use_dummy_serial = False
            self.serial_handler.use_dummy_serial = False
            logging.info(
                f"[AsvHandler] Memaksa koneksi serial ke: {force_port} @ {baud_rate}"
            )
            connected = self.serial_handler.connect(force_port, baud_rate)
            if not connected:
                logging.warning(
                    f"[AsvHandler] Gagal terhubung ke port paksa {force_port}. Melanjutkan auto-scan..."
                )

        if self.use_dummy_serial:
            logging.info("[AsvHandler] Mode DUMMY SERIAL aktif.")
            return

        logging.info("[AsvHandler] Memulai upaya koneksi serial otomatis...")
        self.serial_handler.find_and_connect_esp32(baud_rate)

    # --- [MODIFIKASI: UPDATE AND EMIT DENGAN MINIFICATION] ---
    def _update_and_emit_state(self):
        """Menghitung status kapal, membuat delta payload, dan mengirimnya ke GUI via SocketIO."""
        if not (self.running and self.is_streaming_to_gui):
            return

        delta_payload = {}
        processed_status = "DISCONNECTED"

        with self.state_lock:
            is_serial_connected = self.serial_handler.is_connected
            control_mode = self.current_state.control_mode
            vision_target_active = self.current_state.vision_target.get("active", False)
            recovering = self.current_state.recovering_from_avoidance
            esp_status = self.current_state.esp_status
            waypoints = self.current_state.waypoints
            nav_target_wp_index = self.current_state.nav_target_wp_index
            rc_mode_switch = self.current_state.rc_channels[4]
            active_arena = self.current_state.active_arena
            is_inverted = self.current_state.inverse_servo

            # Update status koneksi internal
            self.current_state.is_connected_to_serial = is_serial_connected

            # --- LOGIKA STATUS STRING ---
            if not is_serial_connected:
                processed_status = "DISCONNECTED (SERIAL)"
            elif rc_mode_switch < self.RC_MODE_SWITCH_THRESHOLD:
                processed_status = "RC MANUAL OVERRIDE"
            elif vision_target_active:
                inv_label = "INV" if is_inverted else "NRM"
                processed_status = f"AI: AVOID ({active_arena}|{inv_label})"
            elif recovering:
                processed_status = "RECOVERING: STRAIGHTENING COURSE"
            elif control_mode == "AUTO":
                total_wps = len(waypoints)
                if esp_status == "WAYPOINT":
                    processed_status = (
                        f"WAYPOINT NAVIGATION ({nav_target_wp_index}/{total_wps})"
                    )
                elif esp_status == "WP_COMPLETE":
                    processed_status = "MISI SELESAI"
                elif esp_status == "NO_WAYPOINTS":
                    processed_status = "AUTO IDLE (NO WPs)"
                elif esp_status == "GPS_INVALID":
                    processed_status = "AUTO IDLE (NO GPS)"
                else:
                    processed_status = "AUTO IDLE"
            else:
                processed_status = control_mode

            self.current_state.status = processed_status

            # --- BUILD DELTA PAYLOAD & MINIFY ---
            current_state_dict = asdict(self.current_state)

            for key, value in current_state_dict.items():
                # 1. Cek apakah data berubah dibandingkan emisi terakhir
                # Menggunakan key ASLI untuk tracking state internal
                if (
                    key not in self.last_emitted_state
                    or self.last_emitted_state[key] != value
                ):
                    # 2. Jika berubah, ambil SHORT CODE dari mapping
                    short_key = TELEMETRY_KEY_MAP.get(
                        key, key
                    )  # Default ke key asli jika tidak ada di map

                    # 3. Masukkan ke payload dengan key pendek
                    delta_payload[short_key] = value

                    # 4. Update tracking state dengan key asli dan value baru
                    self.last_emitted_state[key] = value

        if delta_payload:
            self.socketio.emit("telemetry_update", delta_payload)

    def _read_from_serial_loop(self):
        """
        Loop pembacaan serial yang dioptimalkan untuk latency rendah (Burst Read).
        """
        while self.running:
            # 1. Loop internal: Baca dan proses SEMUA baris yang tersedia di buffer
            #    tanpa melakukan sleep di antaranya. Ini mengosongkan antrian secepat kilat.
            data_processed_in_burst = False

            while True:
                line = self.serial_handler.read_line()
                if not line:
                    break  # Buffer kosong, keluar dari burst loop

                data_processed_in_burst = True

                # Proses data secepat mungkin
                # Logging RAW bisa dimatikan jika beban CPU terlalu tinggi
                # logging.info(f"[Serial RAW] {line}")

                try:
                    data = json.loads(line)
                    # [FIX KRITIS] Pastikan data adalah dictionary (JSON Object) sebelum diproses
                    if isinstance(data, dict):
                        self._parse_json_telemetry(data)
                    # Jika data adalah integer/string tunggal (seperti '1'), ia akan diabaikan dengan aman
                except json.JSONDecodeError:
                    line_str = line.strip()
                    if line_str and (
                        line_str.startswith("ACK:")
                        or line_str.startswith("[")
                        or " " in line_str
                    ):
                        # Asumsikan ini adalah pesan log/debug dari ESP32
                        self.socketio.emit("log_message", {"text": line_str})
                    pass
                # [FIX RED-03] Tangkap hanya exception spesifik dari parsing,
                # bukan 'Exception' yang terlalu luas dan menyembunyikan bug.
                except (KeyError, TypeError, AttributeError, ValueError) as e:
                    logging.warning(f"[Serial] Error parsing: {e}")

            # 2. Manajemen Sleep Cerdas
            if data_processed_in_burst:
                # Jika kita baru saja memproses data, ada kemungkinan data baru
                # segera datang. Yield sangat singkat (0) agar OS tetap responsif
                # tapi prioritas tinggi.
                self.socketio.sleep(0)
            else:
                # Jika tidak ada data, tidur sangat sebentar (1ms) untuk hemat CPU.
                # Mengurangi dari 0.01 (10ms) ke 0.001 (1ms) meningkatkan poll rate ke 1000Hz
                self.socketio.sleep(0.001)

    def _parse_json_telemetry(self, data):
        """Mem-parsing data telemetri JSON dari ESP32 dan memperbarui state kapal."""
        with self.state_lock:
            self.current_state.heading = data.get("heading", self.current_state.heading)
            self.current_state.speed = data.get("speed_kmh", 0.0) / 3.6
            self.current_state.nav_gps_sats = data.get(
                "sats", self.current_state.nav_gps_sats
            )
            if not self.current_state.debug_mode_enabled:
                self.current_state.latitude = data.get(
                    "lat", self.current_state.latitude
                )
                self.current_state.longitude = data.get(
                    "lon", self.current_state.longitude
                )
            status_val = data.get("status", None)
            self.current_state.esp_status = status_val

            self.current_state.rc_channels = data.get(
                "rc_ch", self.current_state.rc_channels
            )

            # [FIX] Sinkronisasi mode ESP -> Backend state (two-way sync)
            esp_mode = data.get("mode")
            if esp_mode and esp_mode in ("MANUAL", "AUTO"):
                previous_mode = self.current_state.control_mode

                # Update control_mode dari ESP
                if previous_mode != esp_mode:
                    self.current_state.control_mode = esp_mode
                    logging.info(
                        f"[AsvHandler] Mode disinkronkan dari ESP: {previous_mode} -> {esp_mode}"
                    )

                # [FIX] Edge-triggered race creation: MANUAL -> AUTO
                if self._previous_esp_mode == "MANUAL" and esp_mode == "AUTO":
                    new_race_id = self._scan_next_race_id()
                    self.current_state.current_race_id = new_race_id
                    logging.info(
                        f"[AsvHandler] ESP MANUAL->AUTO transition. New Race ID: {new_race_id}"
                    )

                self._previous_esp_mode = esp_mode

            mode = data.get("mode")
            if mode == "MANUAL":
                # [FIX RED-04] Fallback ke state saat ini agar tidak None
                self.current_state.manual_servo_cmd = data.get(
                    "servo_out", self.current_state.manual_servo_cmd
                )
                self.current_state.manual_motor_cmd = data.get(
                    "motor_out", self.current_state.manual_motor_cmd
                )
            elif mode == "AUTO":
                status = data.get("status")
                if status == "WAYPOINT":
                    # [FIX RED-04] Fallback ke state saat ini agar tidak None
                    if (
                        not self.current_state.use_dummy_counter
                        and not self.current_state.debug_mode_enabled
                    ):
                        self.current_state.nav_target_wp_index = data.get(
                            "wp_target_idx", self.current_state.nav_target_wp_index
                        )
                        self.current_state.nav_dist_to_wp = data.get(
                            "wp_dist_m", self.current_state.nav_dist_to_wp
                        )
                    self.current_state.nav_target_bearing = data.get(
                        "wp_target_brg", self.current_state.nav_target_bearing
                    )
                    self.current_state.nav_heading_error = data.get(
                        "wp_error_hdg", self.current_state.nav_heading_error
                    )
                    self.current_state.nav_servo_cmd = data.get(
                        "servo_out", self.current_state.nav_servo_cmd
                    )
                    self.current_state.nav_motor_cmd = data.get(
                        "motor_out", self.current_state.nav_motor_cmd
                    )
                elif status == "AI_ACTIVE":
                    # [FIX RED-04] Fallback ke state saat ini agar tidak None
                    self.current_state.nav_servo_cmd = data.get(
                        "servo_out", self.current_state.nav_servo_cmd
                    )
                    self.current_state.nav_motor_cmd = data.get(
                        "motor_out", self.current_state.nav_motor_cmd
                    )

    def main_logic_loop(self):
        """Loop utama kontrol kapal: EKF update, inversi servo, AI avoidance, dan pengiriman serial."""
        self.socketio.start_background_task(self._read_from_serial_loop)
        logging.info("[AsvHandler] Loop pembaca serial dimulai.")
        while self.running:
            try:
                current_time = time.time()
                with self.state_lock:
                    is_auto = self.current_state.control_mode == "AUTO"

                if (
                    is_auto
                    and not self.serial_handler.is_connected
                    and current_time - self.last_reconnect_attempt
                    > self.reconnect_interval
                ):
                    self.last_reconnect_attempt = current_time
                    logging.info(
                        "[AsvHandler] Mode AUTO aktif, mencoba koneksi ulang ke ESP32..."
                    )
                    baud_rate = self.config.get("serial_connection", {}).get(
                        "default_baud_rate", 115200
                    )
                    self.serial_handler.find_and_connect_esp32(baud_rate)

                dt = current_time - self.last_ekf_update_time
                if dt > 0:
                    self.last_ekf_update_time = current_time
                    with self.state_lock:
                        heading_rad = np.radians(self.current_state.heading)
                        speed_ms = self.current_state.speed
                        gyro_z_rad = np.radians(self.current_state.gyro_z)
                    self.ekf.predict(dt)
                    self.ekf.update_compass(heading_rad)
                    self.ekf.update_imu(np.array([speed_ms, gyro_z_rad]))
                    with self.state_lock:
                        if not self.serial_handler.is_connected:
                            self.current_state.heading = (
                                np.degrees(self.ekf.state[2]) + 360
                            ) % 360

                # [FIX RED-01] Baca SEMUA variabel state dalam satu blok lock
                # agar pembacaan konsisten dan tidak ada race condition.
                with self.state_lock:
                    rc_mode_switch = self.current_state.rc_channels[4]
                    control_mode = self.current_state.control_mode
                    manual_servo_cmd = self.current_state.manual_servo_cmd
                    manual_motor_cmd = self.current_state.manual_motor_cmd

                    waypoints = self.current_state.waypoints
                    current_waypoint_index = self.current_state.current_waypoint_index

                    vision_target_active = self.current_state.vision_target.get(
                        "active", False
                    )
                    vision_target_obj_class = self.current_state.vision_target.get(
                        "obstacle_class", ""
                    )

                    resume_waypoint_on_clear = (
                        self.current_state.resume_waypoint_on_clear
                    )
                    nav_dist_to_wp = self.current_state.nav_dist_to_wp
                    esp_status = self.current_state.esp_status
                    status = self.current_state.status

                    # [FIX RED-01] Variabel inversi sekarang dibaca di dalam lock
                    active_arena = self.current_state.active_arena
                    use_dummy_counter = self.current_state.use_dummy_counter
                    debug_waypoint_counter = self.current_state.debug_waypoint_counter
                    inversion_trigger_wp = self.current_state.inversion_trigger_wp

                actuator_config = self.config.get("actuators", {})
                servo_default = actuator_config.get("servo_default_angle", 90)

                # -----------------------------------------------------------
                # [FIX INVERSI SERVO] LOGIKA DETEKSI TRIGGER WAYPOINT
                # -----------------------------------------------------------

                ENABLE_WP_INVERSION_2025 = False
                
                # Tentukan Index Waypoint Efektif (digunakan secara global oleh Misi Foto dll)
                # [FIX RED-01] Gunakan variabel lokal, bukan self.current_state
                if use_dummy_counter:
                    current_effective_index = debug_waypoint_counter
                else:
                    current_effective_index = current_waypoint_index

                if ENABLE_WP_INVERSION_2025:
                    # 1. Normalisasi Status Arena (Fix Bug "ARENA" contains "A")
                    # [FIX RED-01] Gunakan variabel lokal, bukan self.current_state
                    raw_arena_val = str(active_arena or "").strip().upper()
                    is_arena_b = False
                    # Cek apakah string diakhiri dengan B atau sama dengan B
                    if raw_arena_val == "B" or raw_arena_val.endswith("_B"):
                        is_arena_b = True

                    # 3. Logika Trigger Dinamis
                    # [FIX RED-01] Gunakan variabel lokal, bukan self.current_state
                    trigger_threshold_index = inversion_trigger_wp

                    # Trigger aktif jika kita SEDANG MENUJU atau SUDAH LEWAT waypoint trigger
                    # Misal Trigger WP 6 (index 5). Saat current_index = 5, artinya kita OTW ke WP 6.
                    is_wp_triggered = current_effective_index >= trigger_threshold_index

                    # 4. Kalkulasi XOR (Exclusive OR)
                    # Arena A (0) ^ Triggered (0) = 0 (Normal)
                    # Arena A (0) ^ Triggered (1) = 1 (Inverted) -> Misal mau inversi di akhir lintasan A
                    # Arena B (1) ^ Triggered (0) = 1 (Inverted Awal)
                    # Arena B (1) ^ Triggered (1) = 0 (Normal Kembali)
                    final_inversion_state = is_arena_b ^ is_wp_triggered

                    # 5. Update State
                    with self.state_lock:
                        if self.current_state.inverse_servo != final_inversion_state:
                            self.current_state.inverse_servo = final_inversion_state

                            mode_lbl = "INVERTED" if final_inversion_state else "NORMAL"
                            arena_lbl = "B" if is_arena_b else "A"
                            logging.info(
                                f"[Logic Inversi] CHANGE -> {mode_lbl} (Arena={arena_lbl}, CurrWP={current_effective_index}, TrigWP={trigger_threshold_index+1})"
                            )
                else:
                    final_inversion_state = False
                    # 2026 Rules: No inversion logic. Fallback to normal mode.
                    with self.state_lock:
                        if self.current_state.inverse_servo:
                            self.current_state.inverse_servo = False
                            logging.info(
                                "[Logic Inversi] Dinonaktifkan (2026 Rules) -> NORMAL"
                            )
                # -----------------------------------------------------------

                command_to_send = None

                if rc_mode_switch < self.RC_MODE_SWITCH_THRESHOLD:
                    command_to_send = None
                    logging.info("[AsvHandler] RC OVERRIDE -> Kontrol Jetson ditahan.")

                elif control_mode == "MANUAL":
                    # Format 8 parameter: A, Servo, DirB, PwmB, DirFL, PwmFL, DirFR, PwmFR
                    command_to_send = f"A,{int(manual_servo_cmd)},2000,{int(manual_motor_cmd)},2000,{int(manual_motor_cmd)},2000,{int(manual_motor_cmd)}\n"
                    logging.info(
                        f"[AsvHandler] MANUAL CONTROL -> Servo: {int(manual_servo_cmd)} deg, Motor: {int(manual_motor_cmd)} us"
                    )

                elif control_mode == "AUTO":
                    mission_completed = bool(
                        waypoints and current_waypoint_index >= len(waypoints)
                    )

                    # --- [BARU] DOCKING MISSION LOGIC ---
                    dock_cfg = self.mission_config.get("docking", {})
                    trigger_wp = dock_cfg.get("trigger_wp_index", 17)
                    if dock_cfg.get("enabled", False) and current_effective_index >= trigger_wp and nav_dist_to_wp < 3.0:
                        if not self.current_state.is_docking_completed:
                            if not self.current_state.is_docking:
                                with self.state_lock:
                                    self.current_state.is_docking = True
                                    self.current_state.docking_start_time = time.time()
                                logging.info(f"[AsvHandler] DOCKING MISSION TERPICU (WP {current_effective_index}, Dist: {nav_dist_to_wp:.2f}m)")

                    # --- [BARU] FOTOGRAFI, LOITERING, & VISUAL SERVOING ---
                    is_in_photo_zone = False
                    active_photo_cfg = None
                    active_photo_target = None
                    photo_master_cfg = self.mission_config.get("photography", {})
                    blue_cfg = photo_master_cfg.get("target_blue_box", {})
                    green_cfg = photo_master_cfg.get("target_green_box", {})

                    if (
                        blue_cfg.get("wp_start", -1)
                        <= current_effective_index
                        <= blue_cfg.get("wp_stop", -1)
                        and blue_cfg.get("wp_start", -1) != -1
                    ):
                        is_in_photo_zone = True
                        active_photo_cfg = blue_cfg
                        active_photo_target = "kotak-biru"
                    elif (
                        green_cfg.get("wp_start", -1)
                        <= current_effective_index
                        <= green_cfg.get("wp_stop", -1)
                        and green_cfg.get("wp_start", -1) != -1
                    ):
                        is_in_photo_zone = True
                        active_photo_cfg = green_cfg
                        active_photo_target = "kotak-hijau"

                    # Cek Stop & Wait (Loitering) & Reverse Maneuver
                    if is_in_photo_zone and active_photo_cfg:
                        stop_idx = active_photo_cfg.get("wp_stop", -1)
                        if (
                            current_effective_index == stop_idx
                            and nav_dist_to_wp < 3.0
                        ):  # 3.0m toleransi
                            if not self.current_state.is_loitering and not self.current_state.is_reversing:
                                self.current_state.is_loitering = True
                                self.current_state.loiter_start_time = time.time()
                                logging.info(
                                    f"[AsvHandler] Mulai Loitering di WP {stop_idx} untuk foto {active_photo_target}."
                                )

                    if self.current_state.is_docking:
                        elapsed = time.time() - self.current_state.docking_start_time
                        arena_id = self.current_state.active_arena
                        active_dock_cfg = dock_cfg.get(arena_id, dock_cfg.get("Arena_A", {}))
                        duration = active_dock_cfg.get("duration_sec", 20.0)
                        
                        if elapsed < duration:
                            s_ang = active_dock_cfg.get("servo_angle", 90)
                            r_pwm = active_dock_cfg.get("rear_pwm", 1000)
                            fl_dir = active_dock_cfg.get("front_left_dir", 2000)
                            fl_pwm = active_dock_cfg.get("front_left_pwm", 1000)
                            fr_dir = active_dock_cfg.get("front_right_dir", 2000)
                            fr_pwm = active_dock_cfg.get("front_right_pwm", 1000)
                            command_to_send = f"A,{int(s_ang)},2000,{int(r_pwm)},{int(fl_dir)},{int(fl_pwm)},{int(fr_dir)},{int(fr_pwm)}\n"
                        else:
                            with self.state_lock:
                                self.current_state.is_docking = False
                                self.current_state.is_docking_completed = True
                                self.current_state.control_mode = "MANUAL"
                                self.current_state.manual_servo_cmd = 90
                                self.current_state.manual_motor_cmd = 1000
                            
                            # Beritahu ESP32 bahwa kita pindah ke MANUAL agar ESP tidak meng-override balik ke AUTO
                            if self.serial_handler and self.serial_handler.is_connected:
                                self.serial_handler.send_command("M,MANUAL\n")
                                
                            logging.info("[AsvHandler] DOCKING SELESAI. Motor Dimatikan. Beralih ke mode MANUAL (STOP).")
                            command_to_send = f"A,90,2000,1000,2000,1000,2000,1000\n"

                    elif self.current_state.is_loitering:
                        elapsed = time.time() - self.current_state.loiter_start_time
                        delay_req = (
                            active_photo_cfg.get("stop_delay_sec", 3.0)
                            if active_photo_cfg
                            else 3.0
                        )
                        if elapsed < delay_req:
                            # Tahan motor (1000), kemudi netral (90), dir forward (2000)
                            command_to_send = (
                                f"A,90,2000,1000,2000,1000,2000,1000\n"
                            )
                        else:
                            with self.state_lock:
                                self.current_state.is_loitering = False
                                self.current_state.is_reversing = True
                                self.current_state.reverse_start_time = time.time()

                            # Trigger final photo
                            if hasattr(self, "vision_service"):
                                self.vision_service.handle_photography_mission(
                                    self.current_state, mode="surface"
                                )

                            logging.info(
                                "[AsvHandler] Selesai Loitering (Foto Final diambil), memulai Reverse Maneuver."
                            )

                    elif self.current_state.is_reversing:
                        elapsed = time.time() - self.current_state.reverse_start_time
                        rev_delay_req = (
                            active_photo_cfg.get("reverse_delay_sec", 2.0)
                            if active_photo_cfg
                            else 2.0
                        )
                        rev_pwm = (
                            active_photo_cfg.get("reverse_speed_pwm", 1300)
                            if active_photo_cfg
                            else 1300
                        )

                        if elapsed < rev_delay_req:
                            # Mundur: Dir=1000, Speed=rev_pwm
                            command_to_send = (
                                f"A,90,1000,{int(rev_pwm)},1000,{int(rev_pwm)},1000,{int(rev_pwm)}\n"
                            )
                        else:
                            with self.state_lock:
                                self.current_state.is_reversing = False
                            logging.info(
                                "[AsvHandler] Selesai Reverse Maneuver, melanjutkan misi."
                            )
                            command_to_send = "W\n"

                    elif mission_completed:
                        command_to_send = "W\n"

                    # Jika dalam zona foto, override PWM (Speed limit) & apply Visual Servoing
                    elif is_in_photo_zone:
                        target_pwm = active_photo_cfg.get("speed_pwm", 1400)

                        # Cek apakah sedang melihat target foto
                        if (
                            self.current_state.is_photo_centering
                            and self.current_state.vision_target.get("active")
                        ):
                            pixel_error = self.current_state.photo_pixel_error
                            Kp = 0.05  # Proportional gain
                            servo_cmd = 90 + (pixel_error * Kp)
                            servo_cmd = max(45, min(135, servo_cmd))  # Clamp
                            logging.info(
                                f"[AsvHandler] AI Centering ({active_photo_target}): Error={pixel_error:.1f}, Servo={servo_cmd:.1f}"
                            )
                        else:
                            # Jika tidak ada target, pakai kemudi GPS dari telemetry
                            servo_cmd = self.current_state.nav_servo_cmd

                        command_to_send = f"A,{int(servo_cmd)},2000,{int(target_pwm)},2000,{int(target_pwm)},2000,{int(target_pwm)}\n"

                    elif vision_target_active:
                        with self.state_lock:
                            self.current_state.recovering_from_avoidance = False
                            self.current_state.is_avoiding = True
                            self.current_state.gate_context["last_gate_config"] = None

                            profile = self._get_active_vision_profile()
                            # Ambil kecepatan motor belakang dari GUI
                            current_ai_pwm = profile["pwm_utama"]
                            # Kita TIDAK butuh nilai servo dari GUI karena akan di-set selalu 90

                        obj_class = vision_target_obj_class
                        turn_direction = "STRAIGHT"
                        desc = "Neutral"

                        # 1. TENTUKAN ARAH MENGHINDAR (Berdasarkan Warna Objek & Inversi)
                        if final_inversion_state:
                            # --- ARENA B (INVERTED) ---
                            if obj_class in ["bola-hijau", "kotak-hijau"]:
                                turn_direction = "RIGHT"
                                desc = f"{obj_class} -> Menghindar Kanan (INV)"
                            elif obj_class in ["bola-merah", "kotak-biru"]:
                                turn_direction = "LEFT"
                                desc = f"{obj_class} -> Menghindar Kiri (INV)"
                            elif obj_class == "bola-biru":
                                turn_direction = "STRAIGHT"
                                desc = f"{obj_class} -> Target Docking (Lurus)"
                        else:
                            # --- ARENA A (NORMAL) ---
                            if obj_class in ["bola-hijau", "kotak-hijau"]:
                                turn_direction = "LEFT"
                                desc = f"{obj_class} -> Menghindar Kiri (NRM)"
                            elif obj_class in ["bola-merah", "kotak-biru"]:
                                turn_direction = "RIGHT"
                                desc = f"{obj_class} -> Menghindar Kanan (NRM)"
                            elif obj_class == "bola-biru":
                                turn_direction = "STRAIGHT"
                                desc = f"{obj_class} -> Target Docking (Lurus)"

                        # 2. LOGIKA KEMUDI (SERVO BELAKANG & MOTOR DEPAN)
                        # Ambil tenaga motor belakang dari GUI
                        pwm_cmd = current_ai_pwm

                        motor_depan_kiri = self.FRONT_MOTOR_STOP
                        motor_depan_kanan = self.FRONT_MOTOR_STOP

                        # Ambil nilai servo dan motor depan dari profil misi secara realtime
                        pwm_depan_kiri_aktif = profile["pwm_kiri"]
                        pwm_depan_kanan_aktif = profile["pwm_kanan"]
                        servo_kiri_aktif = profile["angle_left"]
                        servo_kanan_aktif = profile["angle_right"]

                        if turn_direction == "LEFT":
                            # Belok Kiri: Menggunakan settingan servo kiri dari GUI
                            servo_cmd = servo_kiri_aktif
                            motor_depan_kanan = pwm_depan_kanan_aktif
                            desc += f" | Belok KIRI (Servo: {servo_kiri_aktif}, M.Kanan: {pwm_depan_kanan_aktif})"

                        elif turn_direction == "RIGHT":
                            # Belok Kanan: Menggunakan settingan servo kanan dari GUI
                            servo_cmd = servo_kanan_aktif
                            motor_depan_kiri = pwm_depan_kiri_aktif
                            desc += f" | Belok KANAN (Servo: {servo_kanan_aktif}, M.Kiri: {pwm_depan_kiri_aktif})"

                        else:
                            # Lurus: Servo Netral
                            servo_cmd = servo_default
                            desc += f" | LURUS (Servo: {servo_default})"

                        # 4. EKSEKUSI PENGIRIMAN SERIAL
                        if nav_dist_to_wp < self.AI_WP_RELEASE_DISTANCE_M:
                            command_to_send = "W\n"
                            logging.info(
                                f"[AsvHandler] AI STATIC: Jarak WP < {self.AI_WP_RELEASE_DISTANCE_M}m. Melepas ke Waypoint Nav."
                            )
                        elif esp_status == "WP_COMPLETE" or status == "WP_COMPLETE":
                            command_to_send = "W\n"
                            logging.info(
                                "[AsvHandler] WP_COMPLETE dilaporkan -> mengirim W"
                            )
                        else:
                            # Kirim 8 parameter: A, Servo, DirB, PwmB, DirFL, PwmFL, DirFR, PwmFR
                            command_to_send = f"A,{servo_cmd},2000,{int(pwm_cmd)},2000,{motor_depan_kiri},2000,{motor_depan_kanan}\n"
                            logging.info(
                                f"[LOGIC DEBUG] AI ACTIVE | Motor Bawah: {int(pwm_cmd)} | Action: {desc}"
                            )

                    elif resume_waypoint_on_clear:
                        with self.state_lock:
                            self.current_state.is_avoiding = False
                            self.current_state.avoidance_direction = None
                            self.current_state.recovering_from_avoidance = False
                            self.current_state.resume_waypoint_on_clear = False
                        command_to_send = "W\n"
                        logging.info("[AsvHandler] Transisi cepat -> waypoint mode")

                    else:
                        with self.state_lock:
                            self.current_state.last_pixel_error = 0
                        if self.serial_handler.is_connected:
                            command_to_send = "W\n"
                            logging.info("[AsvHandler] WAYPOINT CONTROL -> Mengirim: W")
                        else:
                            command_to_send = None
                            logging.info(
                                "[AsvHandler] WAYPOINT CONTROL -> Menunggu koneksi serial..."
                            )

                if control_mode == "AUTO":
                    if esp_status == "WP_COMPLETE" or status in (
                        "WP_COMPLETE",
                        "MISI SELESAI",
                    ):
                        if command_to_send is None or (
                            isinstance(command_to_send, str)
                            and not command_to_send.strip().startswith("W")
                        ):
                            command_to_send = "W\n"
                            logging.info(
                                "[AsvHandler] Mission complete detected -> forcing W"
                            )

                if command_to_send is None and resume_waypoint_on_clear:
                    if self.serial_handler.is_connected and control_mode == "AUTO":
                        command_to_send = "W\n"
                        logging.info("[AsvHandler] Vision cleared -> sending resume W")
                        with self.state_lock:
                            self.current_state.resume_waypoint_on_clear = False

                if command_to_send:
                    self.serial_handler.send_command(command_to_send)

                # --- BAGIAN LOGGING (Disederhanakan) ---
                with self.state_lock:
                    state_for_log = asdict(self.current_state)

                # Panggil satu baris ini saja. Logger akan otomatis format ke Day/Date/GPS/dll.
                self.logger.log_telemetry(state_for_log)

                # Update SocketIO (GUI)
                self._update_and_emit_state()
                # ----------------------------------------

            except Exception as e:
                logging.error(f"[FATAL] Error di main_logic_loop: {e}", exc_info=True)

            self.socketio.sleep(self.LOOP_INTERVAL_SEC)

    def process_command(self, command, payload):
        """Mendistribusikan perintah dari GUI/SocketIO ke handler yang sesuai."""
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control,
            "SET_WAYPOINTS": self._handle_set_waypoints,
            "NAV_START": self._handle_start_mission,
            "NAV_RETURN": self._handle_initiate_rth,
            "UPDATE_PID": self._handle_update_pid,
            "UPDATE_SERVO": self._handle_update_servo,
            "UPDATE_THRUSTER": self._handle_update_thruster,
            "VISION_TARGET_UPDATE": self._handle_vision_target_update,
            "UPDATE_VISION_SERVO": self._handle_update_vision_servo,
            "DEBUG_WP_COUNTER": self._handle_debug_counter,
            "DEBUG_COMMAND": self._handle_debug_command,
            "INVERSE_SERVO": self._handle_inverse_servo,
            "SET_INVERSION": self._handle_set_inversion,
            "UPDATE_INVERSION_TRIGGER": self._handle_update_inversion_trigger,
            "TOGGLE_LOGGING": self._handle_toggle_csv_logging,
            "MANUAL_CAPTURE": self._handle_manual_capture,
            "SWAP_CAMERAS": self._handle_swap_cameras,
            "UPDATE_MISSION_CONFIG": self._handle_update_mission_config,
        }
        handler = command_handlers.get(command)
        if handler:
            handler(payload)
        else:
            logging.warning(
                f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'"
            )

    def _handle_toggle_csv_logging(self, payload):
        """Mengaktifkan/mematikan log CSV kustom user."""
        status = payload.get("status", False)

        if status:
            if not self.is_logging_csv:
                # Start dengan header fix yang sudah ditentukan
                self.logger.start_csv_log(self.custom_csv_headers)
                self.is_logging_csv = True
                logging.info("[AsvHandler] Logging Data Kustom AKTIF.")
        else:
            if self.is_logging_csv:
                self.logger.stop_csv_log()
                self.is_logging_csv = False
                logging.info("[AsvHandler] Logging Data Kustom MATI.")

    def _handle_update_vision_servo(self, payload):
        try:
            left_val = int(payload.get("left", 45))
            right_val = int(payload.get("right", 135))

            # Validasi range sederhana
            left_val = max(0, min(90, left_val))
            right_val = max(90, min(180, right_val))

            with self.state_lock:
                self.current_state.vision_servo_left_cmd = left_val
                self.current_state.vision_servo_right_cmd = right_val

            logging.info(
                f"[AsvHandler] Servo Vision Updated -> Left: {left_val}, Right: {right_val}"
            )
        except ValueError:
            logging.warning("[AsvHandler] Payload servo tidak valid")

    def _handle_set_inversion(self, payload):
        with self.state_lock:
            new_state = payload.get("inverted", False)
            if self.current_state.inverse_servo != new_state:
                self.current_state.inverse_servo = new_state
                logging.info(f"[AsvHandler] Kontrol Inversi diatur ke: {new_state}")

    def _handle_inverse_servo(self, payload):
        with self.state_lock:
            if payload.get("toggle"):
                self.current_state.inverse_servo = not self.current_state.inverse_servo
            elif "value" in payload:
                self.current_state.inverse_servo = bool(payload["value"])
            logging.info(
                f"[AsvHandler] inverse_servo diubah ke: {self.current_state.inverse_servo}"
            )

    def _handle_debug_counter(self, payload):
        action = payload.get("action")
        with self.state_lock:
            self.current_state.use_dummy_counter = True
            if action == "INC":
                self.current_state.debug_waypoint_counter += 1
            elif action == "DEC":
                self.current_state.debug_waypoint_counter = max(
                    0, self.current_state.debug_waypoint_counter - 1
                )
            elif action == "RESET":
                self.current_state.debug_waypoint_counter = 0
            max_points_in_monitor = 9
            self.current_state.debug_waypoint_counter = min(
                self.current_state.debug_waypoint_counter, max_points_in_monitor
            )
            logging.info(
                f"[AsvHandler] Debug counter diatur ke: {self.current_state.debug_waypoint_counter}"
            )

    def _handle_debug_command(self, payload):
        with self.state_lock:
            # Cek apakah toggle mode debug dikirim
            if "debug_mode_enabled" in payload:
                self.current_state.debug_mode_enabled = bool(
                    payload["debug_mode_enabled"]
                )
                # Jika mode debug dimatikan, kembalikan ke mode normal
                if not self.current_state.debug_mode_enabled:
                    self.current_state.use_dummy_counter = False
                    logging.info("[AsvHandler] [SIM] Debug Mode DIMATIKAN")
                    return
                else:
                    self.current_state.use_dummy_counter = True
                    logging.info("[AsvHandler] [SIM] Debug Mode DIAKTIFKAN")

            # Set mode dummy HANYA JIKA debug mode aktif
            if self.current_state.debug_mode_enabled:
                self.current_state.use_dummy_counter = True

            if "set_wp_index" in payload:
                wp_idx = int(payload["set_wp_index"])
                self.current_state.current_waypoint_index = wp_idx
                self.current_state.debug_waypoint_counter = wp_idx  # Fix: sinkronkan counter dummy ke state
                # Sinkronkan juga nav_target_wp_index agar web monitoring ter-update
                self.current_state.nav_target_wp_index = wp_idx
                
                # Reset status misi khusus agar tidak "bocor" antar waypoint saat ditest ulang di mode debug
                self.current_state.is_docking = False
                self.current_state.is_docking_completed = False
                self.current_state.is_loitering = False
                self.current_state.is_reversing = False
                
                logging.info(f"[AsvHandler] [SIM] WP Index disetel ke: {wp_idx}")

                # Sinkronisasi koordinat dengan waypoint yang dimuat
                wps = self.current_state.waypoints
                if wps and len(wps) > 0:
                    # Pastikan index tidak melampaui list
                    safe_idx = min(wp_idx, len(wps) - 1)
                    target_wp = wps[safe_idx]
                    if isinstance(target_wp, dict):
                        self.current_state.latitude = float(
                            target_wp.get(
                                "lat",
                                target_wp.get("latitude", self.current_state.latitude),
                            )
                        )
                        self.current_state.longitude = float(
                            target_wp.get(
                                "lon",
                                target_wp.get(
                                    "longitude", self.current_state.longitude
                                ),
                            )
                        )
                    elif isinstance(target_wp, (list, tuple)) and len(target_wp) >= 2:
                        self.current_state.latitude = float(target_wp[0])
                        self.current_state.longitude = float(target_wp[1])
                    logging.info(
                        f"[AsvHandler] [SIM] Posisi disinkronkan ke WP {safe_idx}: "
                        f"({self.current_state.latitude}, {self.current_state.longitude})"
                    )

            if "set_dist_to_wp" in payload:
                self.current_state.nav_dist_to_wp = float(payload["set_dist_to_wp"])
                logging.info(
                    f"[AsvHandler] [SIM] nav_dist_to_wp disetel ke: "
                    f"{self.current_state.nav_dist_to_wp}"
                )

    def _handle_vision_target_update(self, payload):
        with self.state_lock:
            was_active = self.current_state.vision_target.get("active")
            is_active = payload.get("active")
            is_photo_centering = payload.get("is_photo_centering", False)

            if was_active and not is_active:
                logging.info("[AsvHandler] Deteksi selesai -> langsung ke waypoint")
                self.current_state.recovering_from_avoidance = False
                self.current_state.is_avoiding = False
                self.current_state.avoidance_direction = None
                self.current_state.resume_waypoint_on_clear = True
                self.current_state.gate_context["last_gate_config"] = None

            self.current_state.vision_target["active"] = is_active
            self.current_state.is_photo_centering = is_photo_centering

            if is_active:
                self.current_state.vision_target.update(payload)

                # Calculate pixel error for centering
                if is_photo_centering:
                    center_x = payload.get("object_center_x", 0)
                    frame_width = payload.get("frame_width", 1)
                    frame_center_x = frame_width / 2.0
                    self.current_state.photo_pixel_error = center_x - frame_center_x

    def _handle_manual_capture(self, payload):
        """
        Menangani perintah manual capture (Surface/Underwater)
        dengan opsi RAW (tanpa overlay).
        """
        # Pastikan vision_service sudah di-inject dari main.py
        if not hasattr(self, "vision_service"):
            logging.error("[AsvHandler] Gagal Capture: Vision Service belum terhubung.")
            return

        # --- [KODE YANG ANDA MINTA] ---
        capture_type = payload.get("type", "surface")
        is_raw = payload.get("raw", False)  # Ambil nilai boolean, default False

        # Panggil fungsi vision service dengan parameter baru
        result = self.vision_service.trigger_manual_capture(
            capture_type, raw_mode=is_raw, race_id=self.current_state.current_race_id
        )
        # ------------------------------

        # (Opsional) Log hasil untuk debugging
        if result.get("status") == "success":
            logging.info(
                f"[AsvHandler] Capture Berhasil: {result.get('file')} (Mode: {result.get('mode', 'Overlaid')})"
            )
            # Broadcast NEW_CAPTURE
            broadcast_payload = {
                "type": "NEW_CAPTURE",
                "data": {
                    "camera": capture_type,
                    "mode": "raw" if is_raw else "overlay",
                    "url": f"/captures/race_{self.current_state.current_race_id}/{result.get('file')}",
                },
            }
            self.socketio.emit("NEW_CAPTURE", broadcast_payload)
        else:
            logging.error(f"[AsvHandler] Capture Gagal: {result.get('message')}")

    def _handle_swap_cameras(self, payload):
        """Menukar sumber kamera Surface dan Underwater di VisionService."""
        if not hasattr(self, "vision_service"):
            logging.error("[AsvHandler] Gagal Swap: Vision Service belum terhubung.")
            return
        logging.info("[AsvHandler] Menerima perintah SWAP_CAMERAS.")
        self.vision_service.swap_cameras()

    def _handle_update_mission_config(self, payload):
        """Menerima konfigurasi parsial/penuh auto_missions dari GUI, simpan ke file, dan update RAM."""
        try:
            with self.state_lock:
                # Deep merge payload into self.mission_config
                for key, value in payload.items():
                    if key in self.mission_config and isinstance(value, dict) and isinstance(self.mission_config[key], dict):
                        self.mission_config[key].update(value)
                    else:
                        self.mission_config[key] = value

                self.config["auto_missions"] = self.mission_config
                
                # Update requested photo qty if available
                photo_cfg = self.mission_config.get("photography", {})
                if "max_photo_per_target" in photo_cfg:
                    self.current_state.photo_mission_qty_requested = int(photo_cfg["max_photo_per_target"])
                
                # Reset konter foto setiap kali save misi baru
                self.current_state.photo_mission_qty_taken_blue_surface = 0
                self.current_state.photo_mission_qty_taken_blue_underwater = 0
                self.current_state.photo_mission_qty_taken_green_surface = 0
                self.current_state.photo_mission_qty_taken_green_underwater = 0
                
            # Tulis ke file (aman karena hanya dijalankan saat user klik "Save" di GUI)
            with open("config/config.json", "w") as f:
                json.dump(self.config, f, indent=2)

            logging.info("[AsvHandler] Konfigurasi Misi (auto_missions) diperbarui dan disimpan.")
            self.logger.log_event("Konfigurasi Misi (auto_missions) di Hot-Reload.")
        except Exception as e:
            logging.warning(f"[AsvHandler] Gagal hot-reload konfigurasi Misi: {e}")

    def _get_active_vision_profile(self):
        """Mengembalikan profil aktif (bola atau kotak) berdasarkan waypoint saat ini."""
        wp = self.current_state.current_waypoint_index
        
        bola_cfg = self.mission_config.get("vision_ball_red_green", {})
        kotak_cfg = self.mission_config.get("vision_box_blue_green", {})

        # Cek apakah masuk profil kotak
        if (
            kotak_cfg.get("wp_start", 11)
            <= wp
            <= kotak_cfg.get("wp_end", 14)
        ):
            kotak_profile = kotak_cfg.copy()
            kotak_profile["profile_name"] = "kotak"
            kotak_profile["valid_classes"] = ["kotak-hijau", "kotak-biru"]
            return kotak_profile

        # Cek apakah masuk profil bola
        if (
            bola_cfg.get("wp_start", 0)
            <= wp
            <= bola_cfg.get("wp_end", 11)
        ):
            bola_profile = bola_cfg.copy()
            bola_profile["profile_name"] = "bola"
            bola_profile["valid_classes"] = ["bola-merah", "bola-hijau"]
            return bola_profile
            
        # Di luar rentang misi vision, matikan deteksi
        return {"profile_name": "none", "valid_classes": []}

    def _handle_manual_control(self, payload):
        """Menerjemahkan input keyboard (WASD) ke perintah servo dan motor."""
        with self.state_lock:
            rc_channel_5 = self.current_state.rc_channels[4]
            control_mode = self.current_state.control_mode
            is_inverted = self.current_state.inverse_servo

        if rc_channel_5 < self.RC_MODE_SWITCH_THRESHOLD:
            return
        if control_mode != "MANUAL":
            return

        keys, actuator_config = set(payload), self.config.get("actuators", {})
        pwm_stop, pwr = actuator_config.get(
            "motor_pwm_stop", 1500
        ), actuator_config.get("motor_pwm_manual_power", 150)
        servo_def, servo_min, servo_max = (
            actuator_config.get("servo_default_angle", 90),
            actuator_config.get("servo_min_angle", 45),
            actuator_config.get("servo_max_angle", 135),
        )
        fwd = 1 if "W" in keys else -1 if "S" in keys else 0
        turn = 1 if "D" in keys else -1 if "A" in keys else 0

        if is_inverted:
            turn = -turn

        pwm = pwm_stop + fwd * pwr
        servo = servo_def - turn * (servo_def - servo_min)
        servo = max(servo_min, min(servo_max, servo))

        with self.state_lock:
            self.current_state.manual_servo_cmd = int(servo)
            self.current_state.manual_motor_cmd = int(pwm)

    def set_streaming_status(self, status: bool):
        """Mengatur flag streaming telemetri ke GUI."""
        if self.is_streaming_to_gui != status:
            logging.info(f"[AsvHandler] Status streaming telemetri diatur ke: {status}")
            self.is_streaming_to_gui = status

    def _handle_update_pid(self, payload):
        kp, ki, kd = payload.get("p"), payload.get("i"), payload.get("d")
        if all(isinstance(val, (int, float)) for val in [kp, ki, kd]):
            self.pid_controller.Kp, self.pid_controller.Ki, self.pid_controller.Kd = (
                kp,
                ki,
                kd,
            )
            self.pid_controller.reset()
            logging.info(f"[AsvHandler] Local PID updated: P={kp}, I={ki}, D={kd}")

            # --- [TAMBAHAN BARU] Kirim ke Firmware ---
            # Format protokol: T,PID,<P>,<I>,<D>\n
            tuning_cmd = f"T,PID,{kp},{ki},{kd}\n"
            if hasattr(self, "serial_handler") and self.serial_handler:
                self.serial_handler.send_command(tuning_cmd)
                logging.info(
                    f"[AsvHandler] Dikirim ke Serial ESP32: {tuning_cmd.strip()}"
                )

    def _handle_update_servo(self, payload):
        left = payload.get("left_angle")
        right = payload.get("right_angle")
        if left is not None and right is not None:
            tuning_cmd = f"T,SRV,{left},{right}\n"
            if hasattr(self, "serial_handler") and self.serial_handler:
                self.serial_handler.send_command(tuning_cmd)
                logging.info(
                    f"[AsvHandler] Dikirim ke Serial ESP32: {tuning_cmd.strip()}"
                )

    def _handle_update_thruster(self, payload):
        speed = payload.get("speed")
        if speed is not None:
            tuning_cmd = f"T,THR,{speed}\n"
            if hasattr(self, "serial_handler") and self.serial_handler:
                self.serial_handler.send_command(tuning_cmd)
                logging.info(
                    f"[AsvHandler] Dikirim ke Serial ESP32: {tuning_cmd.strip()}"
                )

    def _handle_serial_configuration(self, payload):
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO":
            success = self.serial_handler.find_and_connect_esp32(baud)
        else:
            success = self.serial_handler.connect(port, baud)

        status_msg = "CONNECTED" if success else "DISCONNECTED"
        self.socketio.emit("CONNECTION_STATUS", {"status": status_msg})

    @staticmethod
    def _scan_next_race_id():
        """Scan filesystem untuk menentukan race_id berikutnya.
        Jika race_1, race_2, race_3 ada, return 4."""
        import os

        captures_dir = os.path.join(os.getcwd(), "logs", "captures")
        if not os.path.exists(captures_dir):
            return 1

        max_id = 0
        for d in os.listdir(captures_dir):
            match = re.match(r"^race_(\d+)$", d)
            if match and os.path.isdir(os.path.join(captures_dir, d)):
                race_num = int(match.group(1))
                if race_num > max_id:
                    max_id = race_num
        return max_id + 1

    def _handle_mode_change(self, payload):
        """Menangani perubahan mode dari GUI. Juga forward perintah ke ESP via serial."""
        with self.state_lock:
            current_mode = self.current_state.control_mode
            new_mode = payload.get("mode", "MANUAL")

            if current_mode == "MANUAL" and new_mode == "AUTO":
                # [FIX] Scan filesystem untuk race_id, bukan increment sederhana
                new_race_id = self._scan_next_race_id()
                self.current_state.current_race_id = new_race_id
                self.logger.start_new_race_log(new_race_id)
                logging.info(
                    f"[AsvHandler] GUI MANUAL->AUTO transition. New Race ID: {new_race_id}"
                )

            self.current_state.control_mode = new_mode
            self.logger.log_event(f"Mode kontrol GUI diubah ke: {new_mode}")

        # [FIX] Forward mode command ke ESP via serial: "M,AUTO\n" atau "M,MANUAL\n"
        mode_cmd = f"M,{new_mode}\n"
        self.serial_handler.send_command(mode_cmd)
        logging.info(f"[AsvHandler] Mode command forwarded to ESP: {mode_cmd.strip()}")

        if new_mode == "MANUAL":
            actuator_config = self.config.get("actuators", {})
            pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
            servo_def = actuator_config.get("servo_default_angle", 90)
            # Format Baru 8 param: A, Servo, DirB, PwmB, DirFL, PwmFL, DirFR, PwmFR
            command_str = f"A,{int(servo_def)},2000,{int(pwm_stop)},2000,{int(pwm_stop)},2000,{int(pwm_stop)}\n"
            logging.info(
                f"[LOG | MODE] GUI ganti ke MANUAL, kirim netral: {command_str.strip()}"
            )
            self.serial_handler.send_command(command_str)

    # [BARU] Handler untuk mengubah titik trigger inversi secara dinamis
    def _handle_update_inversion_trigger(self, payload):
        try:
            # [FIX] GUI sudah mengirim 0-based index, langsung gunakan tanpa dikurangi 1
            trigger_index = int(payload.get("index", 5))

            # Proteksi agar index tidak negatif
            trigger_index = max(0, trigger_index)

            with self.state_lock:
                self.current_state.inversion_trigger_wp = trigger_index

            logging.info(
                f"[AsvHandler] Inversion Trigger Updated: Index {trigger_index} (WP {trigger_index + 1})"
            )
            self.logger.log_event(f"Trigger Inversi diubah ke Index {trigger_index}")

        except ValueError:
            logging.warning("[AsvHandler] Payload inversion trigger tidak valid")

    def _handle_set_waypoints(self, payload):
        waypoints_data = payload.get("waypoints")
        raw_arena = payload.get("arena") or payload.get("arena_id")
        custom_trigger = payload.get("inversion_trigger_wp")

        # --- [FIX KRITIS DETEKSI ARENA] ---
        # Masalah Lama: Kata "ARENA" mengandung huruf "A", jadi logika lama gagal.
        arena_id = "Arena_A"  # Default
        if raw_arena:
            clean_arena = str(raw_arena).strip().upper().replace(" ", "_")

            # Logika deteksi yang lebih kuat
            # Menangkap: "B", "ARENA_B", "LINTASAN_B", "ARENA B"
            if (
                clean_arena == "B"
                or clean_arena.endswith("_B")
                or "ARENA_B" in clean_arena
            ):
                arena_id = "Arena_B"
            else:
                arena_id = "Arena_A"
        # ----------------------------------

        if not isinstance(waypoints_data, list):
            logging.warning("[AsvHandler] Gagal set waypoints: Data tidak valid.")
            return

        with self.state_lock:
            self.current_state.waypoints = waypoints_data
            # Reset index selalu ke 0 saat load misi baru
            self.current_state.current_waypoint_index = 0
            self.current_state.active_arena = arena_id

            # Jika GUI mengirim trigger khusus, pakai itu. Jika tidak, pertahankan yang ada.
            if custom_trigger is not None:
                self.current_state.inversion_trigger_wp = int(custom_trigger)

            self.logger.log_event(
                f"Waypoints dimuat (Arena: {arena_id}, Trigger Inversi WP: {self.current_state.inversion_trigger_wp + 1}). Jml: {len(waypoints_data)}"
            )
            logging.info(
                f"[Setup] Arena set to: {arena_id} (Raw: {raw_arena}) | Inversi Trigger Index: {self.current_state.inversion_trigger_wp}"
            )

    def _handle_start_mission(self, payload):
        with self.state_lock:
            if not self.current_state.waypoints:
                return
            
            # Jika user memulai misi auto kembali, reset status docking
            self.current_state.is_docking_completed = False
            self.current_state.is_docking = False
            
            self.current_state.control_mode = "AUTO"
            self.current_state.current_waypoint_index = 0
            self.current_state.use_dummy_counter = False
        self.logger.log_event("Misi navigasi dimulai.")

    def _handle_initiate_rth(self, payload):
        with self.state_lock:
            if not self.current_state.waypoints:
                return
            self.current_state.waypoints = [self.current_state.waypoints[0]]
            self.current_state.current_waypoint_index = 0
            self.current_state.control_mode = "AUTO"
        self.logger.log_event("Memulai Return to Home.")

    # _handle_set_photo_mission removed, merged into _handle_update_mission_config

    def stop(self):
        """Menghentikan handler, memutus serial, dan menutup logger."""
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        logging.info("[AsvHandler] Dihentikan.")
