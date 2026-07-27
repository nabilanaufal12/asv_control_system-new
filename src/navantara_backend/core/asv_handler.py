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
    photo_mission_target_wp1: int = -1
    photo_mission_target_wp2: int = -1
    photo_mission_qty_requested: int = 0
    photo_mission_qty_taken_1: int = 0
    photo_mission_qty_taken_2: int = 0
    vision_auto_motor_cmd: int = 1300
    vision_front_motor_left_cmd: int = 1500
    vision_front_motor_right_cmd: int = 1500
    vision_servo_left_cmd: int = 70
    vision_servo_right_cmd: int = 110


class AsvHandler:
    # [FIX YEL-02] Konstanta kelas untuk menggantikan magic numbers
    RC_MODE_SWITCH_THRESHOLD = 1500
    PWM_NEUTRAL = 1500
    FRONT_MOTOR_STOP = 1000
    AI_WP_RELEASE_DISTANCE_M = 1.5
    LOOP_INTERVAL_SEC = 0.02

    def __init__(self, config, socketio):
        self.config = config
        self.socketio = socketio
        self.serial_handler = SerialHandler(config)
        self.running = True
        self.state_lock = threading.Lock()
        self.is_streaming_to_gui = False
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 5.0

        # Baca nilai default AI dari konfigurasi (BARU)
        ai_cfg = self.config.get("ai_control_defaults", {})
        vision_motor_cmd = ai_cfg.get("vision_auto_motor_cmd", 1300)
        vision_servo_left = ai_cfg.get("vision_servo_left_cmd", 70)
        vision_servo_right = ai_cfg.get("vision_servo_right_cmd", 110)

        # Inisialisasi state dengan nilai dari konfigurasi AI.
        # Nilai lain (termasuk active_arena="B") akan menggunakan default dataclass.
        self.current_state = AsvState(
            vision_auto_motor_cmd=vision_motor_cmd,
            vision_servo_left_cmd=vision_servo_left,
            vision_servo_right_cmd=vision_servo_right,
        )

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

        # [FIX] Scan filesystem untuk menentukan race_id awal
        initial_race_id = self._scan_next_race_id() - 1  # -1 karena belum mulai race baru
        if initial_race_id < 1:
            initial_race_id = 1
        self.current_state.current_race_id = initial_race_id

        self.logger = MissionLogger(race_id=initial_race_id)
        self.is_logging_csv = False
        self.custom_csv_headers = ["Day", "Date", "Time", "GPS", "SOG", "COG", "HDG"]

        # [FIX] Track previous ESP mode untuk edge-triggered race creation
        self._previous_esp_mode = None

        self.logger.log_event("AsvHandler diinisialisasi.")
        logging.info(f"[AsvHandler] Handler diinisialisasi. Initial Race ID: {initial_race_id}")
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
                    if line_str and (line_str.startswith("ACK:") or line_str.startswith("[") or " " in line_str):
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
            self.current_state.latitude = data.get("lat", self.current_state.latitude)
            self.current_state.longitude = data.get("lon", self.current_state.longitude)
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

                # 1. Normalisasi Status Arena (Fix Bug "ARENA" contains "A")
                # [FIX RED-01] Gunakan variabel lokal, bukan self.current_state
                raw_arena_val = str(active_arena or "").strip().upper()
                is_arena_b = False
                # Cek apakah string diakhiri dengan B atau sama dengan B
                if raw_arena_val == "B" or raw_arena_val.endswith("_B"):
                    is_arena_b = True

                # 2. Tentukan Index Waypoint Efektif
                # [FIX RED-01] Gunakan variabel lokal, bukan self.current_state
                if use_dummy_counter:
                    current_effective_index = debug_waypoint_counter
                else:
                    current_effective_index = current_waypoint_index

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
                # -----------------------------------------------------------

                command_to_send = None

                if rc_mode_switch < self.RC_MODE_SWITCH_THRESHOLD:
                    command_to_send = None
                    logging.info("[AsvHandler] RC OVERRIDE -> Kontrol Jetson ditahan.")

                elif control_mode == "MANUAL":
                    # Tambahkan 1500, 1500 untuk mematikan motor depan saat mode manual
                    command_to_send = f"A,{int(manual_servo_cmd)},{int(manual_motor_cmd)},{self.PWM_NEUTRAL},{self.PWM_NEUTRAL}\n"
                    logging.info(
                        f"[AsvHandler] MANUAL CONTROL -> Servo: {int(manual_servo_cmd)} deg, Motor: {int(manual_motor_cmd)} us"
                    )

                elif control_mode == "AUTO":
                    mission_completed = bool(
                        waypoints and current_waypoint_index >= len(waypoints)
                    )
                    if mission_completed:
                        command_to_send = "W\n"

                    if vision_target_active:
                        with self.state_lock:
                            self.current_state.recovering_from_avoidance = False
                            self.current_state.is_avoiding = True
                            self.current_state.gate_context["last_gate_config"] = None

                            # Ambil kecepatan motor belakang dari GUI
                            current_ai_pwm = self.current_state.vision_auto_motor_cmd
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

                        # Ambil nilai servo dan motor depan dari GUI secara realtime
                        with self.state_lock:
                            pwm_depan_kiri_aktif = (
                                self.current_state.vision_front_motor_left_cmd
                            )
                            pwm_depan_kanan_aktif = (
                                self.current_state.vision_front_motor_right_cmd
                            )
                            servo_kiri_aktif = self.current_state.vision_servo_left_cmd
                            servo_kanan_aktif = (
                                self.current_state.vision_servo_right_cmd
                            )

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
                            # Kirim 5 parameter: A, Servo (Selalu 90), Motor Belakang, Motor Kiri Depan, Motor Kanan Depan
                            command_to_send = f"A,{servo_cmd},{int(pwm_cmd)},{motor_depan_kiri},{motor_depan_kanan}\n"
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
            "UPDATE_VISION_SPEED": self._handle_update_vision_speed,
            "UPDATE_VISION_FRONT_MOTOR": self._handle_update_vision_front_motor,
            "UPDATE_VISION_SERVO": self._handle_update_vision_servo,
            "DEBUG_WP_COUNTER": self._handle_debug_counter,
            "INVERSE_SERVO": self._handle_inverse_servo,
            "SET_INVERSION": self._handle_set_inversion,
            "SET_PHOTO_MISSION": self._handle_set_photo_mission,
            "UPDATE_INVERSION_TRIGGER": self._handle_update_inversion_trigger,
            "TOGGLE_LOGGING": self._handle_toggle_csv_logging,
            "MANUAL_CAPTURE": self._handle_manual_capture,
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

    def _handle_update_vision_speed(self, payload):
        try:
            pwm_val = int(payload.get("pwm", 1500))
            # Batasi safety range di sisi backend juga
            pwm_val = max(1300, min(1800, pwm_val))

            with self.state_lock:
                self.current_state.vision_auto_motor_cmd = pwm_val

            logging.info(f"[AsvHandler] Kecepatan AI Vision diupdate ke PWM: {pwm_val}")
        except ValueError:
            logging.warning("[AsvHandler] Payload PWM tidak valid untuk vision speed")

    def _handle_update_vision_front_motor(self, payload):
        try:
            left_val = int(payload.get("left", 1650))
            right_val = int(payload.get("right", 1650))
            with self.state_lock:
                self.current_state.vision_front_motor_left_cmd = left_val
                self.current_state.vision_front_motor_right_cmd = right_val
            logging.info(
                f"[AsvHandler] Motor Depan AI Updated -> Kiri: {left_val}, Kanan: {right_val}"
            )
        except ValueError:
            logging.warning("[AsvHandler] Payload PWM motor depan tidak valid")

    def _handle_mode_change(self, payload):
        """Memproses permintaan perubahan mode (MANUAL/AUTO)."""
        mode = payload.get("mode")
        if mode in ["MANUAL", "AUTO"]:
            cmd = f"M,{mode}\n"
            self.serial_handler.send_command(cmd)
            logging.info(f"[AsvHandler] Command '{mode}' diteruskan ke serial.")
            self.socketio.emit("log_message", {"text": f"[System] Mode changed to {mode}"})

    def _handle_manual_control(self, payload):
        with self.state_lock:
            if payload.get("toggle"):
                self.current_state.inverse_servo = not self.current_state.inverse_servo
            elif "value" in payload:
                self.current_state.inverse_servo = bool(payload["value"])

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

    def _handle_vision_target_update(self, payload):
        with self.state_lock:
            was_active = self.current_state.vision_target.get("active")
            is_active = payload.get("active")
            if was_active and not is_active:
                logging.info("[AsvHandler] Deteksi selesai -> langsung ke waypoint")
                self.current_state.recovering_from_avoidance = False
                self.current_state.is_avoiding = False
                self.current_state.avoidance_direction = None
                self.current_state.resume_waypoint_on_clear = True
                self.current_state.gate_context["last_gate_config"] = None

            self.current_state.vision_target["active"] = is_active
            if is_active:
                self.current_state.vision_target.update(payload)

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
        else:
            logging.error(f"[AsvHandler] Capture Gagal: {result.get('message')}")

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
            if hasattr(self, 'serial_handler') and self.serial_handler:
                self.serial_handler.send_data(tuning_cmd)
                logging.info(f"[AsvHandler] Dikirim ke Serial ESP32: {tuning_cmd.strip()}")

    def _handle_update_servo(self, payload):
        left = payload.get("left_angle")
        right = payload.get("right_angle")
        if left is not None and right is not None:
            tuning_cmd = f"T,SRV,{left},{right}\n"
            if hasattr(self, 'serial_handler') and self.serial_handler:
                self.serial_handler.send_data(tuning_cmd)
                logging.info(f"[AsvHandler] Dikirim ke Serial ESP32: {tuning_cmd.strip()}")

    def _handle_update_thruster(self, payload):
        speed = payload.get("speed")
        if speed is not None:
            tuning_cmd = f"T,THR,{speed}\n"
            if hasattr(self, 'serial_handler') and self.serial_handler:
                self.serial_handler.send_data(tuning_cmd)
                logging.info(f"[AsvHandler] Dikirim ke Serial ESP32: {tuning_cmd.strip()}")

    def _handle_serial_configuration(self, payload):
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO":
            self.serial_handler.find_and_connect_esp32(baud)
        else:
            self.serial_handler.connect(port, baud)

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
            command_str = f"A,{int(servo_def)},{int(pwm_stop)},{self.PWM_NEUTRAL},{self.PWM_NEUTRAL}\n"
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

    def _handle_set_photo_mission(self, payload):
        try:
            # [FIX] GUI mengirimkan indeks 0-based, langsung gunakan tanpa pengurangan
            wp1 = int(payload.get("wp1", -1))
            wp2 = int(payload.get("wp2", -1))
            count = int(payload.get("count", 0))

            with self.state_lock:
                self.current_state.photo_mission_target_wp1 = wp1  # Start Index (0-based)
                self.current_state.photo_mission_target_wp2 = wp2  # Stop Index (0-based)
                self.current_state.photo_mission_qty_requested = count
                # Reset counter
                self.current_state.photo_mission_qty_taken_1 = (
                    0  # Kita pakai ini sebagai counter utama
                )
                self.current_state.photo_mission_qty_taken_2 = (
                    0  # Tidak dipakai di mode segmen
                )

            logging.info(
                f"[AsvHandler] Misi Segmen Foto diatur: Start Index={wp1}, Stop Index={wp2}, Max={count} foto."
            )
            self.logger.log_event(f"Misi Foto Segmen: Index {wp1}-{wp2}, max {count}.")

        except Exception as e:
            logging.warning(
                f"[AsvHandler] Gagal mengatur Misi Foto: {e}. Payload: {payload}"
            )

    def stop(self):
        """Menghentikan handler, memutus serial, dan menutup logger."""
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        logging.info("[AsvHandler] Dihentikan.")
