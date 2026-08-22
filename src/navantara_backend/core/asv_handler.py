# src/navantara_backend/core/asv_handler.py
import threading
import time
import numpy as np
import json
import logging
import collections
import statistics
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
    "cog": "cog",
    "speed": "sog",  # Speed Over Ground
    "status": "sts",
    "control_mode": "mode",
    "active_arena": "ar",
    # Navigation & Waypoints
    "waypoints": "wps",
    "current_waypoint_index": "cur_wp",
    "nav_target_wp_index": "wp_idx",
    "nav_esp_total_wp": "wp_tot",
    "nav_dist_to_wp": "wp_dst",
    "nav_xte_m": "xte",
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
    "debug_waypoint_counter": "dbg_cnt",
    "vision_target": "vis",
    "esp_status": "esp_sts",
    "portrait_state": "p_st",
    "docking_state": "dk_st",
}
# --- [AKHIR OPTIMASI] ---


@dataclass
class AsvState:
    control_mode: str = "AUTO"
    latitude: float = 0.0
    longitude: float = 0.0
    heading: float = 90.0
    cog: float = 0.0
    speed: float = 0.0
    status: str = "DISCONNECTED"
    mission_time: str = "00:00:00"
    waypoints: list = field(default_factory=list)
    current_waypoint_index: int = 0
    is_connected_to_serial: bool = False
    rc_channels: list = field(default_factory=lambda: [1500] * 6)
    nav_target_wp_index: int = 0
    nav_esp_total_wp: int = 0
    nav_dist_to_wp: float = 0.0
    nav_xte_m: float = 0.0
    nav_target_bearing: float = 0.0
    nav_heading_error: float = 0.0
    nav_servo_cmd: int = 90
    nav_motor_cmd: int = 1500
    nav_gps_sats: int = 0
    manual_servo_cmd: int = 90
    manual_motor_cmd: int = 1500
    active_arena: str = "A"

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
    # Photo Mission Parameters
    photo_mission_surf_wp1: int = 13
    photo_mission_surf_wp2: int = 14
    photo_mission_under_wp1: int = 11
    photo_mission_under_wp2: int = 12
    photo_mission_qty_requested: int = 5
    photo_mission_interval: float = 2.0
    photo_mission_qty_taken_1: int = 0
    photo_mission_qty_taken_2: int = 0
    vision_auto_motor_cmd: int = 1300
    # Portrait Mission Config
    portrait_speed: int = 1400
    portrait_reverse_speed: int = 1400
    portrait_stop_ms: int = 3000
    portrait_reverse_ms: int = 2000
    portrait_state: int = 0  # 0=Normal, 1=Slow, 2=Stop, 3=Reverse
    vision_front_motor_cmd: int = 1500
    vision_servo_left_cmd: int = 70
    vision_servo_right_cmd: int = 110
    
    # Box Avoidance Parameters
    box_avoidance_distance: float = 165.0
    box_servo_left_cmd: int = 70
    box_servo_right_cmd: int = 110
    box_speed_cmd: int = 1500
    box_front_motor_cmd: int = 1800
    box_motor_pwm_cmd: int = 1800
    
    # Docking Parameters
    dock_motor_utama: int = 1200
    dock_motor_depan: int = 1400
    dock_charge_duration_ms: int = 3000
    dock_servo_left: int = 0
    dock_servo_right: int = 180
    docking_state: int = 0
    docking_enabled: bool = True  # [ON/OFF] Toggle aktif/nonaktif docking mission


class AsvHandler:
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

        # Buffer untuk menerima sinkronisasi waypoint dari ESP32
        self._sync_waypoints_buffer = []

        pid_config = self.config.get("navigation", {}).get("heading_pid", {})
        self.pid_controller = PIDController(
            Kp=pid_config.get("kp", 1.2),
            Ki=pid_config.get("ki", 0.1),
            Kd=pid_config.get("kd", 0.05),
        )
        self.ekf = SimpleEKF(np.zeros(5), np.eye(5) * 0.1)
        self.last_ekf_update_time = time.time()
        self.heading_history = collections.deque(
            maxlen=3
        )  # [FIX] Dikurangi menjadi 3 agar merespons instan
        self._ema_heading = None  # State EMA heading

        self.logger = MissionLogger()
        self.is_logging_csv = False
        self.custom_csv_headers = ["Day", "Date", "Time", "GPS", "SOG", "COG", "HDG"]
        self.logger.log_event("AsvHandler diinisialisasi.")
        logging.info("[AsvHandler] Handler diinisialisasi untuk operasi backend.")
        self.initiate_auto_connection()

    def initiate_auto_connection(self):
        serial_cfg = self.config.get("serial_connection", {})
        force_port = serial_cfg.get("force_serial_port")
        baud_rate = serial_cfg.get("default_baud_rate", 115200)

        if force_port:
            self.serial_handler.use_dummy_serial = False
            logging.info(
                f"[AsvHandler] Memaksa koneksi serial ke: {force_port} @ {baud_rate}"
            )
            connected = self.serial_handler.connect(force_port, baud_rate)
            if not connected:
                logging.warning(
                    f"[AsvHandler] Gagal terhubung ke port paksa {force_port}. Melanjutkan auto-scan..."
                )

        logging.info("[AsvHandler] Memulai upaya koneksi serial otomatis...")
        self.serial_handler.find_and_connect_esp32(baud_rate)

    # --- [MODIFIKASI: UPDATE AND EMIT DENGAN MINIFICATION] ---
    def _update_and_emit_state(self):
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

            # Update status koneksi internal
            self.current_state.is_connected_to_serial = is_serial_connected

            # --- LOGIKA STATUS STRING ---
            if not is_serial_connected:
                processed_status = "DISCONNECTED (SERIAL)"
            elif rc_mode_switch < 1500:
                processed_status = "RC MANUAL OVERRIDE"
            elif vision_target_active:
                processed_status = f"AI: AVOID ({active_arena})"
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
                elif esp_status == "DK_TURNING":
                    processed_status = "DOCKING: TURNING"
                elif esp_status == "DK_CHARGING":
                    processed_status = "DOCKING: CHARGING"
                elif esp_status == "DK_COMPLETE":
                    processed_status = "DOCKING SELESAI"
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

    # ... (Sisa kode di bawah ini sama persis dengan sebelumnya)

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

                # [FIX SYNC] Deteksi data sinkronisasi waypoint dari ESP32
                if line.startswith("SYNC_WP"):
                    if line == "SYNC_WP_START":
                        self._sync_waypoints_buffer = []
                        logging.info(
                            "[AsvHandler] Mulai menerima sync waypoint dari ESP32..."
                        )
                    elif line == "SYNC_WP_END":
                        with self.state_lock:
                            self.current_state.waypoints = list(
                                self._sync_waypoints_buffer
                            )

                        # [FIX] PENTING: Emit event ke GUI agar tabel diperbarui!
                        self.socketio.emit(
                            "sync_waypoints", self._sync_waypoints_buffer
                        )
                        logging.info(
                            f"[AsvHandler] Selesai sync waypoint. Total: {len(self.current_state.waypoints)}"
                        )
                    else:
                        parts = line.split(",")
                        if len(parts) >= 4:
                            try:
                                lat = float(parts[2])
                                lon = float(parts[3])
                                self._sync_waypoints_buffer.append(
                                    {"lat": lat, "lon": lon}
                                )
                            except ValueError:
                                pass
                    continue

                try:
                    data = json.loads(line)
                    # [FIX KRITIS] Pastikan data adalah dictionary (JSON Object) sebelum diproses
                    if isinstance(data, dict):
                        self._parse_json_telemetry(data)
                    # Jika data adalah integer/string tunggal (seperti '1'), ia akan diabaikan dengan aman
                except json.JSONDecodeError:
                    pass
                except Exception as e:
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
        try:
            with self.state_lock:
                raw_hdg = data.get("hdg")
                if raw_hdg is not None:
                    # [FIX-5] Matikan semua filter (Median & EMA) untuk kecepatan seketika (0 lag).
                    # Catatan: Jika ada loncatan EMI, biarkan EKF/Kalman Filter yang meratakannya.
                    self.current_state.heading = raw_hdg

                self.current_state.cog = data.get("cog", self.current_state.cog)
                self.current_state.speed = data.get("spd", 0.0) / 3.6
                self.current_state.nav_gps_sats = data.get(
                    "sat", self.current_state.nav_gps_sats
                )
                self.current_state.latitude = data.get(
                    "lat", self.current_state.latitude
                )
                self.current_state.longitude = data.get(
                    "lon", self.current_state.longitude
                )

                status_val = data.get("sts", None)
                if status_val:
                    self.current_state.esp_status = status_val

                self.current_state.rc_channels = data.get(
                    "rc", self.current_state.rc_channels
                )
                mode = data.get("mod")
                if mode:
                    # Cetak informasi di terminal jika mode berubah
                    if self.current_state.control_mode != mode:
                        print("\n======================================")
                        print(
                            f">>> MODE RC BERUBAH: {self.current_state.control_mode} -> {mode} <<<"
                        )
                        print("======================================\n")

                    # Deteksi transisi dari MANUAL ke AUTO untuk membuat folder race_x baru
                    if self.current_state.control_mode == "MANUAL" and mode == "AUTO":
                        if getattr(self, "_is_first_auto_switch", True):
                            logging.info(
                                "[AsvHandler] Mode berubah MANUAL -> AUTO pertama kali. Menggunakan race session yang sudah dibuat saat backend nyala."
                            )
                            self._is_first_auto_switch = False
                        else:
                            logging.info(
                                "[AsvHandler] Mode berubah MANUAL -> AUTO. Membuat Race Session baru."
                            )
                            self.logger.start_new_race()
                            
                        # Reset counter foto ketika memulai putaran AUTO baru
                        self.current_state.photo_mission_qty_taken_1 = 0
                        self.current_state.photo_mission_qty_taken_2 = 0

                    self.current_state.control_mode = mode

                if mode == "MANUAL":
                    self.current_state.manual_servo_cmd = data.get("srv")
                    self.current_state.manual_motor_cmd = data.get("mot")
                elif mode == "AUTO":
                    status = data.get("sts")
                    if status == "WAYPOINT":
                        self.current_state.nav_dist_to_wp = data.get("w_dst")
                        self.current_state.nav_xte_m = data.get("xte", 0.0)
                        self.current_state.nav_target_bearing = data.get("w_brg")
                        self.current_state.nav_heading_error = data.get("w_err")
                        self.current_state.nav_servo_cmd = data.get("srv")
                        self.current_state.nav_motor_cmd = data.get("mot")
                    elif status == "AI_ACTIVE":
                        self.current_state.nav_servo_cmd = data.get("srv")
                        self.current_state.nav_motor_cmd = data.get("mot")

                # Sinkronisasi WP Index selalu dilakukan di semua mode
                if "w_id" in data:
                    new_idx = data.get("w_id")
                    self.current_state.nav_target_wp_index = new_idx
                    self.current_state.current_waypoint_index = new_idx
                if "w_tot" in data:
                    self.current_state.nav_esp_total_wp = data.get("w_tot")

                if "p_st" in data:
                    self.current_state.portrait_state = data.get("p_st", 0)
                if "dk_st" in data:
                    self.current_state.docking_state = data.get("dk_st")
        except Exception as e:
            logging.error(
                f"[AsvHandler] Gagal mem-parsing data JSON: {e}. Data: {data}"
            )

    def main_logic_loop(self):
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
                        gyro_z_rad = 0.0  # Sensor gyro tidak terpasang, gunakan 0
                    self.ekf.predict(dt)
                    self.ekf.update_compass(heading_rad)
                    self.ekf.update_imu(np.array([speed_ms, gyro_z_rad]))
                    with self.state_lock:
                        if not self.serial_handler.is_connected:
                            self.current_state.heading = (
                                np.degrees(self.ekf.state[2]) + 360
                            ) % 360

                with self.state_lock:
                    rc_mode_switch = self.current_state.rc_channels[4]
                    control_mode = self.current_state.control_mode

                    waypoints = self.current_state.waypoints
                    current_waypoint_index = self.current_state.current_waypoint_index

                    vision_target_active = self.current_state.vision_target.get(
                        "active", False
                    )
                    vision_target_obj_class = self.current_state.vision_target.get(
                        "obstacle_class", ""
                    )
                    vision_target_center_x = self.current_state.vision_target.get(
                        "object_center_x", 0
                    )
                    vision_target_frame_width = self.current_state.vision_target.get(
                        "frame_width", 640
                    )

                    resume_waypoint_on_clear = (
                        self.current_state.resume_waypoint_on_clear
                    )
                    nav_dist_to_wp = self.current_state.nav_dist_to_wp
                    esp_status = self.current_state.esp_status
                    status = self.current_state.status

                actuator_config = self.config.get("actuators", {})
                servo_default = actuator_config.get("servo_default_angle", 90)

                command_to_send = None

                if rc_mode_switch < 1500:
                    command_to_send = None
                    logging.info("[AsvHandler] RC OVERRIDE -> Kontrol Jetson ditahan.")

                elif control_mode == "MANUAL":
                    command_to_send = "W\n"
                    logging.info("[AsvHandler] MANUAL CONTROL -> Standby (Kirim W)")

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

                        # Ambil nilai arena yang aktif dari state
                        with self.state_lock:
                            current_arena = self.current_state.active_arena

                        # 1. TENTUKAN ARAH MENGHINDAR ATAU TRACKING
                        # Ambil nilai servo dan motor depan dari GUI secara realtime
                        with self.state_lock:
                            pwm_depan_aktif = self.current_state.vision_front_motor_cmd
                            servo_kiri_aktif = self.current_state.vision_servo_left_cmd
                            servo_kanan_aktif = (
                                self.current_state.vision_servo_right_cmd
                            )

                        pwm_cmd = current_ai_pwm
                        motor_depan_kiri = 1000
                        motor_depan_kanan = 1000
                        servo_cmd = servo_default

                        if obj_class in ["bola-hijau", "bola-merah"]:
                            # --- LOGIKA AVOIDANCE (BOLA) ---
                            if obj_class == "bola-hijau":
                                if current_arena == "Arena_B":
                                    turn_direction = "RIGHT"
                                else:
                                    turn_direction = "LEFT"
                            elif obj_class == "bola-merah":
                                if current_arena == "Arena_B":
                                    turn_direction = "LEFT"
                                else:
                                    turn_direction = "RIGHT"

                            desc = f"{obj_class} -> Avoidance {turn_direction}"

                            if turn_direction == "LEFT":
                                servo_cmd = servo_kiri_aktif
                                motor_depan_kanan = pwm_depan_aktif
                            elif turn_direction == "RIGHT":
                                servo_cmd = servo_kanan_aktif
                                motor_depan_kiri = pwm_depan_aktif
                            else:
                                servo_cmd = servo_default

                        elif obj_class in ["kotak-biru", "kotak-hijau", "kotak-merah"]:
                            total_wps = len(self.current_state.waypoints) if self.current_state.waypoints else 0
                            is_last_wp = (total_wps > 0) and (self.current_state.nav_target_wp_index >= total_wps - 1)
                            
                            if obj_class == "kotak-biru" and is_last_wp:
                                # --- LOGIKA DOCKING (BOLA BIRU) DI WP TERAKHIR ---
                                # Mengarahkan kapal langsung ke bola target yang sudah dikunci (Kiri untuk Arena A, Kanan untuk Arena B)
                                center_x = vision_target_frame_width / 2
                                box_width = self.current_state.vision_target.get("width", 0)

                                target_point = vision_target_center_x
                                error_x = target_point - center_x
                                tolerance = 35
                                max_tracking_deflection = 35

                                with self.state_lock:
                                    dock_front_pwm = self.current_state.dock_motor_depan

                                if error_x < -tolerance:
                                    turn_direction = "TRACKING_LEFT"
                                    offset = (error_x / center_x) * max_tracking_deflection
                                    servo_cmd = int(90 + offset)
                                    motor_depan_kanan = dock_front_pwm
                                    desc = f"Docking -> Track Bola ({current_arena}) Belok Kiri (Err: {error_x:.1f})"
                                elif error_x > tolerance:
                                    turn_direction = "TRACKING_RIGHT"
                                    offset = (error_x / center_x) * max_tracking_deflection
                                    servo_cmd = int(90 + offset)
                                    motor_depan_kiri = dock_front_pwm
                                    desc = f"Docking -> Track Bola ({current_arena}) Belok Kanan (Err: {error_x:.1f})"
                                else:
                                    turn_direction = "TRACKING_CENTER"
                                    servo_cmd = servo_default
                                    desc = f"Docking -> Kunci Bola ({current_arena}) Tengah Lurus"

                                servo_cmd = max(35, min(145, servo_cmd))

                                is_docking_enabled = self.current_state.docking_enabled
                                if is_docking_enabled and vision_target_frame_width > 0 and box_width > 0:
                                    proximity_ratio = box_width / vision_target_frame_width
                                    if proximity_ratio > 0.40:
                                        command_to_send = "S,DOCK_SWING\n"
                                        desc = f"Docking -> BOLA TARGET {current_arena} SANGAT DEKAT! TRIGGER SWING DOCKING!"
                                        logging.warning(f"[AsvHandler] DOCKING SWING TRIGGERED by Proximity ({proximity_ratio:.2f})")
                                        
                            elif obj_class in ["kotak-biru", "kotak-hijau"]:
                                # --- LOGIKA AVOIDANCE (KOTAK BIRU & HIJAU) ---
                                with self.state_lock:
                                    box_servo_kiri_aktif = self.current_state.box_servo_left_cmd
                                    box_servo_kanan_aktif = self.current_state.box_servo_right_cmd
                                    box_front_aktif = self.current_state.box_front_motor_cmd
                                    box_speed_aktif = self.current_state.box_speed_cmd
                                
                                if obj_class == "kotak-biru":
                                    if current_arena == "Arena_B":
                                        turn_direction = "LEFT"
                                    else:
                                        turn_direction = "RIGHT"
                                elif obj_class == "kotak-hijau":
                                    if current_arena == "Arena_B":
                                        turn_direction = "RIGHT"
                                    else:
                                        turn_direction = "LEFT"

                                desc = f"{obj_class} -> Avoidance {turn_direction}"
                                motor_bawah = box_speed_aktif

                                if turn_direction == "LEFT":
                                    servo_cmd = box_servo_kiri_aktif
                                    motor_depan_kanan = box_front_aktif
                                elif turn_direction == "RIGHT":
                                    servo_cmd = box_servo_kanan_aktif
                                    motor_depan_kiri = box_front_aktif
                                else:
                                    servo_cmd = servo_default
                                    
                            elif obj_class == "kotak-merah":
                                # --- LOGIKA TRACKING (KOTAK MERAH) ---
                                center_x = vision_target_frame_width / 2
                                error_x = vision_target_center_x - center_x
                                tolerance = 40
                                max_tracking_deflection = 30

                                if error_x < -tolerance:
                                    turn_direction = "TRACKING_LEFT"
                                    offset = (error_x / center_x) * max_tracking_deflection
                                    servo_cmd = int(90 + offset)
                                    motor_depan_kanan = pwm_depan_aktif
                                    desc = f"{obj_class} -> Track Kiri (Err: {error_x:.1f})"
                                elif error_x > tolerance:
                                    turn_direction = "TRACKING_RIGHT"
                                    offset = (error_x / center_x) * max_tracking_deflection
                                    servo_cmd = int(90 + offset)
                                    motor_depan_kiri = pwm_depan_aktif
                                    desc = f"{obj_class} -> Track Kanan (Err: {error_x:.1f})"
                                else:
                                    turn_direction = "TRACKING_CENTER"
                                    servo_cmd = servo_default
                                    desc = f"{obj_class} -> Track Tengah"

                                servo_cmd = max(40, min(140, servo_cmd))

                            # Clamp servo value safely
                            servo_cmd = max(40, min(140, servo_cmd))
                        else:
                            turn_direction = "STRAIGHT"
                            servo_cmd = servo_default
                            desc = "Unknown -> Lurus"

                        # 4. EKSEKUSI PENGIRIMAN SERIAL
                        if command_to_send and command_to_send.startswith("S,DOCK_SWING"):
                            # Jangan di-overwrite jika kita sudah memicu DOCK_SWING
                            pass
                        elif nav_dist_to_wp < 1.5:
                            command_to_send = "W\n"
                            logging.info(
                                "[AsvHandler] AI STATIC: Jarak WP < 1.5m. Melepas ke Waypoint Nav."
                            )
                        elif esp_status == "WP_COMPLETE" or status == "WP_COMPLETE":
                            command_to_send = "W\n"
                            logging.info(
                                "[AsvHandler] WP_COMPLETE dilaporkan -> mengirim W"
                            )
                        else:
                            # Kirim 5 parameter: A, Servo (Selalu 90), Motor Belakang, Motor Kiri Depan, Motor Kanan Depan
                            command_to_send = f"A,{servo_cmd},{int(pwm_cmd)},{motor_depan_kiri},{motor_depan_kanan}\n"
                            logging.debug(f"[LOGIC DEBUG] AI ACTIVE | Motor Bawah: {int(pwm_cmd)} | Action: {desc}")
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

            self.socketio.sleep(0.02)

    def process_command(self, command, payload):
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "SET_WAYPOINTS": self._handle_set_waypoints,
            "UPDATE_PID": self._handle_update_pid,
            "VISION_TARGET_UPDATE": self._handle_vision_target_update,
            "UPDATE_VISION_SPEED": self._handle_update_vision_speed,
            "UPDATE_VISION_FRONT_MOTOR": self._handle_update_vision_front_motor,
            "UPDATE_VISION_SERVO": self._handle_update_vision_servo,
            "UPDATE_VISION_DISTANCE": self._handle_update_vision_distance,
            "UPDATE_VISION_MODEL": self._handle_update_vision_model,
            "UPDATE_VISION_WP_RANGES": self._handle_update_vision_wp_ranges,
            "UPDATE_BOX_AVOIDANCE_CONFIG": self._handle_update_box_avoidance_config,
            "DEBUG_WP_COUNTER": self._handle_debug_counter,
            "SWAP_CAMERAS": self._handle_swap_cameras,
            "SET_PHOTO_MISSION": self._handle_set_photo_mission,
            "SET_PORTRAIT_CONFIG": self._handle_set_portrait_config,
            "SET_DOCK_CONFIG": self._handle_set_dock_config,
            "SET_DOCK_ENABLED": self._handle_set_dock_enabled,
            "ARM_REPLACE_WP": self._handle_arm_replace_wp,
            "REQUEST_WP_SYNC": self._handle_request_wp_sync,
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

    def _handle_request_wp_sync(self, payload):
        """Meminta data waypoint secara paksa dari ESP32."""
        if self.serial_handler.is_connected:
            self.serial_handler.send_command("P,GET_WP\n")
            logging.info(
                "[AsvHandler] Mengirim permintaan sinkronisasi waypoint (P,GET_WP) ke ESP32."
            )
        else:
            logging.warning("[AsvHandler] Tidak bisa meminta sync WP, serial terputus.")

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
            pwm_val = int(payload.get("pwm", 1500))
            with self.state_lock:
                self.current_state.vision_front_motor_cmd = pwm_val
            logging.info(f"[AsvHandler] Motor Depan AI Updated -> PWM: {pwm_val}")
        except ValueError:
            logging.warning("[AsvHandler] Payload PWM motor depan tidak valid")

    def _handle_update_box_avoidance_config(self, payload):
        try:
            dist = float(payload.get("distance", 165.0))
            left_val = int(payload.get("left", 70))
            right_val = int(payload.get("right", 110))
            speed_val = int(payload.get("speed", 1500))
            front_val = int(payload.get("front_motor", payload.get("pwm", 1800)))
            
            # Validasi range servo dan pwm
            left_val = max(0, min(90, left_val))
            right_val = max(90, min(180, right_val))
            speed_val = max(1000, min(2000, speed_val))
            front_val = max(1000, min(2000, front_val))

            with self.state_lock:
                self.current_state.box_avoidance_distance = dist
                self.current_state.box_servo_left_cmd = left_val
                self.current_state.box_servo_right_cmd = right_val
                self.current_state.box_speed_cmd = speed_val
                self.current_state.box_front_motor_cmd = front_val
                self.current_state.box_motor_pwm_cmd = front_val

            logging.info(f"[AsvHandler] Box Avoidance Config Updated -> Dist: {dist}cm, Left: {left_val}, Right: {right_val}, Speed: {speed_val}, Front: {front_val}")
        except ValueError:
            logging.warning("[AsvHandler] Payload Box Avoidance Config tidak valid")

    def _handle_debug_counter(self, payload):
        action = payload.get("action")
        cmd = ""

        with self.state_lock:
            max_points = (
                len(self.current_state.waypoints)
                if self.current_state.waypoints
                else 99
            )
            if action == "INC":
                cmd = "C,INC\n"
                self.current_state.current_waypoint_index += 1
            elif action == "DEC":
                cmd = "C,DEC\n"
                self.current_state.current_waypoint_index = max(
                    0, self.current_state.current_waypoint_index - 1
                )
            elif action == "RESET":
                cmd = "C,RESET\n"
                self.current_state.current_waypoint_index = 0
                self.current_state.photo_mission_qty_taken_1 = 0
                self.current_state.photo_mission_qty_taken_2 = 0

            self.current_state.current_waypoint_index = min(
                self.current_state.current_waypoint_index, max_points
            )

        if hasattr(self, "serial_handler") and self.serial_handler.is_connected and cmd:
            self.serial_handler.send_command(cmd)
            logging.info(
                f"[AsvHandler] Mengirim manual waypoint update ke ESP32: {cmd.strip()}"
            )
        else:
            logging.warning(
                "[AsvHandler] Gagal mengirim manual wp update, serial tidak terhubung."
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
        if not hasattr(self, "vision_service"):
            logging.error("[AsvHandler] Gagal Capture: Vision Service belum terhubung.")
            return

        capture_type = payload.get("type", "surface")
        is_raw = payload.get("raw", False)

        result = self.vision_service.trigger_manual_capture(
            capture_type, raw_mode=is_raw
        )

        if result.get("status") == "success":
            logging.info(
                f"[AsvHandler] Capture Berhasil: {result.get('file')} (Mode: {result.get('mode', 'Overlaid')})"
            )
        else:
            logging.error(f"[AsvHandler] Capture Gagal: {result.get('message')}")

    def set_streaming_status(self, status: bool):
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
            logging.info(f"[AsvHandler] PID updated: P={kp}, I={ki}, D={kd}")

    def _handle_serial_configuration(self, payload):
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO":
            self.serial_handler.find_and_connect_esp32(baud)
        else:
            self.serial_handler.connect(port, baud)

    def _handle_update_vision_distance(self, payload):
        dist = payload.get("distance", 165)
        self.config["camera_detection"]["obstacle_activation_distance_cm"] = float(dist)
        self._save_config()
        if hasattr(self, "vision_service") and self.vision_service:
            # Bisa ditambahkan metode di vision_service untuk update distance runtime
            pass
        print(f"[AsvHandler] Diperbarui: Vision Distance = {dist} cm")

    def _handle_update_vision_model(self, payload):
        model_name = payload.get("model", "Auto")
        print(f"[AsvHandler] Menerima request ganti model AI: {model_name}")
        if hasattr(self, "vision_service") and self.vision_service:
            self.vision_service.change_model(model_name)
        else:
            print("[AsvHandler] vision_service tidak tersedia untuk ganti model.")

    def _handle_update_vision_wp_ranges(self, payload):
        """Memperbarui rentang deteksi WP dari GUI"""
        vision_cfg = self.config.get("vision", {})
        if "wp_range_bola" in payload:
            vision_cfg["wp_range_bola"] = payload["wp_range_bola"]
        if "wp_range_kotak_biru" in payload:
            vision_cfg["wp_range_kotak_biru"] = payload["wp_range_kotak_biru"]
        if "wp_range_kotak_hijau" in payload:
            vision_cfg["wp_range_kotak_hijau"] = payload["wp_range_kotak_hijau"]

        self.config["vision"] = vision_cfg
        logging.info(f"[AsvHandler] Vision WP Ranges diperbarui: {payload}")

    def _handle_swap_cameras(self, payload):
        with self.state_lock:
            if hasattr(self, "vision_service") and self.vision_service:
                self.vision_service.set_camera_swap(payload.get("swapped", False))
                logging.info(
                    f"[AsvHandler] Perintah Swap Kamera diteruskan ke VisionService: {payload}"
                )
            else:
                logging.warning(
                    "[AsvHandler] vision_service belum di-inject, gagal swap kamera"
                )

    def _handle_arm_replace_wp(self, payload):
        try:
            index = int(payload.get("index", -1))
            if index >= 0:
                if self.serial_handler.is_connected:
                    self.serial_handler.send_command(f"P,ARM,{index}\n")
                    logging.info(
                        f"[AsvHandler] Target bidikan WP {index} dikirim ke ESP32."
                    )
                else:
                    logging.warning(
                        "[AsvHandler] Gagal arming: Serial tidak terhubung."
                    )
        except Exception as e:
            logging.error(f"[AsvHandler] Error saat arming replace wp: {e}")

    def _handle_set_waypoints(self, payload):
        waypoints_data = payload.get("waypoints")
        raw_arena = payload.get("arena") or payload.get("arena_id")

        arena_id = "Arena_A"
        if raw_arena:
            clean_arena = str(raw_arena).strip().upper().replace(" ", "_")

            if (
                clean_arena == "B"
                or clean_arena.endswith("_B")
                or "ARENA_B" in clean_arena
            ):
                arena_id = "Arena_B"
            else:
                arena_id = "Arena_A"

        with self.state_lock:
            if arena_id is not None:
                self.current_state.active_arena = arena_id

            if waypoints_data is not None:
                if isinstance(waypoints_data, list):
                    if len(waypoints_data) > 20:
                        logging.warning(
                            f"[AsvHandler] Waypoints melebih batas (20). Dipotong dari {len(waypoints_data)} menjadi 20."
                        )
                        waypoints_data = waypoints_data[:20]
                    self.current_state.waypoints = waypoints_data
                    self.current_state.current_waypoint_index = 0
                    self.logger.log_event(
                        f"Waypoints dimuat (Arena: {self.current_state.active_arena}). Jml: {len(waypoints_data)}"
                    )

                    if self.serial_handler.is_connected:
                        self.serial_handler.send_command("P,CLEAR\n")
                        time.sleep(0.1)  # Beri waktu ESP32 menghapus memori
                        for wp in waypoints_data:
                            self.serial_handler.send_command(
                                f"P,ADD,{wp['lat']:.6f},{wp['lon']:.6f}\n"
                            )
                            time.sleep(0.05)  # Cegah buffer overflow ESP32
                        self.serial_handler.send_command("P,SAVE\n")
                        logging.info(
                            "[AsvHandler] Waypoints kustom berhasil disinkronkan ke ESP32."
                        )
                else:
                    logging.warning(
                        "[AsvHandler] Gagal set waypoints: Data tidak valid (bukan list)."
                    )

            logging.info(
                f"[Setup] Arena: {self.current_state.active_arena} | Jml Waypoints: {len(self.current_state.waypoints)}"
            )

    def _handle_replace_waypoint(self, payload):
        idx = payload.get("index", -1)
        wp = payload.get("waypoint", {})
        lat = wp.get("lat")
        lon = wp.get("lon")

        if idx >= 0 and lat is not None and lon is not None:
            with self.state_lock:
                if idx < len(self.current_state.waypoints):
                    self.current_state.waypoints[idx] = {"lat": lat, "lon": lon}

            if self.serial_handler.is_connected:
                self.serial_handler.send_command(
                    f"P,REPLACE,{idx},{lat:.6f},{lon:.6f}\n"
                )
                logging.info(
                    f"[AsvHandler] Waypoint index {idx} berhasil direplace via Live GPS."
                )
        else:
            logging.warning(
                "[AsvHandler] Gagal replace waypoint: Payload tidak lengkap."
            )

    def _handle_set_photo_mission(self, payload):
        """Menerima konfigurasi rentang waypoint untuk misi foto segment."""
        try:
            surf_wp1 = payload.get("surf_wp1", 13)
            surf_wp2 = payload.get("surf_wp2", 14)
            under_wp1 = payload.get("under_wp1", 11)
            under_wp2 = payload.get("under_wp2", 12)
            count = payload.get("count", 5)
            interval = payload.get("interval", 2.0)

            with self.state_lock:
                self.current_state.photo_mission_interval = float(interval)
                self.current_state.photo_mission_surf_wp1 = surf_wp1
                self.current_state.photo_mission_surf_wp2 = surf_wp2
                self.current_state.photo_mission_under_wp1 = under_wp1
                self.current_state.photo_mission_under_wp2 = under_wp2
                self.current_state.photo_mission_qty_requested = count
                # Reset counter
                self.current_state.photo_mission_qty_taken_1 = 0
                self.current_state.photo_mission_qty_taken_2 = 0

            # Kirim range portrait ke ESP32
            if self.serial_handler.is_connected:
                self.serial_handler.send_command(
                    f"S,PT_RANGE,{under_wp1},{under_wp2},{surf_wp1},{surf_wp2}\n"
                )
                logging.info(
                    f"[AsvHandler] Portrait Range dikirim ke ESP32: UW={under_wp1}-{under_wp2}, Surf={surf_wp1}-{surf_wp2}"
                )

            self.logger.log_event(
                f"[GUI Command] Set Photo Mission -> Surf: {surf_wp1}-{surf_wp2}, Under: {under_wp1}-{under_wp2}, Qty: {count}"
            )
        except Exception as e:
            logging.error(f"Error handling SET_PHOTO_MISSION: {e}")

    def _handle_set_portrait_config(self, payload):
        """Menerima konfigurasi kecepatan & durasi portrait dari GUI."""
        try:
            speed = payload.get("speed", 1400)
            stop_ms = payload.get("stop_ms", 3000)
            reverse_ms = payload.get("reverse_ms", 2000)
            reverse_speed = payload.get("reverse_speed", 1400)

            with self.state_lock:
                self.current_state.portrait_speed = speed
                self.current_state.portrait_stop_ms = stop_ms
                self.current_state.portrait_reverse_ms = reverse_ms
                self.current_state.portrait_reverse_speed = reverse_speed

            # Kirim konfigurasi portrait ke ESP32
            if self.serial_handler.is_connected:
                self.serial_handler.send_command(
                    f"S,PORTRAIT,{speed},{stop_ms},{reverse_ms},{reverse_speed}\n"
                )
                logging.info(
                    f"[AsvHandler] Portrait Config dikirim ke ESP32: Spd={speed}, Stop={stop_ms}ms, Rev={reverse_ms}ms, RevSpd={reverse_speed}"
                )
        except Exception as e:
            logging.error(f"Error handling SET_PORTRAIT_CONFIG: {e}")

    def _handle_set_dock_enabled(self, payload):
        """Toggle docking mission on/off dari GUI."""
        try:
            enabled = bool(payload.get("enabled", True))
            with self.state_lock:
                self.current_state.docking_enabled = enabled

            cmd = "S,DOCK_EN\n" if enabled else "S,DOCK_DIS\n"
            if self.serial_handler.is_connected:
                self.serial_handler.send_command(cmd)

            state_str = "ENABLED" if enabled else "DISABLED"
            logging.info(f"[AsvHandler] Docking mission {state_str}.")
            self.logger.log_event(f"[GUI Command] Docking {state_str}")
        except Exception as e:
            logging.error(f"Error handling SET_DOCK_ENABLED: {e}")

    def _handle_set_dock_config(self, payload):
        """Menerima konfigurasi docking dari GUI dan mengirim ke ESP32."""
        try:
            motor_utama = int(payload.get("motor_utama_pwm", 1200))
            motor_depan = int(payload.get("motor_depan_pwm", 1400))
            charge_ms = int(payload.get("charge_duration_ms", 3000))
            servo_left = int(payload.get("servo_left", 0))
            servo_right = int(payload.get("servo_right", 180))

            # Validasi range
            motor_utama = max(1000, min(2000, motor_utama))
            motor_depan = max(1000, min(2000, motor_depan))
            servo_left = max(0, min(90, servo_left))
            servo_right = max(90, min(180, servo_right))

            with self.state_lock:
                self.current_state.dock_motor_utama = motor_utama
                self.current_state.dock_motor_depan = motor_depan
                self.current_state.dock_charge_duration_ms = charge_ms
                self.current_state.dock_servo_left = servo_left
                self.current_state.dock_servo_right = servo_right
                arena = self.current_state.active_arena

            direction = 1 if "B" in arena else 0  # 0=KIRI(A), 1=KANAN(B)

            if self.serial_handler.is_connected:
                self.serial_handler.send_command(
                    f"S,DOCK,{motor_utama},{charge_ms},{direction},{servo_left},{servo_right}\n"
                )
                logging.info(
                    f"[AsvHandler] Dock Config dikirim ke ESP32: {motor_utama},{charge_ms},{direction},{servo_left},{servo_right}"
                )

            self.logger.log_event(
                f"[GUI Command] Set Dock Config -> Motor:{motor_utama}, Front:{motor_depan}, Charge:{charge_ms}ms, Dir:{direction}, Left:{servo_left}, Right:{servo_right}"
            )
        except Exception as e:
            logging.error(f"Error handling SET_DOCK_CONFIG: {e}")

    def stop(self):
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        logging.info("[AsvHandler] Dihentikan.")
