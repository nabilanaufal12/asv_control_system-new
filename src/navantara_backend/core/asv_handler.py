# src/navantara_backend/core/asv_handler.py
import threading
import time
import math
import numpy as np
import json
import logging

# --- [OPTIMASI 3] ---
from dataclasses import dataclass, asdict, field
# --- [AKHIR OPTIMASI 3] ---

from navantara_backend.services.serial_service import SerialHandler
from navantara_backend.core.navigation import (
    run_navigation_logic,
    PIDController,
    haversine_distance,
)
from navantara_backend.core.kalman_filter import SimpleEKF
from navantara_backend.core.mission_logger import MissionLogger

from navantara_backend.vision.cloud_utils import send_telemetry_to_firebase


def map_value(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# --- [OPTIMASI 3: DEFINISI DATACLASS] ---
@dataclass
class AsvState:
    control_mode: str = "AUTO"
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
    nav_dist_to_wp: float = 9999.0 # default = 0.0
    nav_target_bearing: float = 0.0
    nav_heading_error: float = 0.0
    nav_servo_cmd: int = 90
    nav_motor_cmd: int = 1500
    nav_gps_sats: int = 0
    manual_servo_cmd: int = 90
    manual_motor_cmd: int = 1500
    active_arena: str = None
    debug_waypoint_counter: int = 0
    use_dummy_counter: bool = False
    esp_status: str = None
    vision_target: dict = field(default_factory=lambda: {"active": False})
    # gate_target & gate_context dibiarkan agar tidak error jika dipanggil UI/Vision, 
    # tapi tidak lagi digunakan dalam logika.
    gate_target: dict = field(default_factory=lambda: {"active": False})
    avoidance_direction: str = None
    is_avoiding: bool = False
    gate_context: dict = field(default_factory=lambda: {"last_gate_config": None})
    recovering_from_avoidance: bool = False
    last_avoidance_time: float = 0.0
    last_pixel_error: float = 0.0
    resume_waypoint_on_clear: bool = False
    inverse_servo: bool = False
    # --- [MODIFIKASI 1.1: Atribut Misi Foto Dual-Target] ---
    photo_mission_target_wp1: int = -1
    photo_mission_target_wp2: int = -1
    photo_mission_qty_requested: int = 0
    photo_mission_qty_taken_1: int = 0
    photo_mission_qty_taken_2: int = 0
    # --- [AKHIR MODIFIKASI 1.1] ---
# --- [AKHIR OPTIMASI 3] ---


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

        # --- [OPTIMASI 3] ---
        self.current_state = AsvState()
        # --- [AKHIR OPTIMASI 3] ---

        # --- [OPTIMASI 1: DELTA STATE] ---
        self.last_emitted_state = {}
        # --- [AKHIR OPTIMASI 1] ---

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
        self.logger = MissionLogger()
        self.logger.log_event("AsvHandler diinisialisasi.")
        logging.info("[AsvHandler] Handler diinisialisasi untuk operasi backend.")
        self.initiate_auto_connection()

    def initiate_auto_connection(self):
        serial_cfg = self.config.get("serial_connection", {})
        force_port = serial_cfg.get("force_serial_port")
        baud_rate = serial_cfg.get("default_baud_rate", 115200)

        if force_port:
            self.use_dummy_serial = False
            self.serial_handler.use_dummy_serial = False
            logging.info(f"[AsvHandler] Memaksa koneksi serial ke: {force_port} @ {baud_rate}")
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

    # --- [FUNGSI _update_and_emit_state DIMODIFIKASI] ---
    def _update_and_emit_state(self):
        """
        Secara efisien menghitung logika status dan HANYA mengirimkan
        perubahan (delta) ke GUI.
        """
        if not (self.running and self.is_streaming_to_gui):
            return

        delta_payload = {}
        processed_status = "DISCONNECTED"  # Default status

        with self.state_lock:
            # 1. Salin HANYA data yang diperlukan untuk logika status
            is_serial_connected = self.serial_handler.is_connected
            control_mode = self.current_state.control_mode
            vision_target_active = self.current_state.vision_target.get("active", False)
            recovering = self.current_state.recovering_from_avoidance
            esp_status = self.current_state.esp_status
            waypoints = self.current_state.waypoints
            nav_target_wp_index = self.current_state.nav_target_wp_index
            rc_mode_switch = self.current_state.rc_channels[4]

            # 2. Update status koneksi internal (GUI perlu ini)
            self.current_state.is_connected_to_serial = is_serial_connected

            # 3. Logika Status (Dimodifikasi: Hapus Gate/Contextual)
            if not is_serial_connected:
                processed_status = "DISCONNECTED (SERIAL)"
            elif rc_mode_switch < 1500:
                processed_status = "RC MANUAL OVERRIDE"
            elif vision_target_active:
                processed_status = "AI: STATIC AVOIDANCE"  # Status Baru
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

            # 4. Bangun "delta"
            current_state_dict = asdict(self.current_state)
            for key, value in current_state_dict.items():
                if (
                    key not in self.last_emitted_state
                    or self.last_emitted_state[key] != value
                ):
                    delta_payload[key] = value

            if delta_payload:
                self.last_emitted_state.update(delta_payload)

        if delta_payload:
            self.socketio.emit("telemetry_update", delta_payload)
    # --- [AKHIR FUNGSI STATE] ---


    def _read_from_serial_loop(self):
        while self.running:
            line = self.serial_handler.read_line()
            if line:
                logging.info(f"[Serial RAW] {line}")
                try:
                    data = json.loads(line)
                    logging.debug(
                        f"[Serial PARSED] mode={data.get('mode')} status={data.get('status')}"
                    )
                    self._parse_json_telemetry(data)
                except json.JSONDecodeError:
                    if line.strip():
                        logging.debug(
                            f"[Serial] Menerima data mentah (Bukan JSON): {line}"
                        )
                except Exception as e:
                    logging.warning(f"[Serial] Error parsing: {e}, Data: {line}")
            self.socketio.sleep(0.01)

    # --- [FUNGSI DIMODIFIKASI UNTUK OPTIMASI 3] ---
    def _parse_json_telemetry(self, data):
        try:
            with self.state_lock:
                self.current_state.heading = data.get(
                    "heading", self.current_state.heading
                )
                self.current_state.speed = data.get("speed_kmh", 0.0) / 3.6
                self.current_state.nav_gps_sats = data.get(
                    "sats", self.current_state.nav_gps_sats
                )
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
                mode = data.get("mode")
                if mode == "MANUAL":
                    self.current_state.manual_servo_cmd = data.get("servo_out")
                    self.current_state.manual_motor_cmd = data.get("motor_out")
                elif mode == "AUTO":
                    status = data.get("status")
                    if status == "WAYPOINT":
                        self.current_state.nav_target_wp_index = data.get(
                            "wp_target_idx"
                        )
                        self.current_state.nav_dist_to_wp = data.get("wp_dist_m")
                        self.current_state.nav_target_bearing = data.get(
                            "wp_target_brg"
                        )
                        self.current_state.nav_heading_error = data.get(
                            "wp_error_hdg"
                        )
                        self.current_state.nav_servo_cmd = data.get("servo_out")
                        self.current_state.nav_motor_cmd = data.get("motor_out")
                    elif status == "AI_ACTIVE":
                        self.current_state.nav_servo_cmd = data.get("servo_out")
                        self.current_state.nav_motor_cmd = data.get("motor_out")
        except Exception as e:
            logging.error(
                f"[AsvHandler] Gagal mem-parsing data JSON: {e}. Data: {data}"
            )
    # --- [AKHIR OPTIMASI 3] ---

    # --- [MAIN LOGIC LOOP DIMODIFIKASI TOTAL UNTUK REQUIREMENT BARU] ---
    def main_logic_loop(self):
        self.socketio.start_background_task(self._read_from_serial_loop)
        logging.info("[AsvHandler] Loop pembaca serial dimulai.")
        while self.running:
            try:
                # Coba reconnect jika dalam mode AUTO dan tidak terhubung
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
                    logging.info("[AsvHandler] Mode AUTO aktif, mencoba koneksi ulang ke ESP32...")
                    baud_rate = self.config.get("serial_connection", {}).get(
                        "default_baud_rate", 115200
                    )
                    self.serial_handler.find_and_connect_esp32(baud_rate)

                # --- (Blok EKF) ---
                current_time = time.time()
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
                # --- (Akhir Blok EKF) ---

                # --- AMBIL DATA STATE ---
                with self.state_lock:
                    rc_mode_switch = self.current_state.rc_channels[4]
                    control_mode = self.current_state.control_mode
                    manual_servo_cmd = self.current_state.manual_servo_cmd
                    manual_motor_cmd = self.current_state.manual_motor_cmd
                    
                    waypoints = self.current_state.waypoints
                    current_waypoint_index = self.current_state.current_waypoint_index
                    
                    vision_target_active = self.current_state.vision_target.get("active", False)
                    vision_target_obj_class = self.current_state.vision_target.get("obstacle_class", "")
                    
                    active_arena = self.current_state.active_arena
                    
                    resume_waypoint_on_clear = self.current_state.resume_waypoint_on_clear
                    nav_dist_to_wp = self.current_state.nav_dist_to_wp
                    esp_status = self.current_state.esp_status
                    status = self.current_state.status
                    # Note: inverse_servo (manual toggle) bisa digabung atau diabaikan jika sistem otomatis penuh

                # --- KONFIGURASI SERVO & INVERSI ---
                actuator_config = self.config.get("actuators", {})
                servo_default = actuator_config.get("servo_default_angle", 90)
                motor_base = actuator_config.get("motor_pwm_auto_base", 1300)
                
                # Sudut Statis (Normal)
                # Default: Green -> 135 (Kanan), Red -> 45 (Kiri)
                angle_static_right = 135
                angle_static_left = 45
                
                # Konfigurasi Trigger Inversi
                # (Bisa ditaruh di config.json bagian "avoidance_inversion")
                inversion_cfg = self.config.get("avoidance_inversion", {
                    "arena_a_wp_index": 7, # Default tinggi agar tidak terpicu sembarangan
                    "arena_b_wp_index": 7
                })
                
                # LOGIKA INVERSI OTOMATIS
                is_inverted_auto = False
                if active_arena == "Arena_A" and current_waypoint_index >= inversion_cfg.get("arena_a_wp_index", 999):
                    is_inverted_auto = True
                    logging.debug(f"[Logic] Auto Inversion ACTIVE (Arena A, WP {current_waypoint_index})")
                elif active_arena == "Arena_B" and current_waypoint_index >= inversion_cfg.get("arena_b_wp_index", 999):
                    is_inverted_auto = True
                    logging.debug(f"[Logic] Auto Inversion ACTIVE (Arena B, WP {current_waypoint_index})")

                command_to_send = None

                # PRIORITAS 1: RC OVERRIDE
                if rc_mode_switch < 1500:
                    command_to_send = None
                    logging.info("[AsvHandler] RC OVERRIDE -> Kontrol Jetson ditahan.")

                # PRIORITAS 2: MANUAL GUI (WASD)
                elif control_mode == "MANUAL":
                    command_to_send = f"A,{int(manual_servo_cmd)},{int(manual_motor_cmd)}\n"
                    logging.info(f"[AsvHandler] MANUAL CONTROL -> Servo: {int(manual_servo_cmd)} deg, Motor: {int(manual_motor_cmd)} us")

                # PRIORITAS 3: AUTO (AI & WAYPOINT)
                elif control_mode == "AUTO":
                    mission_completed = bool(waypoints and current_waypoint_index >= len(waypoints))
                    if mission_completed:
                        command_to_send = "W\n"
                    
                    # === PRIORITAS 3.1: PENGHINDARAN STATIS (SIMPLE AVOIDANCE) ===
                    # Menggantikan Gate Traversal & Contextual Avoidance
                    if vision_target_active:
                        with self.state_lock:
                            self.current_state.recovering_from_avoidance = False
                            self.current_state.is_avoiding = True
                            self.current_state.gate_context["last_gate_config"] = None # Clear context
                        
                        obj_class = vision_target_obj_class
                        servo_cmd = servo_default
                        desc = "Neutral"
                        
                        # Logika Pemilihan Sudut
                        if is_inverted_auto:
                            # INVERS:
                            # Hijau -> Kiri (45)
                            # Merah -> Kanan (135)
                            if obj_class == "green_buoy":
                                servo_cmd = angle_static_left
                                desc = "Green->Left (INV)"
                            elif obj_class == "red_buoy":
                                servo_cmd = angle_static_right
                                desc = "Red->Right (INV)"
                        else:
                            # NORMAL:
                            # Hijau -> Kanan (135)
                            # Merah -> Kiri (45)
                            if obj_class == "green_buoy":
                                servo_cmd = angle_static_right
                                desc = "Green->Right (NRM)"
                            elif obj_class == "red_buoy":
                                servo_cmd = angle_static_left
                                desc = "Red->Left (NRM)"
                        
                        # Motor agak ngebut untuk menghindar
                        pwm_cmd = motor_base + 350
                        
                        # Pengecekan Jarak WP untuk override AI
                        if nav_dist_to_wp < 1.5:
                            command_to_send = "W\n"
                            logging.info("[AsvHandler] AI STATIC: Jarak WP < 1.5m. Melepas ke Waypoint Nav.")
                        elif esp_status == "WP_COMPLETE" or status == "WP_COMPLETE":
                            command_to_send = "W\n"
                            logging.info("[AsvHandler] WP_COMPLETE dilaporkan -> mengirim W untuk melanjutkan waypoint")
                        else:
                            command_to_send = f"A,{servo_cmd},{int(pwm_cmd)}\n"
                            logging.info(f"[AsvHandler] AI CONTROL [Static] -> {desc} | Servo: {servo_cmd}")

                    # === PRIORITAS 3.2: TRANSISI INSTAN KE WAYPOINT ===
                    elif resume_waypoint_on_clear:
                        with self.state_lock:
                            self.current_state.is_avoiding = False
                            self.current_state.avoidance_direction = None
                            self.current_state.recovering_from_avoidance = False
                            self.current_state.resume_waypoint_on_clear = False
                        command_to_send = "W\n"
                        logging.info("[AsvHandler] Transisi cepat -> waypoint mode")

                    # === PRIORITAS 3.3 (DEFAULT): NAVIGASI WAYPOINT ===
                    else:
                        with self.state_lock:
                            self.current_state.last_pixel_error = 0
                        if self.serial_handler.is_connected:
                            command_to_send = "W\n"
                            logging.info("[AsvHandler] WAYPOINT CONTROL -> Mengirim: W")
                        else:
                            command_to_send = None
                            logging.info("[AsvHandler] WAYPOINT CONTROL -> Menunggu koneksi serial...")

                # --- (Blok kirim perintah akhir) ---
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
                            logging.info("[AsvHandler] Mission complete detected -> forcing W")

                if command_to_send is None and resume_waypoint_on_clear:
                    if self.serial_handler.is_connected and control_mode == "AUTO":
                        command_to_send = "W\n"
                        logging.info("[AsvHandler] Vision cleared -> sending resume W")
                        with self.state_lock:
                            self.current_state.resume_waypoint_on_clear = False

                if command_to_send:
                    self.serial_handler.send_command(command_to_send)

                with self.state_lock:
                    state_for_log = asdict(self.current_state)

                self.logger.log_telemetry(state_for_log)
                self._update_and_emit_state() 
                send_telemetry_to_firebase(state_for_log, self.config)

            except Exception as e:
                logging.error(f"[FATAL] Error di main_logic_loop: {e}", exc_info=True)

            self.socketio.sleep(0.02)
    # --- [AKHIR MAIN LOGIC LOOP] ---


    def process_command(self, command, payload):
        # logging.critical(f"[Handler DEBUG] MENERIMA process_command: {command}")
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control,
            "SET_WAYPOINTS": self._handle_set_waypoints,
            "NAV_START": self._handle_start_mission,
            "NAV_RETURN": self._handle_initiate_rth,
            "UPDATE_PID": self._handle_update_pid,
            "VISION_TARGET_UPDATE": self._handle_vision_target_update,
            # "GATE_TRAVERSAL_COMMAND": DIHAPUS
            "DEBUG_WP_COUNTER": self._handle_debug_counter,
            "INVERSE_SERVO": self._handle_inverse_servo,
            "SET_INVERSION": self._handle_set_inversion,
            "SET_PHOTO_MISSION": self._handle_set_photo_mission,
        }
        handler = command_handlers.get(command)
        if handler:
            handler(payload)
        else:
            logging.warning(
                f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'"
            )

    def _handle_set_inversion(self, payload):
        """Menangani toggling mode inversi dari GUI."""
        with self.state_lock:
            new_state = payload.get("inverted", False)
            if self.current_state.inverse_servo != new_state:
                self.current_state.inverse_servo = new_state
                logging.info(f"[AsvHandler] Kontrol Inversi diatur ke: {new_state}")

    def _handle_inverse_servo(self, payload):
        """Handler untuk mengubah arah servo (toggle antara normal dan terbalik)"""
        with self.state_lock:
            if payload.get("toggle"):
                self.current_state.inverse_servo = not self.current_state.inverse_servo
            elif "value" in payload:
                self.current_state.inverse_servo = bool(payload["value"])
            logging.info(f"[AsvHandler] inverse_servo diubah ke: {self.current_state.inverse_servo}")

    # _handle_gate_traversal_command DIHAPUS

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
            logging.info(f"[AsvHandler] Debug counter diatur ke: {self.current_state.debug_waypoint_counter}")

    def _handle_vision_target_update(self, payload):
        with self.state_lock:
            # Pengecekan gate_target.get("active") DIHAPUS karena gate logic sudah non-aktif
            was_active = self.current_state.vision_target.get("active")
            is_active = payload.get("active")
            if was_active and not is_active:
                logging.info("[AsvHandler] Deteksi selesai -> langsung ke waypoint")
                self.current_state.recovering_from_avoidance = False
                self.current_state.is_avoiding = False
                self.current_state.avoidance_direction = None
                self.current_state.resume_waypoint_on_clear = True
                self.current_state.gate_context["last_gate_config"] = None # Reset
                
            self.current_state.vision_target["active"] = is_active
            if is_active:
                self.current_state.vision_target.update(payload)

    # --- [FUNGSI DIMODIFIKASI UNTUK OPTIMASI 2 & 3] ---
    def _handle_manual_control(self, payload):
        with self.state_lock:
            rc_channel_5 = self.current_state.rc_channels[4]
            control_mode = self.current_state.control_mode
            is_inverted = self.current_state.inverse_servo

        if rc_channel_5 < 1500:
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

        # Terapkan inversi
        if is_inverted:
            turn = -turn

        pwm = pwm_stop + fwd * pwr
        servo = servo_def - turn * (servo_def - servo_min)
        servo = max(servo_min, min(servo_max, servo))

        with self.state_lock:
            self.current_state.manual_servo_cmd = int(servo)
            self.current_state.manual_motor_cmd = int(pwm)
    # --- [AKHIR OPTIMASI 2 & 3] ---

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

    def _handle_mode_change(self, payload):
        with self.state_lock:
            new_mode = payload.get("mode", "MANUAL")
            self.current_state.control_mode = new_mode
            self.logger.log_event(f"Mode kontrol GUI diubah ke: {new_mode}")

        if new_mode == "MANUAL":
            actuator_config = self.config.get("actuators", {})
            pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
            servo_def = actuator_config.get("servo_default_angle", 90)
            command_str = f"A,{int(servo_def)},{int(pwm_stop)}\n"
            logging.info(f"[LOG | MODE] GUI ganti ke MANUAL, kirim netral: {command_str.strip()}")
            self.serial_handler.send_command(command_str)

    def _handle_set_waypoints(self, payload):
        waypoints_data = payload.get("waypoints")
        arena_id = payload.get("arena")
        if not isinstance(waypoints_data, list):
            logging.warning("[AsvHandler] Gagal set waypoints: Data tidak valid.")
            return
        with self.state_lock:
            self.current_state.waypoints = waypoints_data
            self.current_state.current_waypoint_index = 0
            self.current_state.active_arena = arena_id
            self.logger.log_event(
                f"Waypoints baru dimuat (Arena: {arena_id}). Jumlah: {len(waypoints_data)}"
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
        """Mengatur parameter untuk misi fotografi otomatis (dual target)."""
        try:
            wp1 = int(payload.get("wp1", -1))
            wp2 = int(payload.get("wp2", -1))
            count = int(payload.get("count", 0))

            with self.state_lock:
                self.current_state.photo_mission_target_wp1 = wp1
                self.current_state.photo_mission_target_wp2 = wp2
                self.current_state.photo_mission_qty_requested = count
                self.current_state.photo_mission_qty_taken_1 = 0
                self.current_state.photo_mission_qty_taken_2 = 0
            
            logging.info(f"[AsvHandler] Misi Foto diatur: WP1={wp1}, WP2={wp2}, Jml={count}/lokasi")
            self.logger.log_event(f"Misi Foto: WP {wp1}&{wp2}, {count}x foto.")
        
        except Exception as e:
            logging.warning(f"[AsvHandler] Gagal mengatur Misi Foto: {e}. Payload: {payload}")

    def stop(self):
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        logging.info("[AsvHandler] Dihentikan.")