# src/navantara_backend/core/asv_handler_dynamic.py
import threading
import time
import numpy as np
import json
import logging
from dataclasses import dataclass, asdict, field

from navantara_backend.services.serial_service import SerialHandler
from navantara_backend.core.navigation import PIDController
from navantara_backend.core.kalman_filter import SimpleEKF
from navantara_backend.core.mission_logger import MissionLogger
from navantara_backend.vision.cloud_utils import send_telemetry_to_firebase

# --- [OPTIMASI KEY MINIFICATION: MAPPING DICTIONARY] ---
TELEMETRY_KEY_MAP = {
    "latitude": "lat",
    "longitude": "lon",
    "heading": "hdg",
    "speed": "sog",
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
    "gate_target": "gate",
    "esp_status": "esp_sts",
}


def map_value(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


@dataclass
class AsvState:
    # --- Core Status ---
    control_mode: str = "AUTO"
    latitude: float = -6.9180
    longitude: float = 107.6185
    heading: float = 90.0
    cog: float = 0.0
    speed: float = 0.0
    battery_voltage: float = 12.5
    status: str = "DISCONNECTED"
    mission_time: str = "00:00:00"
    is_connected_to_serial: bool = False
    esp_status: str = None
    active_arena: str = "Unknown"

    # --- Navigation ---
    waypoints: list = field(default_factory=list)
    current_waypoint_index: int = 0
    nav_target_wp_index: int = 0
    nav_dist_to_wp: float = 0.0
    nav_target_bearing: float = 0.0
    nav_heading_error: float = 0.0
    nav_gps_sats: int = 0
    gyro_z: float = 0.0
    accel_x: float = 0.0

    # --- Actuators ---
    rc_channels: list = field(default_factory=lambda: [1500] * 6)
    nav_servo_cmd: int = 90
    nav_motor_cmd: int = 1500
    manual_servo_cmd: int = 90
    manual_motor_cmd: int = 1500

    # --- Stateful Variables untuk Hierarki Kontrol ---
    # 1. Vision / Obstacle Avoidance
    vision_target: dict = field(
        default_factory=lambda: {
            "active": False,
            "class": "",
            "center_x": 0,
            "frame_width": 640,
        }
    )
    is_avoiding: bool = False
    avoidance_direction: str = None
    last_avoidance_time: float = 0.0
    last_pixel_error: float = 0.0

    # 2. Gate Traversal
    gate_target: dict = field(
        default_factory=lambda: {"active": False, "center_x": 0, "width": 0}
    )
    gate_context: dict = field(
        default_factory=lambda: {"last_gate_config": None, "stable_frames": 0}
    )

    # 3. Recovery & Logic
    recovering_from_avoidance: bool = False
    resume_waypoint_on_clear: bool = False
    inverse_servo: bool = False

    # 4. Mission Parameters
    debug_waypoint_counter: int = 0
    use_dummy_counter: bool = False
    photo_mission_target_wp1: int = -1
    photo_mission_target_wp2: int = -1
    photo_mission_qty_requested: int = 0
    photo_mission_qty_taken_1: int = 0
    photo_mission_qty_taken_2: int = 0

    # 5. Dynamic Tuning
    vision_auto_motor_cmd: int = 1500
    vision_servo_left_cmd: int = 45
    vision_servo_right_cmd: int = 135


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

        self.current_state = AsvState()
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
        self.logger = MissionLogger()
        self.logger.log_event("AsvHandler Dynamic diinisialisasi.")
        logging.info("[AsvHandler] Handler Dynamic siap.")
        self.initiate_auto_connection()

    def initiate_auto_connection(self):
        serial_cfg = self.config.get("serial_connection", {})
        force_port = serial_cfg.get("force_serial_port")
        baud_rate = serial_cfg.get("default_baud_rate", 115200)

        if force_port:
            self.use_dummy_serial = False
            self.serial_handler.use_dummy_serial = False
            logging.info(f"[AsvHandler] Memaksa koneksi serial ke: {force_port}")
            if not self.serial_handler.connect(force_port, baud_rate):
                logging.warning("[AsvHandler] Gagal connect paksa.")

        if self.use_dummy_serial:
            logging.info("[AsvHandler] Mode DUMMY SERIAL aktif.")
            return

        logging.info("[AsvHandler] Auto-connect serial...")
        self.serial_handler.find_and_connect_esp32(baud_rate)

    def set_streaming_status(self, status: bool):
        if self.is_streaming_to_gui != status:
            logging.info(f"[AsvHandler] Status streaming telemetri diatur ke: {status}")
            self.is_streaming_to_gui = status

    def _update_and_emit_state(self):
        if not (self.running and self.is_streaming_to_gui):
            return

        delta_payload = {}
        processed_status = "DISCONNECTED"

        with self.state_lock:
            self.current_state.is_connected_to_serial = self.serial_handler.is_connected

            control_mode = self.current_state.control_mode
            vision_active = self.current_state.vision_target.get("active", False)
            gate_active = self.current_state.gate_target.get("active", False)
            rc_ch5 = self.current_state.rc_channels[4]

            if not self.current_state.is_connected_to_serial:
                processed_status = "DISCONNECTED (SERIAL)"
            elif rc_ch5 < 1500:
                processed_status = "RC MANUAL OVERRIDE"
            elif gate_active:
                processed_status = "AI: GATE TRAVERSAL"
            elif vision_active:
                processed_status = "AI: DYNAMIC AVOIDANCE"
            elif self.current_state.recovering_from_avoidance:
                processed_status = "AI: RECOVERING"
            elif control_mode == "AUTO":
                if self.current_state.esp_status == "WP_COMPLETE":
                    processed_status = "MISI SELESAI"
                else:
                    idx = self.current_state.nav_target_wp_index
                    total = len(self.current_state.waypoints)
                    processed_status = f"WAYPOINT NAVIGATION ({idx}/{total})"
            else:
                processed_status = control_mode

            self.current_state.status = processed_status

            current_state_dict = asdict(self.current_state)
            for key, value in current_state_dict.items():
                if (
                    key not in self.last_emitted_state
                    or self.last_emitted_state[key] != value
                ):
                    short_key = TELEMETRY_KEY_MAP.get(key, key)
                    delta_payload[short_key] = value
                    self.last_emitted_state[key] = value

        if delta_payload:
            self.socketio.emit("telemetry_update", delta_payload)

    def _read_from_serial_loop(self):
        while self.running:
            data_processed = False
            while True:
                line = self.serial_handler.read_line()
                if not line:
                    break
                data_processed = True
                try:
                    data = json.loads(line)
                    self._parse_json_telemetry(data)
                except:
                    pass

            if data_processed:
                self.socketio.sleep(0)
            else:
                self.socketio.sleep(0.001)

    def _parse_json_telemetry(self, data):
        with self.state_lock:
            self.current_state.heading = data.get("heading", self.current_state.heading)
            self.current_state.speed = data.get("speed_kmh", 0.0) / 3.6
            self.current_state.latitude = data.get("lat", self.current_state.latitude)
            self.current_state.longitude = data.get("lon", self.current_state.longitude)
            self.current_state.nav_gps_sats = data.get("sats", 0)
            self.current_state.esp_status = data.get("status")
            self.current_state.rc_channels = data.get(
                "rc_ch", self.current_state.rc_channels
            )

            mode = data.get("mode")
            if mode == "AUTO":
                if data.get("status") == "WAYPOINT":
                    self.current_state.nav_target_wp_index = data.get(
                        "wp_target_idx", 0
                    )
                    self.current_state.nav_dist_to_wp = data.get("wp_dist_m", 0.0)
                    self.current_state.nav_heading_error = data.get("wp_error_hdg", 0.0)

    def main_logic_loop(self):
        self.socketio.start_background_task(self._read_from_serial_loop)
        logging.info("[AsvHandler] Logic loop dimulai.")

        while self.running:
            try:
                current_time = time.time()

                # Snapshot State
                with self.state_lock:
                    state = self.current_state
                    rc_mode = state.rc_channels[4]
                    control_mode = state.control_mode

                    # State AI
                    gate_active = state.gate_target.get("active", False)
                    gate_center = state.gate_target.get("center_x", 0)
                    gate_width = state.gate_target.get("frame_width", 640)

                    vision_active = state.vision_target.get("active", False)
                    vision_cls = state.vision_target.get("obstacle_class", "")
                    vision_center_x = state.vision_target.get("object_center_x", 0)
                    vision_frame_width = state.vision_target.get("frame_width", 640)

                    recovering = state.recovering_from_avoidance
                    resume_wp = state.resume_waypoint_on_clear

                    # Parameter Tuning
                    base_pwm_ai = state.vision_auto_motor_cmd  # Nilai dari Slider GUI

                    # Config
                    actuator_config = self.config.get("actuators", {})
                    servo_def = actuator_config.get("servo_default_angle", 90)
                    servo_min = actuator_config.get("servo_min_angle", 45)
                    servo_max = actuator_config.get("servo_max_angle", 135)

                    # Inversi & Arena (Arena B = Inverted)
                    is_arena_b = "b" in str(state.active_arena).lower()
                    is_inverted = is_arena_b

                command_to_send = None

                # --- LAYER 0: RC OVERRIDE ---
                if rc_mode < 1500:
                    command_to_send = None

                # --- LAYER 1: MANUAL GUI ---
                elif control_mode == "MANUAL":
                    command_to_send = (
                        f"A,{state.manual_servo_cmd},{state.manual_motor_cmd}\n"
                    )

                # --- LAYER 2: AUTO MODE ---
                elif control_mode == "AUTO":

                    # === PRIORITAS 1: GATE TRAVERSAL (DENGAN PID/MAP) ===
                    if gate_active:
                        with self.state_lock:
                            self.current_state.is_avoiding = True
                            self.current_state.recovering_from_avoidance = False

                        # Kejar tengah gate
                        frame_center = gate_width / 2
                        pixel_error = gate_center - frame_center

                        # Logika Map Value dari kode lama (Smooth)
                        correction = map_value(
                            pixel_error, -frame_center, frame_center, -35.0, 35.0
                        )

                        servo_out = int(
                            max(servo_min, min(servo_max, servo_def - correction))
                        )
                        pwm_out = base_pwm_ai - 75  # Pelan saat masuk gate

                        command_to_send = f"A,{servo_out},{int(pwm_out)}\n"
                        logging.info(
                            f"[GATE] Tracking Gate center: {gate_center}, Servo: {servo_out}"
                        )

                    # === PRIORITAS 2: DYNAMIC CONTEXTUAL AVOIDANCE ===
                    # (Menggunakan logika 'kode lama' + integrasi dynamic PWM)
                    elif vision_active:
                        with self.state_lock:
                            self.current_state.is_avoiding = True
                            self.current_state.recovering_from_avoidance = False

                        # 1. Tentukan Arah Hindar (Target X)
                        avoid_dir = "right"  # Default hindar kanan

                        if is_inverted:
                            # Arena B (Logika Terbalik)
                            if "green" in vision_cls:
                                avoid_dir = "left"
                            elif "red" in vision_cls:
                                avoid_dir = "right"
                        else:
                            # Arena A (Normal)
                            if "green" in vision_cls:
                                avoid_dir = "right"
                            elif "red" in vision_cls:
                                avoid_dir = "left"

                        # Target di 20% kiri atau 80% kanan frame
                        target_x = (
                            (vision_frame_width * 0.2)
                            if avoid_dir == "left"
                            else (vision_frame_width * 0.8)
                        )

                        # 2. Hitung Error & Koreksi (Proportional Control)
                        pixel_error = vision_center_x - target_x

                        # Konversi error pixel ke derajat (Max koreksi 45 derajat)
                        correction_deg = (pixel_error / (vision_frame_width / 2)) * 45.0

                        # 3. Hitung Servo
                        servo_cmd = int(
                            max(servo_min, min(servo_max, servo_def - correction_deg))
                        )

                        # 4. Hitung PWM Dinamis (Fitur dari kode lama)
                        # Semakin besar koreksi (belok tajam), semakin lambat motor
                        # base_pwm_ai diambil dari slider GUI
                        pwm_reduction = (
                            abs(correction_deg) * 3
                        )  # Faktor pengurangan (bisa di-tuning)
                        pwm_cmd = int(max(1300, base_pwm_ai - pwm_reduction))

                        command_to_send = f"A,{servo_cmd},{pwm_cmd}\n"
                        logging.info(
                            f"[AVOID] {avoid_dir.upper()} | Err: {pixel_error:.1f} | Srv: {servo_cmd} | PWM: {pwm_cmd}"
                        )

                    # === PRIORITAS 3: RECOVERY PHASE ===
                    elif recovering or resume_wp:
                        with self.state_lock:
                            self.current_state.is_avoiding = False
                            self.current_state.recovering_from_avoidance = False
                            self.current_state.resume_waypoint_on_clear = False
                        command_to_send = "W\n"
                        logging.info("[RECOVERY] Jalur bersih, kembali ke Waypoint.")

                    # === PRIORITAS 4: WAYPOINT NAVIGATION ===
                    else:
                        if state.esp_status == "WP_COMPLETE":
                            command_to_send = "W\n"
                        else:
                            command_to_send = "W\n"

                if command_to_send:
                    self.serial_handler.send_command(command_to_send)

                self.logger.log_telemetry(asdict(self.current_state))
                self._update_and_emit_state()
                send_telemetry_to_firebase(asdict(self.current_state), self.config)

            except Exception as e:
                logging.error(f"[AsvHandler] Error loop utama: {e}")

            self.socketio.sleep(0.02)

    def process_command(self, command, payload):
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control,
            "SET_WAYPOINTS": self._handle_set_waypoints,
            "NAV_START": self._handle_start_mission,
            "NAV_RETURN": self._handle_initiate_rth,
            "UPDATE_PID": self._handle_update_pid,
            "VISION_TARGET_UPDATE": self._handle_vision_target_update,
            "GATE_TRAVERSAL_COMMAND": self._handle_gate_traversal_command,
            "UPDATE_VISION_SPEED": self._handle_update_vision_speed,
            "UPDATE_VISION_SERVO": self._handle_update_vision_servo,
            "SET_PHOTO_MISSION": self._handle_set_photo_mission,
            "UPDATE_VISION_DISTANCE": self._handle_update_vision_distance_placeholder,
        }
        handler = command_handlers.get(command)
        if handler:
            handler(payload)
        else:
            logging.warning(f"Perintah tidak dikenal: {command}")

    def _handle_update_vision_distance_placeholder(self, payload):
        pass

    def _handle_gate_traversal_command(self, payload):
        with self.state_lock:
            active = payload.get("active", False)
            self.current_state.gate_target["active"] = active
            if active:
                self.current_state.gate_target.update(payload)
                self.current_state.recovering_from_avoidance = False

    def _handle_vision_target_update(self, payload):
        with self.state_lock:
            was_active = self.current_state.vision_target.get("active", False)
            is_active = payload.get("active", False)

            if was_active and not is_active:
                logging.info("[AsvHandler] Rintangan hilang -> Masuk fase Recovery.")
                self.current_state.recovering_from_avoidance = True
                self.current_state.last_avoidance_time = time.time()
                self.current_state.gate_target["active"] = False

            self.current_state.vision_target["active"] = is_active
            if is_active:
                self.current_state.vision_target.update(payload)

    def _handle_update_vision_speed(self, payload):
        val = int(payload.get("pwm", 1500))
        with self.state_lock:
            self.current_state.vision_auto_motor_cmd = val
        logging.info(f"Vision Speed Updated: {val}")

    def _handle_update_vision_servo(self, payload):
        left = int(payload.get("left", 45))
        right = int(payload.get("right", 135))
        with self.state_lock:
            self.current_state.vision_servo_left_cmd = left
            self.current_state.vision_servo_right_cmd = right
        logging.info(f"Vision Servo Updated: L={left}, R={right}")

    def _handle_serial_configuration(self, payload):
        port = payload.get("serial_port")
        baud = payload.get("baud_rate")
        if port == "AUTO":
            self.serial_handler.find_and_connect_esp32(baud)
        else:
            self.serial_handler.connect(port, baud)

    def _handle_mode_change(self, payload):
        mode = payload.get("mode", "MANUAL")
        with self.state_lock:
            self.current_state.control_mode = mode
        if mode == "MANUAL":
            self.serial_handler.send_command("A,90,1500\n")

    def _handle_manual_control(self, payload):
        keys = set(payload)
        pwm = 1500
        servo = 90
        if "W" in keys:
            pwm += 100
        if "S" in keys:
            pwm -= 100
        if "A" in keys:
            servo -= 45
        if "D" in keys:
            servo += 45

        with self.state_lock:
            self.current_state.manual_motor_cmd = pwm
            self.current_state.manual_servo_cmd = servo

    def _handle_set_waypoints(self, payload):
        wps = payload.get("waypoints", [])
        arena = payload.get("arena", "Unknown")
        with self.state_lock:
            self.current_state.waypoints = wps
            self.current_state.active_arena = arena
            self.current_state.current_waypoint_index = 0
        logging.info(f"Waypoints set: {len(wps)} points (Arena: {arena})")

    def _handle_start_mission(self, payload):
        with self.state_lock:
            self.current_state.control_mode = "AUTO"

    def _handle_initiate_rth(self, payload):
        with self.state_lock:
            self.current_state.control_mode = "AUTO"
            if self.current_state.waypoints:
                self.current_state.current_waypoint_index = 0

    def _handle_update_pid(self, payload):
        pass

    def _handle_set_photo_mission(self, payload):
        try:
            with self.state_lock:
                self.current_state.photo_mission_target_wp1 = int(
                    payload.get("wp1", -1)
                )
                self.current_state.photo_mission_target_wp2 = int(
                    payload.get("wp2", -1)
                )
                self.current_state.photo_mission_qty_requested = int(
                    payload.get("count", 0)
                )
                self.current_state.photo_mission_qty_taken_1 = 0
        except:
            pass

    def stop(self):
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        logging.info("[AsvHandler] Dihentikan.")
