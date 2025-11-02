# src/navantara_backend/core/asv_handler.py
import threading
import time
import math
import numpy as np
import json
import logging  # <-- PASTIKAN INI ADA

# [OPTIMASI] Impor untuk langkah optimasi berikutnya
from dataclasses import dataclass, asdict, field

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

        # --- [REFAKTOR ARSITEKTURAL] ---
        # State terpusat
        self.current_state = {
            "control_mode": "AUTO",
            "latitude": -6.9180,
            "longitude": 107.6185,
            "heading": 90.0,
            "cog": 0.0,
            "speed": 0.0,
            "battery_voltage": 12.5,
            "status": "DISCONNECTED",
            "mission_time": "00:00:00",
            "waypoints": [],
            "current_waypoint_index": 0,
            "is_connected_to_serial": False,
            "gyro_z": 0.0,
            "accel_x": 0.0,
            "rc_channels": [1500] * 6,
            "nav_target_wp_index": 0,
            "nav_dist_to_wp": 0.0,
            "nav_target_bearing": 0.0,
            "nav_heading_error": 0.0,
            "nav_servo_cmd": 90,
            "nav_motor_cmd": 1500,
            "nav_gps_sats": 0,
            "manual_servo_cmd": 90,
            "manual_motor_cmd": 1500,
            "active_arena": None,
            "debug_waypoint_counter": 0,
            "use_dummy_counter": False,
            "esp_status": None,
            "vision_target": {"active": False},
            "gate_target": {"active": False},
            "avoidance_direction": None,
            "is_avoiding": False,
            "gate_context": {"last_gate_config": None},
            "recovering_from_avoidance": False,
            "last_avoidance_time": 0,
            "last_pixel_error": 0,
            "resume_waypoint_on_clear": False,
            "inverse_servo": False,
        }
        # --- [AKHIR REFAKTOR] ---

        # --- [OPTIMASI 1: DELTA STATE] ---
        # Melacak state terakhir yang dikirim ke GUI
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
        # logging.info("[AsvHandler] Handler diinisialisasi untuk operasi backend.")
        self.initiate_auto_connection()

    def initiate_auto_connection(self):
        serial_cfg = self.config.get("serial_connection", {})
        force_port = serial_cfg.get("force_serial_port")
        baud_rate = serial_cfg.get("default_baud_rate", 115200)

        if force_port:
            self.use_dummy_serial = False
            self.serial_handler.use_dummy_serial = False
            # logging.info(f"[AsvHandler] Memaksa koneksi serial ke: {force_port} @ {baud_rate}")
            connected = self.serial_handler.connect(force_port, baud_rate)
            if not connected:
                logging.warning(
                    f"[AsvHandler] Gagal terhubung ke port paksa {force_port}. Melanjutkan auto-scan..."
                )

        if self.use_dummy_serial:
            # logging.info("[AsvHandler] Mode DUMMY SERIAL aktif.")
            return

        # logging.info("[AsvHandler] Memulai upaya koneksi serial otomatis...")
        self.serial_handler.find_and_connect_esp32(baud_rate)

    # --- [OPTIMASI 1: FUNGSI BARU _update_and_emit_state] ---
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
            # Ini jauh lebih cepat daripada menyalin seluruh kamus
            is_serial_connected = self.serial_handler.is_connected
            control_mode = self.current_state["control_mode"]
            gate_target_active = self.current_state["gate_target"].get("active", False)
            vision_target_active = self.current_state["vision_target"].get(
                "active", False
            )
            last_gate_config = self.current_state["gate_context"]["last_gate_config"]
            recovering = self.current_state["recovering_from_avoidance"]
            esp_status = self.current_state["esp_status"]
            waypoints = self.current_state["waypoints"]
            nav_target_wp_index = self.current_state["nav_target_wp_index"]
            rc_mode_switch = self.current_state["rc_channels"][4]

            # 2. Update status koneksi internal (GUI perlu ini)
            self.current_state["is_connected_to_serial"] = is_serial_connected

            # --- (SEMUA LOGIKA 'status' ANDA TETAP SAMA) ---
            if not is_serial_connected:
                processed_status = "DISCONNECTED (SERIAL)"
            elif rc_mode_switch < 1500:
                processed_status = "RC MANUAL OVERRIDE"
                # (Kita mungkin ingin memperbarui control_mode juga)
                # self.current_state["control_mode"] = "MANUAL"
            elif gate_target_active:
                processed_status = "AI: GATE TRAVERSAL"
            elif vision_target_active and last_gate_config:
                processed_status = f"AI: CONTEXTUAL AVOIDANCE ({last_gate_config})"
            elif vision_target_active:
                processed_status = "AI: SIMPLE AVOIDANCE"
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
                processed_status = control_mode  # "MANUAL"

            # 3. Tulis status yang baru diproses kembali ke state utama
            self.current_state["status"] = processed_status

            # 4. Bangun "delta" (perbedaan)
            for key, value in self.current_state.items():
                if (
                    key not in self.last_emitted_state
                    or self.last_emitted_state[key] != value
                ):
                    delta_payload[key] = value

            # 5. Update 'last_emitted_state' HANYA dengan apa yang berubah
            if delta_payload:
                self.last_emitted_state.update(delta_payload)
            # --- [LOCK DILEPAS] ---

        # 6. Kirim HANYA 'delta_payload', bukan seluruh state
        if delta_payload:  # Hanya kirim jika ada perubahan
            self.socketio.emit("telemetry_update", delta_payload)

    # --- [AKHIR OPTIMASI 1] ---

    def _read_from_serial_loop(self):
        while self.running:
            line = self.serial_handler.read_line()
            if line:
                # logging.info(f"[Serial RAW] {line}")
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

    def _parse_json_telemetry(self, data):
        try:
            with self.state_lock:
                self.current_state["heading"] = data.get(
                    "heading", self.current_state["heading"]
                )
                self.current_state["speed"] = data.get("speed_kmh", 0.0) / 3.6
                self.current_state["nav_gps_sats"] = data.get(
                    "sats", self.current_state["nav_gps_sats"]
                )
                self.current_state["latitude"] = data.get(
                    "lat", self.current_state["latitude"]
                )
                self.current_state["longitude"] = data.get(
                    "lon", self.current_state["longitude"]
                )
                status_val = data.get("status", None)
                self.current_state["esp_status"] = status_val
                # HAPUS: 'status' sekarang dihitung di _update_and_emit_state
                # if status_val is not None:
                #     self.current_state["status"] = status_val
                self.current_state["rc_channels"] = data.get(
                    "rc_ch", self.current_state["rc_channels"]
                )
                mode = data.get("mode")
                if mode == "MANUAL":
                    self.current_state["manual_servo_cmd"] = data.get("servo_out")
                    self.current_state["manual_motor_cmd"] = data.get("motor_out")
                elif mode == "AUTO":
                    status = data.get("status")
                    if status == "WAYPOINT":
                        self.current_state["nav_target_wp_index"] = data.get(
                            "wp_target_idx"
                        )
                        self.current_state["nav_dist_to_wp"] = data.get("wp_dist_m")
                        self.current_state["nav_target_bearing"] = data.get(
                            "wp_target_brg"
                        )
                        self.current_state["nav_heading_error"] = data.get(
                            "wp_error_hdg"
                        )
                        self.current_state["nav_servo_cmd"] = data.get("servo_out")
                        self.current_state["nav_motor_cmd"] = data.get("motor_out")
                    elif status == "AI_ACTIVE":
                        self.current_state["nav_servo_cmd"] = data.get("servo_out")
                        self.current_state["nav_motor_cmd"] = data.get("motor_out")
        except Exception as e:
            logging.error(
                f"[AsvHandler] Gagal mem-parsing data JSON: {e}. Data: {data}"
            )

    # --- [OPTIMASI 2: FUNGSI BARU main_logic_loop] ---
    def main_logic_loop(self):
        self.socketio.start_background_task(self._read_from_serial_loop)
        # logging.info("[AsvHandler] Loop pembaca serial dimulai.")
        while self.running:
            try:
                # Coba reconnect jika dalam mode AUTO dan tidak terhubung
                current_time = time.time()
                with self.state_lock:
                    is_auto = self.current_state["control_mode"] == "AUTO"

                if (
                    is_auto
                    and not self.serial_handler.is_connected
                    and current_time - self.last_reconnect_attempt
                    > self.reconnect_interval
                ):
                    self.last_reconnect_attempt = current_time
                    # logging.info("[AsvHandler] Mode AUTO aktif, mencoba koneksi ulang ke ESP32...")
                    baud_rate = self.config.get("serial_connection", {}).get(
                        "default_baud_rate", 115200
                    )
                    self.serial_handler.find_and_connect_esp32(baud_rate)

                # --- (Blok EKF tetap sama) ---
                current_time = time.time()
                dt = current_time - self.last_ekf_update_time
                if dt > 0:
                    self.last_ekf_update_time = current_time
                    with self.state_lock:
                        heading_rad = np.radians(self.current_state["heading"])
                        speed_ms = self.current_state["speed"]
                        gyro_z_rad = np.radians(self.current_state["gyro_z"])
                    self.ekf.predict(dt)
                    self.ekf.update_compass(heading_rad)
                    self.ekf.update_imu(np.array([speed_ms, gyro_z_rad]))
                    with self.state_lock:
                        if not self.serial_handler.is_connected:
                            self.current_state["heading"] = (
                                np.degrees(self.ekf.state[2]) + 360
                            ) % 360
                # --- (Akhir Blok EKF) ---

                # --- [OPTIMASI 2: MENGURANGI DURASI LOCK] ---
                # Ambil HANYA data yang diperlukan untuk keputusan logika
                # Ini jauh lebih cepat daripada .copy() dan melepas lock lebih cepat
                with self.state_lock:
                    rc_mode_switch = self.current_state["rc_channels"][4]
                    control_mode = self.current_state["control_mode"]
                    manual_servo_cmd = self.current_state["manual_servo_cmd"]
                    manual_motor_cmd = self.current_state["manual_motor_cmd"]

                    waypoints = self.current_state["waypoints"]
                    current_waypoint_index = self.current_state[
                        "current_waypoint_index"
                    ]

                    gate_target_active = self.current_state["gate_target"].get(
                        "active", False
                    )
                    gate_target_frame_width = self.current_state["gate_target"].get(
                        "frame_width", 640
                    )
                    gate_target_center_x = self.current_state["gate_target"].get(
                        "gate_center_x"
                    )
                    gate_target_red_x = self.current_state["gate_target"].get(
                        "red_buoy_x"
                    )
                    gate_target_green_x = self.current_state["gate_target"].get(
                        "green_buoy_x"
                    )

                    vision_target_active = self.current_state["vision_target"].get(
                        "active", False
                    )
                    vision_target_frame_width = self.current_state["vision_target"].get(
                        "frame_width", 640
                    )
                    vision_target_obj_center_x = self.current_state[
                        "vision_target"
                    ].get("object_center_x")
                    vision_target_obj_class = self.current_state["vision_target"].get(
                        "obstacle_class", ""
                    )

                    gate_context_last_config = self.current_state["gate_context"].get(
                        "last_gate_config"
                    )

                    resume_waypoint_on_clear = self.current_state[
                        "resume_waypoint_on_clear"
                    ]
                    nav_dist_to_wp = self.current_state["nav_dist_to_wp"]
                    esp_status = self.current_state["esp_status"]
                    status = self.current_state["status"]  # Untuk cek WP_COMPLETE
                    is_inverted = self.current_state.get("inverse_servo", False)
                # --- [LOCK DILEPAS] ---

                command_to_send = None

                # PRIORITAS 1: RC OVERRIDE
                if rc_mode_switch < 1500:
                    command_to_send = None
                    # logging.info("[AsvHandler] RC OVERRIDE -> Kontrol Jetson ditahan.")

                # PRIORITAS 2: MANUAL GUI (WASD)
                elif control_mode == "MANUAL":
                    command_to_send = (
                        f"A,{int(manual_servo_cmd)},{int(manual_motor_cmd)}\n"
                    )
                    # logging.info(f"[AsvHandler] MANUAL CONTROL -> Servo: {int(manual_servo_cmd)} deg, Motor: {int(manual_motor_cmd)} us")

                # PRIORITAS 3: AUTO (AI & WAYPOINT)
                elif control_mode == "AUTO":
                    mission_completed = bool(
                        waypoints and current_waypoint_index >= len(waypoints)
                    )
                    if mission_completed:
                        command_to_send = "W\n"

                    actuator_config = self.config.get("actuators", {})
                    servo_default = actuator_config.get("servo_default_angle", 90)
                    servo_min = actuator_config.get("servo_min_angle", 45)
                    servo_max = actuator_config.get("servo_max_angle", 135)
                    motor_base = actuator_config.get("motor_pwm_auto_base", 1300)

                    # === PRIORITAS 3.1: MELEWATI GERBANG (GATE TRAVERSAL) ===
                    if gate_target_active:
                        with self.state_lock:
                            self.current_state["recovering_from_avoidance"] = False
                            self.current_state["is_avoiding"] = False
                            self.current_state["avoidance_direction"] = None

                        frame_width = gate_target_frame_width
                        gate_center_x = gate_target_center_x
                        red_buoy_x = gate_target_red_x
                        green_buoy_x = gate_target_green_x

                        if red_buoy_x is not None and green_buoy_x is not None:
                            with self.state_lock:
                                if red_buoy_x < green_buoy_x:
                                    self.current_state["gate_context"][
                                        "last_gate_config"
                                    ] = "red_left_green_right"
                                else:
                                    self.current_state["gate_context"][
                                        "last_gate_config"
                                    ] = "green_left_red_right"

                        if gate_center_x is not None:
                            pixel_error = gate_center_x - (frame_width / 2)
                            correction = map_value(
                                pixel_error,
                                -frame_width / 2,
                                frame_width / 2,
                                -35.0,
                                35.0,
                            )

                            # Terapkan inversi
                            if is_inverted:
                                correction = -correction

                            servo_cmd = int(
                                max(
                                    servo_min,
                                    min(servo_max, servo_default - correction),
                                )
                            )
                            pwm_cmd = motor_base - 75

                            # Cek jarak WP
                            if nav_dist_to_wp < 1.5:
                                command_to_send = "W\n"
                                # logging.info("[AsvHandler] AI GATE: Jarak WP < 1.5m. Melepas ke Waypoint Nav.")
                            else:
                                command_to_send = f"A,{servo_cmd},{int(pwm_cmd)}\n"
                                # logging.info(f"[AsvHandler] AI CONTROL [Gate] -> Servo: {servo_cmd} deg, Motor: {int(pwm_cmd)} us")

                    # === PRIORITAS 3.2: PENGHINDARAN TUNGGAL BERKONTEKS ===
                    elif vision_target_active and gate_context_last_config:
                        with self.state_lock:
                            self.current_state["recovering_from_avoidance"] = False
                            self.current_state["is_avoiding"] = True

                        frame_width = vision_target_frame_width
                        obj_center_x = vision_target_obj_center_x
                        obj_class = vision_target_obj_class
                        last_config = gate_context_last_config
                        target_side = None

                        if last_config == "red_left_green_right":
                            if obj_class == "red_buoy":
                                target_side = "left"
                            elif obj_class == "green_buoy":
                                target_side = "right"
                        elif last_config == "green_left_red_right":
                            if obj_class == "green_buoy":
                                target_side = "left"
                            elif obj_class == "red_buoy":
                                target_side = "right"

                        if target_side:
                            target_x = (
                                frame_width * 0.2
                                if target_side == "left"
                                else frame_width * 0.8
                            )
                            pixel_error = obj_center_x - target_x
                            correction_deg = (pixel_error / (frame_width / 2)) * 45.0

                            if is_inverted:
                                correction_deg = -correction_deg

                            servo_cmd = int(
                                max(
                                    servo_min,
                                    min(servo_max, servo_default - correction_deg),
                                )
                            )
                            pwm_cmd = int(
                                max(1300, motor_base - abs(correction_deg) * 2)
                            )

                            if nav_dist_to_wp < 1.5:
                                command_to_send = "W\n"
                                # logging.info("[AsvHandler] AI CTX: Jarak WP < 1.5m. Melepas ke Waypoint Nav.")
                            elif esp_status == "WP_COMPLETE" or status == "WP_COMPLETE":
                                command_to_send = "W\n"
                                # logging.info("[AsvHandler] WP_COMPLETE dilaporkan oleh ESP32 -> mengirim W untuk melanjutkan waypoint")
                            else:
                                command_to_send = f"A,{servo_cmd},{pwm_cmd}\n"
                                # logging.info(f"[AsvHandler] AI CONTROL [Avoid Ctx] -> Servo: {servo_cmd} deg, Motor: {int(pwm_cmd)} us")
                        else:
                            with self.state_lock:
                                self.current_state["gate_context"][
                                    "last_gate_config"
                                ] = None

                    # === PRIORITAS 3.3: PENGHINDARAN TUNGGAL SEDERHANA (TANPA KONTEKS) ===
                    elif vision_target_active:
                        with self.state_lock:
                            self.current_state["recovering_from_avoidance"] = False
                            self.current_state["is_avoiding"] = True

                        obj_class = vision_target_obj_class

                        angle_green_buoy = 50
                        angle_red_buoy = 130

                        if is_inverted:
                            if obj_class == "green_buoy":
                                servo_cmd = angle_red_buoy
                            elif obj_class == "red_buoy":
                                servo_cmd = angle_green_buoy
                            else:
                                servo_cmd = servo_default
                        else:
                            if obj_class == "green_buoy":
                                servo_cmd = angle_green_buoy
                            elif obj_class == "red_buoy":
                                servo_cmd = angle_red_buoy
                            else:
                                servo_cmd = servo_default

                        pwm_cmd = motor_base + 50

                        if nav_dist_to_wp < 1.5:
                            command_to_send = "W\n"
                            # logging.info("[AsvHandler] AI SIMPLE: Jarak WP < 1.5m. Melepas ke Waypoint Nav.")
                        elif esp_status == "WP_COMPLETE" or status == "WP_COMPLETE":
                            command_to_send = "W\n"
                            # logging.info("[AsvHandler] WP_COMPLETE dilaporkan -> mengirim W untuk melanjutkan waypoint")
                        else:
                            command_to_send = f"A,{servo_cmd},{int(pwm_cmd)}\n"
                            # logging.info(f"[AsvHandler] AI CONTROL [Avoid] -> Servo diputar ke: {servo_cmd}° untuk {obj_class}")

                    # === PRIORITAS 3.4: TRANSISI INSTAN KE WAYPOINT ===
                    elif resume_waypoint_on_clear:
                        with self.state_lock:
                            self.current_state.update(
                                {
                                    "is_avoiding": False,
                                    "avoidance_direction": None,
                                    "recovering_from_avoidance": False,
                                    "resume_waypoint_on_clear": False,
                                }
                            )
                        command_to_send = "W\n"
                        # logging.info("[AsvHandler] Transisi cepat -> waypoint mode")

                    # === PRIORITAS 3.5 (DEFAULT): NAVIGASI WAYPOINT ===
                    else:
                        with self.state_lock:
                            self.current_state["last_pixel_error"] = 0
                        if self.serial_handler.is_connected:
                            command_to_send = "W\n"
                            # logging.info("[AsvHandler] WAYPOINT CONTROL -> Mengirim: W")
                        else:
                            command_to_send = None
                            # logging.info("[AsvHandler] WAYPOINT CONTROL -> Menunggu koneksi serial...")

                # --- (Blok kirim perintah tetap sama) ---
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
                            # logging.info("[AsvHandler] Mission complete detected -> forcing W and suppressing actuator commands")

                if command_to_send is None and resume_waypoint_on_clear:
                    if self.serial_handler.is_connected and control_mode == "AUTO":
                        command_to_send = "W\n"
                        # logging.info("[AsvHandler] Vision cleared -> sending resume W to ESP32")
                        with self.state_lock:
                            self.current_state["resume_waypoint_on_clear"] = False

                if command_to_send:
                    self.serial_handler.send_command(command_to_send)

                # --- [OPTIMASI 2: Buat salinan HANYA untuk logging] ---
                # Fungsi _update_and_emit_state sekarang menangani statenya sendiri
                # Kita hanya perlu salinan untuk logger dan firebase.
                with self.state_lock:
                    state_for_log = self.current_state.copy()

                self.logger.log_telemetry(state_for_log)
                self._update_and_emit_state()  # Memanggil fungsi delta-state baru
                send_telemetry_to_firebase(state_for_log, self.config)

            except Exception as e:
                logging.error(f"[FATAL] Error di main_logic_loop: {e}", exc_info=True)

            self.socketio.sleep(0.02)

    # --- [AKHIR OPTIMASI 2] ---

    def process_command(self, command, payload):
        # (Log debug sudah dihapus sesuai permintaan)
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
            "GATE_TRAVERSAL_COMMAND": self._handle_gate_traversal_command,
            "DEBUG_WP_COUNTER": self._handle_debug_counter,
            "INVERSE_SERVO": self._handle_inverse_servo,
            "SET_INVERSION": self._handle_set_inversion,  # (Handler inversi baru)
        }
        handler = command_handlers.get(command)
        if handler:
            handler(payload)
        else:
            logging.warning(
                f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'"
            )

    # (Handler baru dari diskusi kita sebelumnya)
    def _handle_set_inversion(self, payload):
        """Menangani toggling mode inversi dari GUI."""
        with self.state_lock:
            new_state = payload.get("inverted", False)
            if self.current_state["inverse_servo"] != new_state:
                self.current_state["inverse_servo"] = new_state
                # logging.info(f"[AsvHandler] Kontrol Inversi diatur ke: {new_state}")

    def _handle_inverse_servo(self, payload):
        """Handler untuk mengubah arah servo (toggle antara normal dan terbalik)"""
        with self.state_lock:
            if payload.get("toggle"):
                self.current_state["inverse_servo"] = not self.current_state.get(
                    "inverse_servo", False
                )
            elif "value" in payload:
                self.current_state["inverse_servo"] = bool(payload["value"])
            # logging.info(f"[AsvHandler] inverse_servo diubah ke: {self.current_state['inverse_servo']}")

    def _handle_gate_traversal_command(self, payload):
        with self.state_lock:
            was_active = self.current_state["gate_target"].get("active", False)
            is_active = payload.get("active", False)
            if was_active and not is_active:
                # logging.info("[AsvHandler] Gate traversal complete. Initiating recovery...")
                self.current_state["recovering_from_avoidance"] = True
                self.current_state["last_avoidance_time"] = time.time()

            self.current_state["gate_target"]["active"] = is_active
            if is_active:
                self.current_state["gate_target"].update(payload)
                self.current_state["vision_target"]["active"] = False

    def _handle_debug_counter(self, payload):
        action = payload.get("action")
        with self.state_lock:
            self.current_state["use_dummy_counter"] = True
            if action == "INC":
                self.current_state["debug_waypoint_counter"] += 1
            elif action == "DEC":
                self.current_state["debug_waypoint_counter"] = max(
                    0, self.current_state["debug_waypoint_counter"] - 1
                )
            elif action == "RESET":
                self.current_state["debug_waypoint_counter"] = 0
            max_points_in_monitor = 9
            self.current_state["debug_waypoint_counter"] = min(
                self.current_state["debug_waypoint_counter"], max_points_in_monitor
            )
            # logging.info(f"[AsvHandler] Debug counter diatur ke: {self.current_state['debug_waypoint_counter']}")

    def _handle_vision_target_update(self, payload):
        with self.state_lock:
            if self.current_state["gate_target"].get("active"):
                return
            was_active = self.current_state["vision_target"].get("active")
            is_active = payload.get("active")
            if was_active and not is_active:
                # logging.info("[AsvHandler] Deteksi selesai -> langsung ke waypoint")
                self.current_state.update(
                    {
                        "recovering_from_avoidance": False,
                        "is_avoiding": False,
                        "avoidance_direction": None,
                        "resume_waypoint_on_clear": True,
                    }
                )
            self.current_state["vision_target"]["active"] = is_active
            if is_active:
                self.current_state["vision_target"].update(payload)

    # --- [OPTIMASI 2: FUNGSI BARU _handle_manual_control] ---
    def _handle_manual_control(self, payload):
        # Ambil nilai yang diperlukan dari state DENGAN lock
        with self.state_lock:
            rc_channel_5 = self.current_state["rc_channels"][4]
            control_mode = self.current_state["control_mode"]
            is_inverted = self.current_state.get("inverse_servo", False)

        # Logika keputusan bisa di luar lock
        if rc_channel_5 < 1500:
            return
        if control_mode != "MANUAL":
            return

        # Logika perhitungan
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

        # Tulis kembali hasil ke state DENGAN lock
        with self.state_lock:
            self.current_state["manual_servo_cmd"] = int(servo)
            self.current_state["manual_motor_cmd"] = int(pwm)

    # --- [AKHIR OPTIMASI 2] ---

    def set_streaming_status(self, status: bool):
        if self.is_streaming_to_gui != status:
            # logging.info(f"[AsvHandler] Status streaming telemetri diatur ke: {status}")
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
            # logging.info(f"[AsvHandler] PID updated: P={kp}, I={ki}, D={kd}")

    def _handle_serial_configuration(self, payload):
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO":
            self.serial_handler.find_and_connect_esp32(baud)
        else:
            self.serial_handler.connect(port, baud)

    def _handle_mode_change(self, payload):
        with self.state_lock:
            new_mode = payload.get("mode", "MANUAL")
            self.current_state["control_mode"] = new_mode
            # Simpan log
            self.logger.log_event(f"Mode kontrol GUI diubah ke: {new_mode}")

        if new_mode == "MANUAL":
            actuator_config = self.config.get("actuators", {})
            pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
            servo_def = actuator_config.get("servo_default_angle", 90)
            command_str = f"A,{int(servo_def)},{int(pwm_stop)}\n"
            # logging.info(f"[LOG | MODE] GUI ganti ke MANUAL, kirim netral: {command_str.strip()}")
            self.serial_handler.send_command(command_str)

    def _handle_set_waypoints(self, payload):
        waypoints_data = payload.get("waypoints")
        arena_id = payload.get("arena")
        if not isinstance(waypoints_data, list):
            logging.warning("[AsvHandler] Gagal set waypoints: Data tidak valid.")
            return
        with self.state_lock:
            self.current_state["waypoints"] = waypoints_data
            self.current_state["current_waypoint_index"] = 0
            self.current_state["active_arena"] = arena_id
            self.logger.log_event(
                f"Waypoints baru dimuat (Arena: {arena_id}). Jumlah: {len(waypoints_data)}"
            )

    def _handle_start_mission(self, payload):
        with self.state_lock:
            if not self.current_state["waypoints"]:
                return
            self.current_state["control_mode"] = "AUTO"
            self.current_state["current_waypoint_index"] = 0
            self.current_state["use_dummy_counter"] = False
        self.logger.log_event("Misi navigasi dimulai.")

    def _handle_initiate_rth(self, payload):
        with self.state_lock:
            if not self.current_state["waypoints"]:
                return
            self.current_state["waypoints"] = [self.current_state["waypoints"][0]]
            self.current_state["current_waypoint_index"] = 0
            self.current_state["control_mode"] = "AUTO"
        self.logger.log_event("Memulai Return to Home.")

    def stop(self):
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        # logging.info("[AsvHandler] Dihentikan.")
