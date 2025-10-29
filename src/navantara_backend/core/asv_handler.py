# src/navantara_backend/core/asv_handler.py
# --- VERSI FINAL DENGAN HIRARKI KONTROL KONTEKSTUAL & STATEFUL ---
import threading
import time
import math
import numpy as np

from navantara_backend.services.serial_service import SerialHandler
from navantara_backend.core.navigation import (
    run_navigation_logic,
    PIDController,
    haversine_distance,
)
from navantara_backend.core.kalman_filter import SimpleEKF
from navantara_backend.core.mission_logger import MissionLogger
# --- MODIFIKASI: Import Firebase di-comment ---
# from navantara_backend.vision.cloud_utils import send_telemetry_to_firebase
# --- AKHIR MODIFIKASI ---


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
            # --- TAMBAHAN BARU ---
            "manual_servo_cmd": 90,
            "manual_motor_cmd": 1500,
            "active_arena": None,
            "debug_waypoint_counter": 0,  # Counter dummy
            "use_dummy_counter": False,  # Flag untuk mode debug
            # --- AKHIR TAMBAHAN ---
        }

        # Inisialisasi state untuk setiap mode AI
        self.vision_target = {"active": False}
        self.gate_target = {"active": False}
        self.last_pixel_error = 0

        self.avoidance_direction = None  # 'left' atau 'right'
        self.is_avoiding = False

        # --- MANAJEMEN STATE BARU UNTUK KONTEKS GERBANG ---
        # Kamus ini berfungsi sebagai "memori" ASV.
        # 'last_gate_config' akan menyimpan konfigurasi gerbang terakhir yang terlihat.
        # Nilai yang mungkin: None, 'red_left_green_right', 'green_left_red_right'
        self.gate_context = {
            "last_gate_config": None,
        }
        # --- AKHIR MANAJEMEN STATE BARU ---

        # Variabel untuk manuver transisi (recovery)
        self.recovering_from_avoidance = False
        self.last_avoidance_time = 0

        pid_config = self.config.get("navigation", {}).get("heading_pid", {})
        self.pid_controller = PIDController(
            Kp=pid_config.get("kp", 1.2),
            Ki=pid_config.get("ki", 0.1),
            Kd=pid_config.get("kd", 0.05),
        )
        self.ekf = SimpleEKF(np.zeros(5), np.eye(5) * 0.1)
        self.last_ekf_update_time = time.time()

        self.use_dummy_serial = self.config.get("serial_connection", {}).get("use_dummy_serial", False)

        self.logger = MissionLogger()
        self.logger.log_event("AsvHandler diinisialisasi.")
        print("[AsvHandler] Handler diinisialisasi untuk operasi backend.")

        self.initiate_auto_connection()

    def initiate_auto_connection(self):
        
        if self.use_dummy_serial:
            print("[AsvHandler] Mode DUMMY SERIAL aktif. Koneksi serial fisik dilewati.")
        return
    
        print("[AsvHandler] Memulai upaya koneksi serial otomatis...")
        baud_rate = self.config.get("serial_connection", {}).get(
            "default_baud_rate", 115200
        )
        self.serial_handler.find_and_connect_esp32(baud_rate)

    def _update_and_emit_state(self):
        # === MODIFIKASI: Hapus cek 'self.is_streaming_to_gui' ===
        # if self.running and self.is_streaming_to_gui: (LAMA)
        if self.running: # (BARU)
        # === AKHIR MODIFIKASI ===
            with self.state_lock:
                state_copy = self.current_state.copy()
                state_copy["is_connected_to_serial"] = self.serial_handler.is_connected
                if not self.serial_handler.is_connected:
                    state_copy["status"] = "DISCONNECTED (SERIAL)"

                rc_mode_switch = state_copy.get("rc_channels", [1500] * 6)[4]
                # Hierarki status untuk ditampilkan di GUI
                if rc_mode_switch < 1500:
                    state_copy["status"] = "RC MANUAL OVERRIDE"
                    state_copy["control_mode"] = "MANUAL"
                elif self.gate_target["active"]:
                    state_copy["status"] = "AI: GATE TRAVERSAL"
                elif (
                    self.vision_target["active"]
                    and self.gate_context["last_gate_config"]
                ):
                    state_copy["status"] = (
                        f"AI: CONTEXTUAL AVOIDANCE ({self.gate_context['last_gate_config']})"
                    )
                elif self.vision_target["active"]:
                    state_copy["status"] = "AI: SIMPLE AVOIDANCE"
                elif self.recovering_from_avoidance:
                    state_copy["status"] = "RECOVERING: STRAIGHTENING COURSE"
                elif state_copy["control_mode"] == "AUTO":
                    wp_idx = state_copy.get("current_waypoint_index", 0)
                    total_wps = len(state_copy.get("waypoints", []))
                    if total_wps > 0 and wp_idx < total_wps:
                        state_copy["status"] = (
                            f"WAYPOINT NAVIGATION ({wp_idx + 1}/{total_wps})"
                        )
                    else:
                        state_copy["status"] = "AUTO IDLE"
                else:
                    # Ini akan menangkap mode MANUAL dari GUI
                    state_copy["status"] = state_copy.get("control_mode", "MANUAL")

            self.socketio.emit("telemetry_update", state_copy)

    def _read_from_serial_loop(self):
        while self.running:
            line = self.serial_handler.read_line()

            # --- MODIFIKASI: Tambahkan parser untuk "DATA:MANUAL," ---
            if line:  # Hanya proses jika 'line' tidak kosong
                if line.startswith("T:"):
                    print(f"[Serial] Menerima Telemetri ESP: {line}")
                    self._parse_telemetry(line)

                elif line.startswith("DATA:AUTO,WAYPOINT,"):
                    print(f"[Serial] Menerima Data Waypoint ESP: {line}")
                    self._parse_waypoint_data(line)

                elif line.startswith("DATA:MANUAL,"):
                    print(f"[Serial] Menerima Data Manual ESP: {line}")
                    self._parse_manual_data(line)

                # Menangkap pesan AI_MODE_ACTIVATED
                elif "AI_MODE_ACTIVATED" in line:
                    print(f"[Serial] Konfirmasi ESP: {line}")

                else:
                    # Hanya cetak jika itu bukan string kosong
                    if line.strip():
                        print(f"[Serial] Menerima data TIDAK DIKENAL: {line}")

            # Selalu beri jeda singkat di setiap loop
            self.socketio.sleep(0.02)
            # --- AKHIR MODIFIKASI ---

    def _parse_telemetry(self, line):
        try:
            with self.state_lock:
                parts = line.strip("T:").split(";")
                for part in parts:
                    if not part:
                        continue
                    data = part.split(",")
                    data_type = data[0]
                    if data_type == "GPS" and len(data) == 3:
                        (
                            self.current_state["latitude"],
                            self.current_state["longitude"],
                        ) = float(data[1]), float(data[2])
                    elif data_type == "COMP" and len(data) == 2:
                        self.current_state["heading"] = float(data[1])
                    elif data_type == "SPD" and len(data) == 2:
                        self.current_state["speed"] = float(data[1])
                    elif data_type == "RC" and len(data) == 7:
                        self.current_state["rc_channels"] = [
                            int(val) for val in data[1:]
                        ]
        except (ValueError, IndexError, TypeError):
            pass

    def _parse_waypoint_data(self, line):
        """
        Mem-parsing data telemetri navigasi khusus dari firmware.
        Format: "DATA:AUTO,WAYPOINT,[counter+1],[dist],[targetBearing],[heading],
                 [errorHeading],[servoPos],[motorSpeed],[speed],[sats]"
        """
        try:
            parts = line.split(",")

            if len(parts) != 13:
                print(
                    f"[AsvHandler] Peringatan: Format data waypoint tidak dikenal. {line}"
                )
                return

            with self.state_lock:
                self.current_state["nav_target_wp_index"] = int(parts[2])
                self.current_state["nav_dist_to_wp"] = float(parts[3])
                self.current_state["nav_target_bearing"] = float(parts[4])
                self.current_state["heading"] = float(parts[5])  # Update heading
                self.current_state["nav_heading_error"] = float(parts[6])
                self.current_state["nav_servo_cmd"] = int(parts[7])
                self.current_state["nav_motor_cmd"] = int(parts[8])
                # Konversi speed dari kmph ke m/s
                self.current_state["speed"] = float(parts[9]) / 3.6
                self.current_state["nav_gps_sats"] = int(parts[10])
                self.current_state["latitude"] = float(parts[11])
                self.current_state["longitude"] = float(parts[12])

        except (ValueError, IndexError, TypeError) as e:
            print(f"[AsvHandler] Gagal mem-parsing data waypoint: {e}. Data: {line}")

    # --- TAMBAHAN: Fungsi Parser Baru ---
    # --- TAMBAHAN: Fungsi Parser Baru ---
    def _parse_manual_data(self, line):
        """
        Mem-parsing data telemetri mode manual.
        Format DARI LOG: "DATA:MANUAL,[heading],[speed],[sats],[servoPos],[motorMicros],[lat],[lon]"
        """
        try:
            parts = line.split(",")

            # --- PERBAIKAN: Ubah cek dari 6 ke 8 ---
            if len(parts) != 8:
                print(
                    f"[AsvHandler] Peringatan: Format data manual tidak dikenal (diharapkan 8, didapat {len(parts)}). {line}"
                )
                return

            # parts[0] = "DATA:MANUAL"
            # parts[1] = heading
            # parts[2] = speed (kmph)
            # parts[3] = sats
            # parts[4] = servoPos
            # parts[5] = motorMicros
            # parts[6] = latitude (dari log)
            # parts[7] = longitude (dari log)

            with self.state_lock:
                self.current_state["heading"] = float(parts[1])
                # Konversi speed dari kmph (sesuai ESP32) ke m/s (sesuai GUI)
                self.current_state["speed"] = float(parts[2]) / 3.6
                self.current_state["nav_gps_sats"] = int(parts[3])
                self.current_state["manual_servo_cmd"] = int(parts[4])
                self.current_state["manual_motor_cmd"] = int(parts[5])

                # --- TAMBAHAN: Parse data lat/lon yang baru ---
                self.current_state["latitude"] = float(parts[6])
                self.current_state["longitude"] = float(parts[7])
                # --- AKHIR TAMBAHAN ---

        except (ValueError, IndexError, TypeError) as e:
            print(f"[AsvHandler] Gagal mem-parsing data manual: {e}. Data: {line}")
        # --- AKHIR TAMBAHAN ---
        pass

    def main_logic_loop(self):
        self.socketio.start_background_task(self._read_from_serial_loop)
        print("[AsvHandler] Loop pembaca serial dimulai.")
        while self.running:
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
                    # Hanya update dari EKF jika tidak ada data serial
                    if not self.serial_handler.is_connected:
                        self.current_state["heading"] = (
                            np.degrees(self.ekf.state[2]) + 360
                        ) % 360

            with self.state_lock:
                state_for_logic = self.current_state.copy()

            rc_mode_switch = state_for_logic.get("rc_channels", [1500] * 6)[4]
            command_to_send = None

            # Jika RC mengambil alih, jangan kirim perintah apapun
            if rc_mode_switch < 1500:
                command_to_send = None  # ESP32 akan menangani logika RC secara internal

            # Jika mode GUI adalah MANUAL, kirim perintah manual
            elif state_for_logic.get("control_mode") == "MANUAL":
                # Perintah manual (dari WASD) ditangani oleh _handle_manual_control
                # Jadi kita tidak perlu mengirim apa-apa di loop utama
                command_to_send = None

            # Jika mode GUI adalah AUTO
            elif state_for_logic.get("control_mode") == "AUTO":
                # --- IMPLEMENTASI HIRARKI KEPUTUSAN MODE AUTO ---
                waypoints = state_for_logic.get("waypoints", [])
                wp_index = state_for_logic.get("current_waypoint_index", 0)
                mission_completed = bool(waypoints and wp_index >= len(waypoints))

                if mission_completed:
                    command_to_send = "W\n"
                    print("[AsvHandler] Misi Selesai. Mengirim 'W' (idle).")

                actuator_config = self.config.get("actuators", {})
                servo_default = actuator_config.get("servo_default_angle", 90)
                servo_min = actuator_config.get("servo_min_angle", 45)
                servo_max = actuator_config.get("servo_max_angle", 135)
                motor_base = actuator_config.get("motor_pwm_auto_base", 1300)

                # === PRIORITAS 1: MELEWATI GERBANG (GATE TRAVERSAL) ===
                if self.gate_target.get("active", False):
                    self.recovering_from_avoidance = False
                    self.is_avoiding = False
                    self.avoidance_direction = None

                    frame_width = self.gate_target.get("frame_width", 640)
                    gate_center_x = self.gate_target.get("gate_center_x")
                    red_buoy_x = self.gate_target.get("red_buoy_x")
                    green_buoy_x = self.gate_target.get("green_buoy_x")

                    if red_buoy_x is not None and green_buoy_x is not None:
                        if red_buoy_x < green_buoy_x:
                            self.gate_context["last_gate_config"] = (
                                "red_left_green_right"
                            )
                        else:
                            self.gate_context["last_gate_config"] = (
                                "green_left_red_right"
                            )

                    if gate_center_x is not None:
                        pixel_error = gate_center_x - (frame_width / 2)
                        correction = map_value(
                            pixel_error, -frame_width / 2, frame_width / 2, -35.0, 35.0
                        )
                        servo_cmd = int(
                            max(servo_min, min(servo_max, servo_default - correction))
                        )
                        pwm_cmd = motor_base - 75

                        print(f"[Gate Traversal] Sending -> Servo: {servo_cmd}, PWM: {int(pwm_cmd)}")

                        command_to_send = f"A,{servo_cmd},{int(pwm_cmd)}\n"

                # === PRIORITAS 2: PENGHINDARAN TUNGGAL BERKONTEKS ===
                elif (
                    self.vision_target.get("active", False)
                    and self.gate_context["last_gate_config"]
                ):
                    self.recovering_from_avoidance = False
                    self.is_avoiding = True

                    frame_width = self.vision_target.get("frame_width", 640)
                    obj_center_x = self.vision_target.get("object_center_x")
                    obj_class = self.vision_target.get("obstacle_class")
                    last_config = self.gate_context["last_gate_config"]

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
                        servo_cmd = int(
                            max(
                                servo_min,
                                min(servo_max, servo_default - correction_deg),
                            )
                        )
                        pwm_cmd = int(max(1300, motor_base - abs(correction_deg) * 2))

                        print(f"[Obstacle Avoid] Sending -> Servo: {servo_cmd}, PWM: {pwm_cmd}")

                        command_to_send = f"A,{servo_cmd},{pwm_cmd}\n"
                    else:
                        self.gate_context["last_gate_config"] = None

                # === PRIORITAS 3: PENGHINDARAN TUNGGAL SEDERHANA (TANPA KONTEKS) ===
                elif self.vision_target.get("active", False):
                    self.recovering_from_avoidance = False
                    self.is_avoiding = True

                    frame_width = self.vision_target.get("frame_width", 640)
                    object_center_x = self.vision_target.get(
                        "object_center_x", frame_width / 2
                    )

                    if self.avoidance_direction is None:
                        if object_center_x < frame_width / 2:
                            self.avoidance_direction = "right"
                        else:
                            self.avoidance_direction = "left"

                    target_x = (
                        frame_width * 0.8
                        if self.avoidance_direction == "right"
                        else frame_width * 0.2
                    )
                    pixel_error = object_center_x - target_x
                    correction_deg = (pixel_error / (frame_width / 2)) * 45.0
                    servo_cmd = int(
                        max(servo_min, min(servo_max, servo_default - correction_deg))
                    )
                    pwm_cmd = int(max(1300, motor_base - abs(correction_deg) * 3))

                    command_to_send = f"A,{servo_cmd},{pwm_cmd}\n"

                # === FASE RECOVERY/PELURUSAN SETELAH MENGHINDAR ===
                elif self.recovering_from_avoidance:
                    self.is_avoiding = False
                    self.avoidance_direction = None
                    servo_cmd, pwm_cmd = servo_default, 1300

                    print(f"[Recovery] Sending -> Servo: {servo_cmd}, PWM: {pwm_cmd}")

                    command_to_send = f"A,{servo_cmd},{pwm_cmd}\n"
                    if time.time() - self.last_avoidance_time > 0.2:
                        self.recovering_from_avoidance = False

                # === KONDISI DEFAULT: NAVIGASI WAYPOINT ===
                else:
                    self.last_pixel_error = 0
                    command_to_send = "W\n"  # Minta ESP32 untuk navigasi waypoint

            # Hanya kirim jika ada perintah
            if command_to_send:
                self.serial_handler.send_command(command_to_send)

            self.logger.log_telemetry(state_for_logic)
            self._update_and_emit_state()
            
            # --- MODIFIKASI: Panggilan Firebase di-comment ---
            # send_telemetry_to_firebase(state_for_logic, self.config)
            # --- AKHIR MODIFIKASI ---
            
            self.socketio.sleep(0.1)

    def process_command(self, command, payload):
        # --- MODIFIKASI: Tambahkan penanganan arena ---
        # Kita asumsikan GUI mengirim arena bersamaan dengan waypoints
        # Contoh: payload = {"waypoints": [...], "arena": "A"}
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control,
            "SET_WAYPOINTS": self._handle_set_waypoints,  # Modifikasi fungsi ini
            "NAV_START": self._handle_start_mission,
            "NAV_RETURN": self._handle_initiate_rth,
            "UPDATE_PID": self._handle_update_pid,
            "VISION_TARGET_UPDATE": self._handle_vision_target_update,
            "GATE_TRAVERSAL_COMMAND": self._handle_gate_traversal_command,
            "DEBUG_WP_COUNTER": self._handle_debug_counter,
        }
        # --- AKHIR MODIFIKASI ---
        handler = command_handlers.get(command)
        if handler:
            handler(payload)
        else:
            print(f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'")

    def _handle_gate_traversal_command(self, payload):
        with self.state_lock:
            was_active = self.gate_target.get("active", False)
            is_active = payload.get("active", False)

            if was_active and not is_active:
                print("[AsvHandler] Gate traversal complete. Initiating recovery...")
                self.recovering_from_avoidance = True
                self.last_avoidance_time = time.time()

            self.gate_target["active"] = is_active
            if is_active:
                self.gate_target.update(payload)
                self.vision_target["active"] = False  # Pastikan mode lain nonaktif

    def _handle_debug_counter(self, payload):
        """Menangani increment/decrement dummy waypoint counter."""
        action = payload.get("action")
        with self.state_lock:
            # Otomatis aktifkan mode dummy saat tombol ini digunakan
            self.current_state["use_dummy_counter"] = True

            if action == "INC":
                self.current_state["debug_waypoint_counter"] += 1
            elif action == "DEC":
                self.current_state["debug_waypoint_counter"] = max(
                    0, self.current_state["debug_waypoint_counter"] - 1
                )
            elif action == "RESET":
                self.current_state["debug_waypoint_counter"] = 0

            # Kita batasi 9, karena data waypoint di monitor ada 9 titik
            max_points_in_monitor = 9
            self.current_state["debug_waypoint_counter"] = min(
                self.current_state["debug_waypoint_counter"], max_points_in_monitor
            )

            print(
                f"[AsvHandler] Debug counter diatur ke: {self.current_state['debug_waypoint_counter']}"
            )

    def _handle_vision_target_update(self, payload):
        with self.state_lock:
            if self.gate_target.get("active", False):
                return  # Abaikan jika sedang dalam mode gerbang

            was_active = self.vision_target.get("active", False)
            is_active = payload.get("active", False)

            if was_active and not is_active:
                print("[AsvHandler] Obstacle cleared. Initiating recovery...")
                self.recovering_from_avoidance = True
                self.last_avoidance_time = time.time()
                self.is_avoiding = False
                self.avoidance_direction = None

            self.vision_target["active"] = is_active
            if is_active:
                self.vision_target.update(payload)

    def _handle_manual_control(self, payload):
        with self.state_lock:
            # Jangan kirim perintah manual GUI jika RC mengambil alih
            if self.current_state.get("rc_channels", [1500] * 6)[4] < 1500:
                return
            # Hanya kirim jika mode GUI adalah MANUAL
            if self.current_state.get("control_mode") != "MANUAL":
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

        # Hitung perintah
        pwm = pwm_stop + fwd * pwr
        servo = servo_def - turn * (servo_def - servo_min)
        servo = max(servo_min, min(servo_max, servo))
        print(f"[Manual Control] Sending -> Servo: {int(servo)}, PWM: {int(pwm)}")

        # Kirim sebagai perintah AI "A," karena ESP32 mengharapkan ini
        # saat tidak dalam mode RC Manual
        self.serial_handler.send_command(f"A,{int(servo)},{int(pwm)}\n")

        # Simpan secara lokal untuk GUI
        with self.state_lock:
            self.current_state["manual_servo_cmd"] = int(servo)
            self.current_state["manual_motor_cmd"] = int(pwm)

    def set_streaming_status(self, status: bool):
        if self.is_streaming_to_gui != status:
            print(f"[AsvHandler] Status streaming telemetri diatur ke: {status}")
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
            print(f"[AsvHandler] PID updated: P={kp}, I={ki}, D={kd}")
            # TODO: Kirim PID baru ke ESP32 jika ESP32 yang menjalankan PID
            # self.serial_handler.send_command(f"P{kp};I{ki};D{kd}\n")

    def _handle_serial_configuration(self, payload):
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO":
            self.serial_handler.find_and_connect_esp32(baud)
        else:
            self.serial_handler.connect(port, baud)

    def _handle_mode_change(self, payload):
        with self.state_lock:
            self.current_state["control_mode"] = payload.get("mode", "MANUAL")
        self.logger.log_event(
            f"Mode kontrol GUI diubah ke: {self.current_state['control_mode']}"
        )

        # Jika beralih ke mode MANUAL, kirim perintah netral
        if self.current_state["control_mode"] == "MANUAL":
            actuator_config = self.config.get("actuators", {})
            pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
            servo_def = actuator_config.get("servo_default_angle", 90)
            self.serial_handler.send_command(f"A,{int(servo_def)},{int(pwm_stop)}\n")

    def _handle_set_waypoints(self, payload):
        """Menangani penerimaan waypoints DAN arena dari GUI."""
        waypoints_data = payload.get("waypoints")
        arena_id = payload.get("arena")  # Ambil ID arena

        if not isinstance(waypoints_data, list):
            print("[AsvHandler] Gagal set waypoints: Data tidak valid.")
            return

        with self.state_lock:
            self.current_state["waypoints"] = waypoints_data
            self.current_state["current_waypoint_index"] = 0  # Selalu mulai dari 0
            self.current_state["active_arena"] = arena_id  # Simpan arena
            self.logger.log_event(
                f"Waypoints baru dimuat (Arena: {arena_id}). Jumlah: {len(waypoints_data)}"
            )
        # TODO: Kirim waypoints ini ke ESP32 jika ESP32 yang navigasi
        # self.serial_handler.send_command("WP_START\n")
        # for wp in payload:
        #    self.serial_handler.send_command(f"WP:{wp['lat']},{wp['lon']}\n")
        # self.serial_handler.send_command("WP_END\n")

    def _handle_start_mission(self, payload):
        with self.state_lock:
            if not self.current_state["waypoints"]:
                return
            self.current_state["control_mode"] = "AUTO"
            self.current_state["current_waypoint_index"] = 0  # Mulai ulang dari 0
            self.current_state["use_dummy_counter"] = False
        self.logger.log_event("Misi navigasi dimulai.")
        # Perintah "W" akan dikirim oleh loop utama
        pass

    def _handle_initiate_rth(self, payload):
        with self.state_lock:
            if not self.current_state["waypoints"]:
                return
            self.current_state["waypoints"] = [self.current_state["waypoints"][0]]
            self.current_state["current_waypoint_index"] = 0
            self.current_state["control_mode"] = "AUTO"
        self.logger.log_event("Memulai Return to Home.")
        # Perintah "W" akan dikirim oleh loop utama
        pass

    def stop(self):
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        print("[AsvHandler] Dihentikan.")
        pass