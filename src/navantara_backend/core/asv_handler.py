# src/navantara_backend/core/asv_handler.py
# --- VERSI FINAL DENGAN LOGIKA GATE TRAVERSAL DAN HIERARKI KONTROL ---
import threading
import time
import math
import numpy as np

from navantara_backend.services.serial_service import SerialHandler
from navantara_backend.core.navigation import run_navigation_logic, PIDController
from navantara_backend.core.kalman_filter import SimpleEKF
from navantara_backend.core.mission_logger import MissionLogger

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
            "control_mode": "AUTO", "latitude": -6.9180, "longitude": 107.6185,
            "heading": 90.0, "cog": 0.0, "speed": 0.0, "battery_voltage": 12.5,
            "status": "DISCONNECTED", "mission_time": "00:00:00", "waypoints": [],
            "current_waypoint_index": 0, "is_connected_to_serial": False,
            "gyro_z": 0.0, "accel_x": 0.0, "rc_channels": [1500] * 6,
        }
        
        # Inisialisasi state untuk setiap mode AI
        self.vision_target = {"active": False}
        self.gate_target = {"active": False}
        self.last_pixel_error = 0 

        self.avoidance_direction = None # 'left' atau 'right'
        self.is_avoiding = False

        # --- FIX IS HERE: Re-initializing gate_context ---
        # This dictionary holds the memory of the gate approach maneuver.
        self.gate_context = {
            "is_gate_approaching": False,
            "red_buoy_is_left": True,
        }
        # --- END OF FIX ---

        # Variabel untuk manuver transisi (recovery)
        self.recovering_from_avoidance = False
        self.last_avoidance_time = 0

        pid_config = self.config.get("navigation", {}).get("heading_pid", {})
        self.pid_controller = PIDController(
            Kp=pid_config.get("kp", 1.2), Ki=pid_config.get("ki", 0.1), Kd=pid_config.get("kd", 0.05)
        )
        self.ekf = SimpleEKF(np.zeros(5), np.eye(5) * 0.1)
        self.last_ekf_update_time = time.time()
        
        self.logger = MissionLogger()
        self.logger.log_event("AsvHandler diinisialisasi.")
        print("[AsvHandler] Handler diinisialisasi untuk operasi backend.")
        
        self.initiate_auto_connection()

    def initiate_auto_connection(self):
        print("[AsvHandler] Memulai upaya koneksi serial otomatis...")
        baud_rate = self.config.get("serial_connection", {}).get("default_baud_rate", 115200)
        self.serial_handler.find_and_connect_esp32(baud_rate)

    def _update_and_emit_state(self):
        if self.running and self.is_streaming_to_gui:
            with self.state_lock:
                state_copy = self.current_state.copy()
                state_copy["is_connected_to_serial"] = self.serial_handler.is_connected
                if not self.serial_handler.is_connected:
                    state_copy["status"] = "DISCONNECTED (SERIAL)"

                rc_mode_switch = state_copy.get("rc_channels", [1500]*6)[4]
                # Hierarki status untuk ditampilkan di GUI
                if rc_mode_switch < 1500:
                    state_copy["status"] = "RC MANUAL OVERRIDE"
                    state_copy["control_mode"] = "MANUAL"
                elif self.gate_target["active"]: # Prioritas status tertinggi
                    state_copy["status"] = "AI GATE TRAVERSAL"
                elif self.vision_target["active"]:
                    state_copy["status"] = "AI OBSTACLE AVOIDANCE"
                elif self.recovering_from_avoidance:
                    state_copy["status"] = "STRAIGHTENING COURSE"
                elif state_copy["control_mode"] == "AUTO":
                    wp_idx = state_copy.get("current_waypoint_index", 0)
                    total_wps = len(state_copy.get("waypoints", []))
                    if total_wps > 0 and wp_idx < total_wps:
                        state_copy["status"] = f"WAYPOINT NAVIGATION ({wp_idx + 1}/{total_wps})"
                    else:
                        state_copy["status"] = "AUTO IDLE (Waiting for mission)"
                else:
                    state_copy["status"] = "MANUAL (GUI CONTROL)"
            self.socketio.emit("telemetry_update", state_copy)

    # ... (_read_from_serial_loop dan _parse_telemetry tidak berubah)
    def _read_from_serial_loop(self):
        while self.running:
            line = self.serial_handler.read_line()
            if line and line.startswith("T:"):
                self._parse_telemetry(line)
            else:
                self.socketio.sleep(0.05)
    
    def _parse_telemetry(self, line):
        try:
            with self.state_lock:
                parts = line.strip("T:").split(";")
                for part in parts:
                    if not part: continue
                    data = part.split(",")
                    data_type = data[0]
                    if data_type == "GPS" and len(data) == 3:
                        self.current_state["latitude"], self.current_state["longitude"] = float(data[1]), float(data[2])
                    elif data_type == "COMP" and len(data) == 2:
                        self.current_state["heading"] = float(data[1])
                    elif data_type == "SPD" and len(data) == 2:
                        self.current_state["speed"] = float(data[1])
                    elif data_type == "RC" and len(data) == 7:
                        self.current_state["rc_channels"] = [int(val) for val in data[1:]]
        except (ValueError, IndexError, TypeError):
            pass

    def main_logic_loop(self):
        self.socketio.start_background_task(self._read_from_serial_loop)
        print("[AsvHandler] Loop pembaca serial dimulai.")
        while self.running:
            # --- Bagian EKF Update (tidak berubah) ---
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
                    self.current_state["heading"] = (np.degrees(self.ekf.state[2]) + 360) % 360
            
            with self.state_lock:
                state_for_logic = self.current_state.copy()

            rc_mode_switch = state_for_logic.get("rc_channels", [1500]*6)[4]
            command_to_send = None

            if rc_mode_switch < 1500:
                # --- Mode RC Override (tidak berubah) ---
                rc_servo_raw = state_for_logic["rc_channels"][0]
                rc_motor_raw = state_for_logic["rc_channels"][2]
                actuator_config = self.config.get("actuators", {})
                servo_min = actuator_config.get("servo_min_angle", 45)
                servo_max = actuator_config.get("servo_max_angle", 135)
                servo_cmd = int(map_value(rc_servo_raw, 1000, 2000, servo_min, servo_max))
                pwm_cmd = rc_motor_raw
                command_to_send = f"S{int(pwm_cmd)};D{int(servo_cmd)}\n"
            else:
                # --- HIERARKI KEPUTUSAN MODE AUTO ---
                # --- HIRARKI KEPUTUSAN MODE AUTO ---
                
                actuator_config = self.config.get("actuators", {})
                servo_default = actuator_config.get("servo_default_angle", 90)
                servo_min = actuator_config.get("servo_min_angle", 45)
                servo_max = actuator_config.get("servo_max_angle", 135)
                motor_base = actuator_config.get("motor_pwm_auto_base", 1650)

                # === PRIORITAS 1: MELEWATI GERBANG (GATE TRAVERSAL) ===
                # Ini adalah prioritas tertinggi. Jika aktif, logika lain diabaikan.
                if self.gate_target.get("active", False):
                    self.recovering_from_avoidance = False
                    self.is_avoiding = False
                    self.avoidance_direction = None

                    frame_width = self.gate_target.get("frame_width", 640)
                    gate_center_x = self.gate_target.get("gate_center_x")

                    if gate_center_x is not None:
                        # Logika untuk mengarahkan kapal ke tengah gerbang
                        pixel_error = gate_center_x - (frame_width / 2)
                        max_correction_angle = 35.0 # Koreksi lebih halus untuk gerbang
                        correction = map_value(pixel_error, -frame_width/2, frame_width/2, -max_correction_angle, max_correction_angle)
                        
                        servo_cmd = int(max(servo_min, min(servo_max, servo_default - correction)))
                        # Kurangi kecepatan saat melewati gerbang untuk presisi
                        pwm_cmd = motor_base - 75 
                        
                        print(f"GATE MODE | TargetX:{frame_width/2:.0f}, GateX:{gate_center_x:.0f}, Err:{pixel_error:.0f}px -> SERVO:{servo_cmd}, PWM:{pwm_cmd}")
                        command_to_send = f"A,{servo_cmd},{int(pwm_cmd)}\n"

                # === PRIORITAS 2: PENGHINDARAN RINTANGAN TUNGGAL ===
                # Blok ini HANYA akan berjalan jika Prioritas 1 (Gate) TIDAK aktif.
                elif self.vision_target.get("active", False):
                    self.recovering_from_avoidance = False
                    self.is_avoiding = True

                    # (Logika penghindaran rintangan tunggal dari perbaikan sebelumnya tetap di sini)
                    # ... (salin-tempel seluruh blok logika Prioritas 2 dari jawaban saya sebelumnya, tidak ada yang berubah di dalamnya)
                    frame_width = self.vision_target.get("frame_width", 640)
                    object_center_x = self.vision_target.get("object_center_x", frame_width / 2)
                    if self.avoidance_direction is not None:
                        if self.avoidance_direction == 'left' and object_center_x < (frame_width / 2):
                            print("SCENARIO CHANGE: Obstacle crossed to the LEFT. Re-evaluating.")
                            self.avoidance_direction = None
                        elif self.avoidance_direction == 'right' and object_center_x > (frame_width / 2):
                            print("SCENARIO CHANGE: Obstacle crossed to the RIGHT. Re-evaluating.")
                            self.avoidance_direction = None
                    if self.avoidance_direction is None:
                        if object_center_x < frame_width / 2:
                            self.avoidance_direction = 'right'
                            print("DECISION: Obstacle on LEFT. Evading to the RIGHT.")
                        else:
                            self.avoidance_direction = 'left'
                            print("DECISION: Obstacle on RIGHT. Evading to the LEFT.")
                    target_x = 0
                    if self.avoidance_direction == 'left':
                        target_x = frame_width * 0.8
                    else:
                        target_x = frame_width * 0.2
                    pixel_error = object_center_x - target_x
                    max_correction_angle = 45.0
                    correction_deg = (pixel_error / (frame_width / 2)) * max_correction_angle
                    servo_cmd = int(max(servo_min, min(servo_max, servo_default - correction_deg)))
                    reduction = abs(correction_deg) * 3
                    pwm_cmd = int(max(1550, motor_base - reduction))
                    print(f"AVOID ({self.avoidance_direction}) | TargetX:{target_x:.0f}, ObjX:{object_center_x:.0f}, Err:{pixel_error:.0f}px -> SERVO:{servo_cmd}, PWM:{pwm_cmd}")
                    command_to_send = f"A,{servo_cmd},{pwm_cmd}\n"


                # === PRIORITAS 3: FASE RECOVERY/PELURUSAN ===
                elif self.recovering_from_avoidance:
                    self.is_avoiding = False
                    self.avoidance_direction = None
                    print("DECISION: Straightening course...")
                    command_to_send = "A,90,1600\n" 
                    if time.time() - self.last_avoidance_time > 1.5: 
                        self.recovering_from_avoidance = False
                        print("DECISION: Straightening complete. Resuming mission.")

                # === KONDISI DEFAULT: NAVIGASI WAYPOINT ===
                elif state_for_logic.get("control_mode") == "AUTO":
                    self.last_pixel_error = 0
                    command_to_send = "W\n"
            
            # --- Pengiriman Perintah Serial ---
            if command_to_send:
                self.serial_handler.send_command(command_to_send)

            # --- Logging dan Update GUI ---
            self.logger.log_telemetry(state_for_logic)
            self._update_and_emit_state()
            self.socketio.sleep(0.1)

    def process_command(self, command, payload):
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration, "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control, "SET_WAYPOINTS": self._handle_set_waypoints,
            "NAV_START": self._handle_start_mission, "NAV_RETURN": self._handle_initiate_rth,
            "UPDATE_PID": self._handle_update_pid,
            "VISION_TARGET_UPDATE": self._handle_vision_target_update, # Nama fungsi diperjelas
            "GATE_TRAVERSAL_COMMAND": self._handle_gate_traversal_command # Handler baru
        }
        handler = command_handlers.get(command)
        if handler: handler(payload)
        else: print(f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'")

    def _handle_gate_traversal_command(self, payload):
        """Handler untuk perintah melewati gerbang dari vision_service."""
        with self.state_lock:
            was_active = self.gate_target.get("active", False)
            is_active = payload.get("active", False)
            
            # Deteksi transisi dari aktif ke tidak aktif untuk memulai fase recovery.
            if was_active and not is_active:
                print("Gate traversal complete. Initiating recovery...")
                self.recovering_from_avoidance = True
                self.last_avoidance_time = time.time()

            self.gate_target["active"] = is_active
            if is_active:
                self.gate_target.update(payload)
                # Pastikan mode penghindaran rintangan tunggal dinonaktifkan
                # untuk mencegah konflik.
                self.vision_target["active"] = False

    def _handle_vision_target_update(self, payload): # Nama fungsi diperjelas
        """Handler untuk perintah penghindaran rintangan tunggal."""
        with self.state_lock:
            # Jangan aktifkan mode ini jika mode gerbang sedang aktif.
            if self.gate_target.get("active", False):
                return
            
            was_active = self.vision_target.get("active", False)
            is_active = payload.get("active", False)
            
            # --- PERUBAHAN UTAMA DI SINI ---
            # Jika rintangan BARU SAJA HILANG (transisi dari aktif ke non-aktif)
            if was_active and not is_active:
                print("Obstacle cleared. Initiating recovery...")
                self.recovering_from_avoidance = True
                self.last_avoidance_time = time.time()
                # Reset state penghindaran agar manuver berikutnya bisa menentukan arah baru
                self.is_avoiding = False
                self.avoidance_direction = None
            # --- AKHIR PERUBAHAN ---

            self.vision_target["active"] = is_active
            if is_active:
                self.vision_target.update(payload)

    # ... (sisa fungsi handler tidak berubah)
    def _handle_manual_control(self, payload):
        with self.state_lock:
            if self.current_state.get("rc_channels", [1500]*6)[4] < 1500: return
            if self.current_state.get("control_mode") != "MANUAL": return
        keys, actuator_config = set(payload), self.config.get("actuators", {})
        pwm_stop, pwr = actuator_config.get("motor_pwm_stop", 1500), actuator_config.get("motor_pwm_manual_power", 150)
        servo_def, servo_min, servo_max = actuator_config.get("servo_default_angle", 90), actuator_config.get("servo_min_angle", 45), actuator_config.get("servo_max_angle", 135)
        fwd = 1 if "W" in keys else -1 if "S" in keys else 0
        turn = 1 if "D" in keys else -1 if "A" in keys else 0
        pwm = pwm_stop + fwd * pwr
        servo = servo_def - turn * (servo_def - servo_min)
        servo = max(servo_min, min(servo_max, servo))
        self.serial_handler.send_command(f"S{int(pwm)};D{int(servo)}\n")

    def set_streaming_status(self, status: bool):
        if self.is_streaming_to_gui != status:
            print(f"[AsvHandler] Status streaming telemetri diatur ke: {status}")
            self.is_streaming_to_gui = status

    def _handle_update_pid(self, payload):
        kp, ki, kd = payload.get("p"), payload.get("i"), payload.get("d")
        if all(isinstance(val, (int, float)) for val in [kp, ki, kd]):
            self.pid_controller.Kp, self.pid_controller.Ki, self.pid_controller.Kd = kp, ki, kd
            self.pid_controller.reset()
            print(f"[AsvHandler] PID updated: P={kp}, I={ki}, D={kd}")

    def _handle_serial_configuration(self, payload):
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO": self.serial_handler.find_and_connect_esp32(baud)
        else: self.serial_handler.connect(port, baud)

    def _handle_mode_change(self, payload):
        with self.state_lock: self.current_state["control_mode"] = payload.get("mode", "MANUAL")
        self.logger.log_event(f"Mode kontrol GUI diubah ke: {self.current_state['control_mode']}")

    def _handle_set_waypoints(self, payload):
        with self.state_lock:
            self.current_state["waypoints"], self.current_state["current_waypoint_index"] = payload, 0
        self.logger.log_event(f"Waypoints baru dimuat. Jumlah: {len(payload)}")

    def _handle_start_mission(self, payload):
        with self.state_lock:
            if not self.current_state["waypoints"]: return
            self.current_state["control_mode"] = "AUTO"
        self.logger.log_event("Misi navigasi dimulai.")

    def _handle_initiate_rth(self, payload):
        with self.state_lock:
            if not self.current_state["waypoints"]: return
            self.current_state["waypoints"] = [self.current_state["waypoints"][0]]
            self.current_state["current_waypoint_index"] = 0
            self.current_state["control_mode"] = "AUTO"
        self.logger.log_event("Memulai Return to Home.")

    def stop(self):
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        print("[AsvHandler] Dihentikan.")