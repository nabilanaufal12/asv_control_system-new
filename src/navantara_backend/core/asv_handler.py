# src/navantara_backend/core/asv_handler.py
# --- VERSI FINAL LENGKAP: Kontrol Hierarkis + Debugging + EKF + AUTO-CONNECT ---
import threading
import time
import math
import numpy as np

from navantara_backend.services.serial_service import SerialHandler
from navantara_backend.core.navigation import run_navigation_logic, PIDController
from navantara_backend.core.kalman_filter import SimpleEKF
from navantara_backend.core.mission_logger import MissionLogger

def map_value(x, in_min, in_max, out_min, out_max):
    """Fungsi map seperti di Arduino untuk konversi nilai."""
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
            "latitude": -6.9180, "longitude": 107.6185,
            "heading": 90.0, "cog": 0.0, "speed": 0.0, "battery_voltage": 12.5,
            "status": "DISCONNECTED", "mission_time": "00:00:00",
            "waypoints": [], "current_waypoint_index": 0,
            "is_connected_to_serial": False,
            "gyro_z": 0.0, "accel_x": 0.0,
            "rc_channels": [1500] * 6,
        }
        self.vision_target = {"active": False, "v_opt": 0.0, "omega_opt": 0.0}

        pid_config = self.config.get("navigation", {}).get("heading_pid", {})
        self.pid_controller = PIDController(
            Kp=pid_config.get("kp", 1.2), Ki=pid_config.get("ki", 0.1), Kd=pid_config.get("kd", 0.05)
        )
        self.ekf = SimpleEKF(np.zeros(5), np.eye(5) * 0.1)
        self.last_ekf_update_time = time.time()
        
        self.logger = MissionLogger()
        self.logger.log_event("AsvHandler diinisialisasi.")
        print("[AsvHandler] Handler diinisialisasi untuk operasi backend.")

        # --- PERUBAHAN UTAMA DI SINI ---
        # Secara otomatis mencoba terhubung ke ESP32 saat backend dimulai
        self.initiate_auto_connection()
        # --- AKHIR PERUBAHAN ---

    # --- FUNGSI BARU UNTUK KONEKSI OTOMATIS ---
    def initiate_auto_connection(self):
        """Mencari dan terhubung ke ESP32 menggunakan konfigurasi."""
        print("[AsvHandler] Memulai upaya koneksi serial otomatis...")
        baud_rate = self.config.get("serial_connection", {}).get("default_baud_rate", 115200)
        # Fungsi ini sudah ada di SerialHandler, kita tinggal memanggilnya
        self.serial_handler.find_and_connect_esp32(baud_rate)
    # --- AKHIR FUNGSI BARU ---

    def _update_and_emit_state(self):
        if self.running and self.is_streaming_to_gui:
            with self.state_lock:
                state_copy = self.current_state.copy()
                # --- PERBAIKAN KECIL: Tambahkan status koneksi serial ke state ---
                state_copy["is_connected_to_serial"] = self.serial_handler.is_connected
                if not self.serial_handler.is_connected:
                    state_copy["status"] = "DISCONNECTED (SERIAL)"

                rc_mode_switch = state_copy.get("rc_channels", [1500]*6)[4]
                if rc_mode_switch < 1500:
                    state_copy["status"] = "RC MANUAL OVERRIDE"
                    state_copy["control_mode"] = "MANUAL"
                elif self.vision_target["active"]:
                    state_copy["status"] = "AI OBSTACLE AVOIDANCE"
                    state_copy["control_mode"] = "AUTO"
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

            # --- AWAL DARI LOGIKA KONTROL YANG DIPERBAIKI ---
            rc_mode_switch = state_for_logic.get("rc_channels", [1500]*6)[4]
            command_to_send = None # Variabel untuk menampung perintah final yang akan dikirim

            # PRIORITAS 1: Remote Control Override (Kendali Manual dari RC)
            if rc_mode_switch < 1500:
                rc_servo_raw = state_for_logic["rc_channels"][0]
                rc_motor_raw = state_for_logic["rc_channels"][2]
                actuator_config = self.config.get("actuators", {})
                servo_min = actuator_config.get("servo_min_angle", 45)
                servo_max = actuator_config.get("servo_max_angle", 135)
                servo_cmd = int(map_value(rc_servo_raw, 1000, 2000, servo_min, servo_max))
                pwm_cmd = rc_motor_raw
                print(f"🕹️  [CONTROL] RC Manual Override -> PWM: {pwm_cmd}, Servo: {servo_cmd}°")
                # Dalam mode RC, kita kirim perintah S;D untuk kontrol langsung
                command_to_send = f"S{int(pwm_cmd)};D{int(servo_cmd)}\n"

            # PRIORITAS 2: Logika Otonom (Hanya jika tidak di-override oleh RC)
            else:
                # Sub-Prioritas 2.1: Penghindaran Rintangan oleh AI Vision
                if self.vision_target["active"]:
                    # Ambil data posisi rintangan dari vision_service
                    obstacle_x = self.vision_target.get("obstacle_center_x", 320)
                    frame_width = self.vision_target.get("frame_width", 640)
                    
                    # Hitung seberapa jauh rintangan dari tengah layar (-1.0 kiri, 1.0 kanan)
                    error_x = (obstacle_x - (frame_width / 2)) / (frame_width / 2)
                    
                    # --- LOGIKA KEMUDI REAKTIF ---
                    # Semakin jauh rintangan dari tengah, semakin tajam belokannya
                    # Nilai '40' ini adalah "kekuatan" belokan, bisa di-tuning
                    steering_correction = error_x * 40.0
                    
                    actuator_config = self.config.get("actuators", {})
                    servo_default = actuator_config.get("servo_default_angle", 90)
                    servo_min = actuator_config.get("servo_min_angle", 45)
                    servo_max = actuator_config.get("servo_max_angle", 135)

                    # Kita balik koreksinya (-steering_correction) agar belok menjauh
                    servo_cmd = servo_default - steering_correction
                    servo_cmd = int(max(servo_min, min(servo_max, servo_cmd)))
                    
                    # Kurangi kecepatan saat menghindar
                    pwm_cmd = actuator_config.get("motor_pwm_auto_base", 1650) - 50 
                    
                    print(f"🤖 [CONTROL] AI Vision Reactive -> ErrorX: {error_x:.2f}, Servo: {servo_cmd}°")
                    command_to_send = f"A,{int(servo_cmd)},{int(pwm_cmd)}\n"

                # Sub-Prioritas 2.2: Navigasi Waypoint (jika AI tidak aktif)
                elif state_for_logic.get("control_mode") == "AUTO":
                    print(f"🛰️  [CONTROL] Auto (Waypoint) -> Mengirim 'W' ke ESP32")
                    # Kirim perintah 'W' agar ESP32 melanjutkan logika waypoint internalnya
                    command_to_send = "W\n"
                
                # Jika tidak ada kondisi di atas, maka kapal akan idle (diam)
                else:
                    pass
            
            # Kirim perintah ke ESP32 HANYA jika ada perintah yang valid
            if command_to_send:
                self.serial_handler.send_command(command_to_send)
            # --- AKHIR DARI LOGIKA KONTROL YANG DIPERBAIKI ---

            self.logger.log_telemetry(state_for_logic)
            self._update_and_emit_state()
            self.socketio.sleep(0.1)

    def process_command(self, command, payload):
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration, "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control, "SET_WAYPOINTS": self._handle_set_waypoints,
            "NAV_START": self._handle_start_mission, "NAV_RETURN": self._handle_initiate_rth,
            "UPDATE_PID": self._handle_update_pid, "PLANNED_MANEUVER": self._handle_vision_maneuver,
            "VISION_TARGET_UPDATE": self._handle_vision_maneuver,
        }
        handler = command_handlers.get(command)
        if handler: handler(payload)
        else: print(f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'")

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
        print(f"⌨️  [CONTROL] GUI Manual (WASD) -> PWM: {int(pwm)}, Servo: {int(servo)}°")
        self.serial_handler.send_command(f"S{int(pwm)};D{int(servo)}\n")

    def set_streaming_status(self, status: bool):
        if self.is_streaming_to_gui != status:
            print(f"[AsvHandler] Status streaming telemetri diatur ke: {status}")
            self.is_streaming_to_gui = status

    def _handle_vision_maneuver(self, payload):
        with self.state_lock:
            self.vision_target["active"] = payload.get("active", False)
            if self.vision_target["active"]:
                self.vision_target["v_opt"] = payload.get("v_opt", 0.0)
                self.vision_target["omega_opt"] = payload.get("omega_opt", 0.0)

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