# src/navantara_backend/core/asv_handler.py
# --- VERSI FINAL DENGAN LOGIKA REMOTE CONTROL OVERRIDE ---
import threading
import time
import math
import numpy as np

from navantara_backend.services.serial_service import SerialHandler
from navantara_backend.core.navigation import run_navigation_logic, PIDController
from navantara_backend.core.kalman_filter import SimpleEKF
from navantara_backend.core.mission_logger import MissionLogger

# --- FUNGSI HELPER BARU ---
def map_value(x, in_min, in_max, out_min, out_max):
    """Fungsi map seperti di Arduino untuk konversi nilai."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class AsvHandler:
    """
    Mengelola state, logika, dan komunikasi hardware dari ASV.
    Berjalan sebagai greenlet independen dan berkomunikasi melalui WebSockets.
    """

    def __init__(self, config, socketio):
        self.config = config
        self.socketio = socketio
        self.serial_handler = SerialHandler(config)
        self.running = True
        self.state_lock = threading.Lock()

        self.is_streaming_to_gui = False

        self.current_state = {
            "control_mode": "MANUAL",
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
            "last_patrol_waypoint_index": 0,
            "original_patrol_waypoints": [],
            "points_of_interest": [],
            "gyro_z": 0.0,
            "accel_x": 0.0,
            "rc_channels": [1500] * 6,  # Tambahkan state untuk menyimpan nilai RC
        }
        self.vision_target = {"active": False, "degree": 90}
        self.is_returning_home = False
        self.rth_paused_until = 0
        self.mission_phase = "IDLE"

        pid_config = self.config.get("navigation", {}).get("heading_pid", {})
        self.pid_controller = PIDController(
            Kp=pid_config.get("kp", 1.2),
            Ki=pid_config.get("ki", 0.1),
            Kd=pid_config.get("kd", 0.05),
        )
        self.ekf = SimpleEKF(np.zeros(5), np.eye(5) * 0.1)
        self.last_ekf_update_time = time.time()
        self.logger = MissionLogger()
        self.logger.log_event("AsvHandler diinisialisasi.")
        print("[AsvHandler] Handler diinisialisasi untuk operasi backend.")

    def _update_and_emit_state(self):
        if self.running and self.is_streaming_to_gui:
            with self.state_lock:
                state_copy = self.current_state.copy()
                mission_phase = self.mission_phase
                
                # Cek mode dari remote
                rc_mode_switch = state_copy.get("rc_channels", [1500]*6)[4]
                if rc_mode_switch < 1500:
                    state_copy["status"] = "RC MANUAL OVERRIDE"
                elif mission_phase == "NAVIGATING":
                    wp_idx = state_copy.get("current_waypoint_index", 0)
                    total_wps = len(state_copy.get("waypoints", []))
                    state_copy["status"] = f"NAVIGATING TO WP {wp_idx + 1}/{total_wps}"
                elif mission_phase == "RETURNING_HOME":
                    state_copy["status"] = "RETURNING TO HOME"
                elif mission_phase == "IDLE":
                    state_copy["status"] = "IDLE (Ready for mission)"
                else:
                    state_copy["status"] = ("CONNECTED" if state_copy.get("is_connected_to_serial") else "DISCONNECTED")
            self.socketio.emit("telemetry_update", state_copy)

    def _read_from_serial_loop(self):
        if self.config.get("general", {}).get("use_simulation", False):
            print("SENSOR SIMULATOR: Pembacaan data serial dilewati.")
            return
        while self.running:
            line = self.serial_handler.read_line()
            if line and line.startswith("T:"):
                self._parse_telemetry(line)
            else:
                self.socketio.sleep(0.1)

    def _parse_telemetry(self, line):
        try:
            with self.state_lock:
                parts = line.strip("T:").split(";")
                for part in parts:
                    data = part.split(",")
                    if data[0] == "GPS":
                        self.current_state["latitude"], self.current_state["longitude"] = float(data[1]), float(data[2])
                    elif data[0] == "COMP":
                        self.current_state["heading"] = float(data[1])
                    elif data[0] == "SPD":
                        self.current_state["speed"] = float(data[1])
                    elif data[0] == "RC": # Parse data Remote Control
                        self.current_state["rc_channels"] = [int(val) for val in data[1:]]
        except (ValueError, IndexError, TypeError):
            pass # Abaikan jika data serial korup

    def _run_simulation_step(self, servo_angle, motor_pwm):
        """Memperbarui state heading dan posisi berdasarkan input aktuator."""
        with self.state_lock:
            turn_rate_factor = 0.5
            heading_change = (servo_angle - 90) * turn_rate_factor * -0.1
            new_heading = (self.current_state["heading"] + heading_change + 360) % 360
            self.current_state["heading"] = new_heading
            if motor_pwm > 1500:
                speed_m_per_s = 1.5
                dist = speed_m_per_s * 0.2
                heading_rad = math.radians(new_heading)
                d_lat = dist * math.cos(heading_rad) / 111320.0
                d_lon = dist * math.sin(heading_rad) / (40075000 * math.cos(math.radians(self.current_state["latitude"])) / 360)
                self.current_state["latitude"] += d_lat
                self.current_state["longitude"] += d_lon

    def main_logic_loop(self):
        self.socketio.start_background_task(self._read_from_serial_loop)
        print("[AsvHandler] Loop pembaca serial dimulai sebagai greenlet.")
        start_time = time.time()
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
                    self.current_state["speed"] = self.ekf.state[3]

            elapsed = time.time() - start_time
            mission_time_str = time.strftime("%H:%M:%S", time.gmtime(elapsed))
            new_connection_status = self.serial_handler.is_connected
            with self.state_lock:
                self.current_state["mission_time"] = mission_time_str
                if self.current_state["is_connected_to_serial"] != new_connection_status:
                    self.current_state["is_connected_to_serial"] = new_connection_status
                    self.logger.log_event(f"Status Koneksi Serial: {'CONNECTED' if new_connection_status else 'DISCONNECTED'}")
            
            with self.state_lock:
                state_for_logic = self.current_state.copy()

            servo_cmd, pwm_cmd = 90, 1500
            
            rc_mode_switch = state_for_logic.get("rc_channels", [1500]*6)[4]

            if rc_mode_switch < 1500:
                rc_servo_raw = state_for_logic["rc_channels"][0]
                rc_motor_raw = state_for_logic["rc_channels"][2]
                servo_cmd = int(map_value(rc_servo_raw, 1000, 2000, 0, 180))
                pwm_cmd = rc_motor_raw
                print(f"🕹️  [CONTROL] RC Manual Override -> PWM: {pwm_cmd}, Servo: {servo_cmd}°")
                self.serial_handler.send_command(f"S{pwm_cmd};D{servo_cmd}\n")
            else:
                if state_for_logic.get("control_mode") == "AUTO":
                    servo_cmd, pwm_cmd = run_navigation_logic(state_for_logic, self.config, self.pid_controller)
                    print(f"🚢 [CONTROL] Auto Mode -> PWM: {pwm_cmd}, Servo: {servo_cmd}°")
                    self.serial_handler.send_command(f"S{pwm_cmd};D{servo_cmd}\n")
                else:
                    pass

            if self.config.get("general", {}).get("use_simulation", False):
                self._run_simulation_step(servo_cmd, pwm_cmd)

            self.logger.log_telemetry(state_for_logic)
            self._update_and_emit_state()
            self.socketio.sleep(0.2)

    def set_streaming_status(self, status: bool):
        if self.is_streaming_to_gui != status:
            print(f"[AsvHandler] Status streaming telemetri diatur ke: {status}")
            self.is_streaming_to_gui = status

    def process_command(self, command, payload):
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control,
            "SET_WAYPOINTS": self._handle_set_waypoints,
            "NAV_START": self._handle_start_mission,
            "NAV_RETURN": self._handle_initiate_rth,
            "UPDATE_PID": self._handle_update_pid,
        }
        handler = command_handlers.get(command)
        if handler:
            handler(payload)
        else:
            print(f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'")

    def _handle_update_pid(self, payload):
        kp = payload.get("p")
        ki = payload.get("i")
        kd = payload.get("d")
        if all(isinstance(val, (int, float)) for val in [kp, ki, kd]):
            self.pid_controller.Kp = kp
            self.pid_controller.Ki = ki
            self.pid_controller.Kd = kd
            self.pid_controller.reset()
            print(f"[AsvHandler] PID updated: P={kp}, I={ki}, D={kd}")
        else:
            print("[AsvHandler] Peringatan: Menerima data PID yang tidak valid.")

    def _handle_serial_configuration(self, payload):
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO":
            self.serial_handler.find_and_connect_esp32(baud)
        else:
            self.serial_handler.connect(port, baud)

    def _handle_mode_change(self, payload):
        with self.state_lock:
            self.current_state["control_mode"] = payload
        self.logger.log_event(f"Mode kontrol diubah ke: {payload}")

    def _handle_manual_control(self, payload):
        with self.state_lock:
            if self.current_state.get("rc_channels", [1500]*6)[4] < 1500:
                print("[AsvHandler] Input WASD diabaikan, RC Manual aktif.")
                return
            if self.current_state.get("control_mode") != "MANUAL":
                return
        
        keys = set(payload)
        actuator_config = self.config.get("actuators", {})
        pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
        pwr = actuator_config.get("motor_pwm_manual_power", 150)
        servo_def = actuator_config.get("servo_default_angle", 90)
        servo_min = actuator_config.get("servo_min_angle", 45)
        fwd = 1 if "W" in keys else -1 if "S" in keys else 0
        turn = 1 if "D" in keys else -1 if "A" in keys else 0
        pwm = pwm_stop + fwd * pwr
        servo = max(0, min(180, int(servo_def - turn * (servo_def - servo_min))))
        print(f"⌨️  [CONTROL] GUI Manual -> PWM: {pwm}, Servo: {servo}°")
        self.serial_handler.send_command(f"S{pwm};D{servo}\n")

    def _handle_set_waypoints(self, payload):
        with self.state_lock:
            self.current_state["waypoints"] = payload
            self.current_state["current_waypoint_index"] = 0
            self.mission_phase = "IDLE"
        self.logger.log_event(f"Waypoints baru dimuat. Jumlah: {len(payload)}")

    def _handle_start_mission(self, payload):
        with self.state_lock:
            if not self.current_state["waypoints"]:
                print("[AsvHandler] Tidak bisa memulai misi: Tidak ada waypoint.")
                return
            self.current_state["control_mode"] = "AUTO"
            self.mission_phase = "NAVIGATING"
        self.logger.log_event("Misi navigasi dimulai.")

    def _handle_initiate_rth(self, payload):
        with self.state_lock:
            if not self.current_state["waypoints"]:
                print("[AsvHandler] Tidak bisa RTH: Tidak ada home point (waypoint pertama).")
                return
            home_point = self.current_state["waypoints"][0]
            self.current_state["waypoints"] = [home_point]
            self.current_state["current_waypoint_index"] = 0
            self.current_state["control_mode"] = "AUTO"
            self.mission_phase = "RETURNING_HOME"
        self.logger.log_event("Memulai Return to Home.")

    def stop(self):
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        print("[AsvHandler] Dihentikan.")