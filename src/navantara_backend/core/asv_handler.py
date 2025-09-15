# src/navantara_backend/core/asv_handler.py
import threading
import time
import math
import numpy as np

# --- PERUBAHAN UTAMA: Hapus semua impor PySide6 (QObject, Signal, Slot) ---
from navantara_backend.services.serial_service import SerialHandler
from navantara_backend.core.navigation import run_navigation_logic, PIDController
from navantara_backend.core.kalman_filter import SimpleEKF
from navantara_backend.core.mission_logger import MissionLogger


class AsvHandler:
    """
    Mengelola state, logika, dan komunikasi hardware dari ASV.
    Berjalan sebagai greenlet independen dan berkomunikasi melalui WebSockets.
    """
    
    # --- PERUBAHAN: Hapus warisan dari QObject dan sinyal-sinyal Qt ---

    def __init__(self, config, socketio):
        # super().__init__() # Dihapus
        self.config = config
        self.socketio = socketio  # Simpan instance socketio untuk komunikasi
        self.serial_handler = SerialHandler(config)
        self.running = True
        self.state_lock = threading.Lock()  # Lock tetap digunakan untuk thread-safety state
        
        # Flag baru untuk mengontrol pengiriman data, diatur oleh permintaan klien
        self.is_streaming_to_gui = False

        # State yang dilindungi oleh state_lock (struktur tidak berubah)
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
            "last_patrol_waypoint_index": 0,
            "original_patrol_waypoints": [],
            "points_of_interest": [],
            "gyro_z": 0.0,
            "accel_x": 0.0,
        }
        self.vision_target = {"active": False, "degree": 90}
        self.is_returning_home = False
        self.rth_paused_until = 0
        self.mission_phase = "IDLE"

        pid_config = self.config.get("navigation", {}).get("heading_pid", {})
        self.pid_controller = PIDController(
            Kp=pid_config.get("kp", 1.0),
            Ki=pid_config.get("ki", 0.0),
            Kd=pid_config.get("kd", 0.0),
        )
        self.ekf = SimpleEKF(np.zeros(5), np.eye(5) * 0.1)
        self.last_ekf_update_time = time.time()
        self.logger = MissionLogger()
        self.logger.log_event("AsvHandler diinisialisasi.")
        print("[AsvHandler] Handler diinisialisasi untuk operasi backend.")

    def _update_and_emit_state(self):
        """
        Mengirim state terbaru ke klien GUI melalui WebSocket jika streaming aktif.
        """
        if self.running and self.is_streaming_to_gui:
            with self.state_lock:
                state_copy = self.current_state.copy()
                state_copy["mission_phase"] = self.mission_phase
            # --- PERUBAHAN UTAMA: Menggunakan socketio.emit alih-alih sinyal Qt ---
            self.socketio.emit('telemetry_update', state_copy)

    def _read_from_serial_loop(self):
        """Loop yang berjalan sebagai greenlet untuk membaca data dari hardware."""
        while self.running:
            line = self.serial_handler.read_line()
            if line and line.startswith("T:"):
                self._parse_telemetry(line)
            else:
                # eventlet.sleep() (via socketio.sleep) penting untuk kerja sama (cooperative yielding)
                self.socketio.sleep(0.1)

    def _parse_telemetry(self, line):
        """Mem-parsing data telemetri dari string serial."""
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
                    elif data[0] == "BAT":
                        self.current_state["battery_voltage"] = float(data[1])
                    elif data[0] == "IMU":
                        self.current_state["accel_x"], self.current_state["gyro_z"] = float(data[1]), float(data[2])
        except (ValueError, IndexError):
            pass

    def main_logic_loop(self):
        """
        Loop utama layanan. Dijalankan oleh eventlet.spawn dari app factory.
        """
        # Jalankan pembaca serial di greenlet terpisah yang dikelola oleh SocketIO/Eventlet
        self.socketio.start_background_task(self._read_from_serial_loop)
        print("[AsvHandler] Loop pembaca serial dimulai sebagai greenlet.")

        start_time = time.time()
        while self.running:
            # --- BLOK LOGIKA EKF ---
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

                fused_heading_rad = self.ekf.state[2]
                fused_speed = self.ekf.state[3]

                with self.state_lock:
                    self.current_state["heading"] = (np.degrees(fused_heading_rad) + 360) % 360
                    self.current_state["speed"] = fused_speed

            # --- PEMBARUAN STATE LAINNYA ---
            elapsed = time.time() - start_time
            mission_time_str = time.strftime("%H:%M:%S", time.gmtime(elapsed))
            new_connection_status = self.serial_handler.is_connected

            with self.state_lock:
                self.current_state["mission_time"] = mission_time_str
                if self.current_state["is_connected_to_serial"] != new_connection_status:
                    self.current_state["is_connected_to_serial"] = new_connection_status
                    status = "CONNECTED" if new_connection_status else "DISCONNECTED"
                    self.current_state["status"] = status
                    self.logger.log_event(f"Status Koneksi Serial: {status}")

            # --- LOGIKA KONTROL & NAVIGASI ---
            with self.state_lock:
                state_for_logic = self.current_state.copy()
            
            if state_for_logic.get("control_mode") == "AUTO":
                servo, pwm = run_navigation_logic(state_for_logic, self.config, self.pid_controller)
                self.serial_handler.send_command(f"S{pwm};D{servo}\n")

            self.logger.log_telemetry(state_for_logic)
            self._update_and_emit_state()

            # --- PERUBAHAN UTAMA: Ganti time.sleep dengan socketio.sleep ---
            self.socketio.sleep(0.2)

    def set_streaming_status(self, status: bool):
        """Metode untuk mengontrol pengiriman data telemetri ke GUI."""
        if self.is_streaming_to_gui != status:
            print(f"[AsvHandler] Status streaming telemetri diatur ke: {status}")
            self.is_streaming_to_gui = status

    def process_command(self, command, payload):
        """Memproses perintah yang datang dari endpoint API."""
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control,
            "SET_WAYPOINTS": self._handle_set_waypoints,
            "NAV_START": self._handle_start_mission,
            "NAV_RETURN": self._handle_initiate_rth,
        }
        handler = command_handlers.get(command)
        if handler:
            handler(payload)
        else:
            print(f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'")
            
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
        """Menghentikan layanan dengan aman."""
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        print("[AsvHandler] Dihentikan.")