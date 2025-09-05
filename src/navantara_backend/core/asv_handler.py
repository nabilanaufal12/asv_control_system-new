# backend/core/asv_handler.py
# --- VERSI MODIFIKASI LENGKAP: Integrasi EKF & Logger ---

import threading
import time
import math
import numpy as np
from PySide6.QtCore import QObject, Signal, Slot

# Impor kelas-kelas baru
from navantara_backend.services.serial_service import SerialHandler
from navantara_backend.core.navigation import run_navigation_logic, PIDController
from navantara_backend.core.kalman_filter import SimpleEKF
from navantara_backend.core.mission_logger import MissionLogger


class AsvHandler(QObject):
    telemetry_updated = Signal(dict)
    mission_target_changed = Signal(str)

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.serial_handler = SerialHandler(config)
        self.running = True

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
            # State baru untuk data IMU mentah
            "gyro_z": 0.0,
            "accel_x": 0.0,
        }

        self.vision_target = {"active": False, "degree": 90}
        self.is_returning_home = False
        self.rth_paused_until = 0
        self.mission_phase = "IDLE"
        self.phase_transition_wp_index = {"NAVIGATING_GATES": 7}

        pid_config = self.config.get("navigation", {}).get("heading_pid", {})
        self.pid_controller = PIDController(
            Kp=pid_config.get("kp", 1.0),
            Ki=pid_config.get("ki", 0.0),
            Kd=pid_config.get("kd", 0.0),
        )

        # Inisialisasi EKF
        initial_state = np.zeros(5)  # [x, y, heading, v, omega]
        initial_covariance = np.eye(5) * 0.1
        self.ekf = SimpleEKF(initial_state, initial_covariance)
        self.last_ekf_update_time = time.time()

        # Inisialisasi Logger
        self.logger = MissionLogger()
        self.logger.log_event("AsvHandler diinisialisasi.")
        print(
            "[AsvHandler] Handler diinisialisasi dengan Kontroler PID, EKF, Logger, dan logika Investigasi."
        )

    @Slot()
    def run(self):
        self._read_thread = threading.Thread(
            target=self._read_from_serial_loop, daemon=True
        )
        self._logic_thread = threading.Thread(target=self._main_logic_loop, daemon=True)
        self._read_thread.start()
        self._logic_thread.start()
        print("[AsvHandler] Thread-thread internal dimulai.")

    def _update_and_emit_state(self):
        if self.running:
            state_copy = self.current_state.copy()
            state_copy["mission_phase"] = self.mission_phase
            self.telemetry_updated.emit(state_copy)

    def _read_from_serial_loop(self):
        while self.running:
            line = self.serial_handler.read_line()
            if line and line.startswith("T:"):
                self._parse_telemetry(line)
            else:
                time.sleep(0.1)

    def _parse_telemetry(self, line):
        try:
            parts = line.strip("T:").split(";")
            for part in parts:
                data = part.split(",")
                if data[0] == "GPS":
                    self.current_state["latitude"] = float(data[1])
                    self.current_state["longitude"] = float(data[2])
                elif data[0] == "COMP":
                    self.current_state["heading"] = float(data[1])
                elif data[0] == "SPD":
                    self.current_state["speed"] = float(data[1])
                elif data[0] == "BAT":
                    self.current_state["battery_voltage"] = float(data[1])
                # Tambahkan parsing untuk data IMU baru dari ESP32
                elif data[0] == "IMU":
                    self.current_state["accel_x"] = float(data[1])
                    self.current_state["gyro_z"] = float(data[2])
        except (ValueError, IndexError):
            pass

    def _main_logic_loop(self):
        start_time = time.time()
        while self.running:
            # --- BLOK LOGIKA EKF ---
            current_time = time.time()
            dt = current_time - self.last_ekf_update_time
            if dt > 0:
                self.last_ekf_update_time = current_time
                # 1. Tahap Prediksi EKF
                self.ekf.predict(dt)

                # 2. Tahap Update EKF dengan pengukuran sensor
                self.ekf.update_compass(np.radians(self.current_state["heading"]))
                self.ekf.update_imu(np.array([self.current_state["speed"], np.radians(self.current_state["gyro_z"])]))
                
                # 3. Ambil hasil fusi dari EKF
                fused_heading_rad = self.ekf.state[2]
                fused_speed = self.ekf.state[3]
                
                # 4. Ganti nilai di state utama dengan hasil fusi yang lebih stabil
                self.current_state["heading"] = (np.degrees(fused_heading_rad) + 360) % 360
                self.current_state["speed"] = fused_speed
            # --- AKHIR BLOK EKF ---

            elapsed = time.time() - start_time
            self.current_state["mission_time"] = time.strftime(
                "%H:%M:%S", time.gmtime(elapsed)
            )

            new_connection_status = self.serial_handler.is_connected
            if self.current_state["is_connected_to_serial"] != new_connection_status:
                self.current_state["is_connected_to_serial"] = new_connection_status
                status = "CONNECTED" if new_connection_status else "DISCONNECTED"
                self.current_state["status"] = status
                self.logger.log_event(f"Status Koneksi Serial: {status}")


            if self.mission_phase == "INVESTIGATING":
                if self.current_state["current_waypoint_index"] >= len(self.current_state["waypoints"]):
                    self.logger.log_event("Tiba di lokasi POI. Memulai dokumentasi.")
                    print("[AsvHandler] 📸 Tiba di lokasi POI. Memulai dokumentasi selama 5 detik...")
                    time.sleep(5)
                    self.logger.log_event("Dokumentasi POI selesai.")
                    print("[AsvHandler] Dokumentasi selesai. Kembali ke rute patroli.")
                    self._resume_patrol()

            self._check_mission_phase_transition()

            if self.current_state.get("control_mode") == "AUTO":
                servo, pwm = 0, 0
                if (self.is_returning_home and self.vision_target["active"] and time.time() > self.rth_paused_until):
                    servo = self.config.get("actuators", {}).get("servo_default_angle", 90)
                    pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
                    self.rth_paused_until = time.time() + 5
                elif time.time() < self.rth_paused_until:
                    servo = self.config.get("actuators", {}).get("servo_default_angle", 90)
                    pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
                elif self.vision_target["active"]:
                    servo, pwm = self.convert_degree_to_actuators(
                        self.vision_target["degree"], self.vision_target.get("is_inverted", False)
                    )
                else:
                    servo, pwm = run_navigation_logic(
                        self.current_state, self.config, self.pid_controller
                    )
                    if (self.is_returning_home and self.current_state["current_waypoint_index"] >= len(self.current_state["waypoints"])):
                        self._stop_rth()
                self.serial_handler.send_command(f"S{pwm};D{servo}\n")

            # Log data telemetri di setiap iterasi
            self.logger.log_telemetry(self.current_state)

            time.sleep(0.2)
            self._update_and_emit_state()

    def _check_mission_phase_transition(self):
        # ... (fungsi tidak berubah)
        current_wp_index = self.current_state["current_waypoint_index"]
        if self.mission_phase == "NAVIGATING_GATES":
            transition_index = self.phase_transition_wp_index["NAVIGATING_GATES"]
            if current_wp_index >= transition_index:
                self._set_mission_phase("SEARCHING_GREEN_BOX")
        if (self.mission_phase not in ["IDLE", "MISSION_COMPLETE"] and not self.is_returning_home and current_wp_index >= len(self.current_state["waypoints"])):
            if self.mission_phase != "INVESTIGATING":
                self._set_mission_phase("MISSION_COMPLETE")


    def _set_mission_phase(self, new_phase):
        if self.mission_phase != new_phase:
            self.mission_phase = new_phase
            self.logger.log_event(f"Fase misi diubah ke: {self.mission_phase}")
            print(f"[AsvHandler] Fase misi diubah ke: {self.mission_phase}")
            self.current_state["status"] = self.mission_phase
            if new_phase == "PATROLLING":
                self.mission_target_changed.emit("ANOMALY")
            elif new_phase == "NAVIGATING_GATES":
                self.mission_target_changed.emit("BUOYS")
            elif new_phase == "SEARCHING_GREEN_BOX":
                self.mission_target_changed.emit("GREEN_BOX")
            elif new_phase == "SEARCHING_BLUE_BOX":
                self.mission_target_changed.emit("BLUE_BOX")
            elif new_phase == "MISSION_COMPLETE":
                print("[AsvHandler] Misi Selesai!")
                self.current_state["control_mode"] = "MANUAL"
            else:
                self.mission_target_changed.emit("NONE")

    def process_command(self, command, payload):
        # ... (fungsi tidak berubah)
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control,
            "SET_WAYPOINTS": self._handle_set_waypoints,
            "VISION_TARGET_UPDATE": self._handle_vision_target_update,
            "INITIATE_RTH": self._handle_initiate_rth,
            "START_MISSION": self._handle_start_mission,
            "PHOTOGRAPHY_COMPLETE": self._handle_photography_complete,
            "INVESTIGATE_POI": self._handle_investigate_poi,
        }
        handler = command_handlers.get(command)
        if handler:
            handler(payload)
            self._update_and_emit_state()
        else:
            print(f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'")

    def _handle_investigate_poi(self, payload):
        self.logger.log_event(f"Menerima perintah investigasi untuk POI: {payload.get('class_name')}")
        print("[AsvHandler] Menerima perintah investigasi. Menginterupsi patroli...")
        self.current_state["original_patrol_waypoints"] = list(self.current_state["waypoints"])
        self.current_state["last_patrol_waypoint_index"] = self.current_state["current_waypoint_index"]
        
        investigation_distance_m = 10.0
        approach_distance_m = 5.0
        target_bearing_rad = math.radians(payload.get("bearing_deg"))
        
        R = 6371000
        lat1 = math.radians(self.current_state["latitude"])
        lon1 = math.radians(self.current_state["longitude"])
        
        d = investigation_distance_m - approach_distance_m
        lat2 = math.asin(math.sin(lat1) * math.cos(d / R) + math.cos(lat1) * math.sin(d / R) * math.cos(target_bearing_rad))
        lon2 = lon1 + math.atan2(math.sin(target_bearing_rad) * math.sin(d / R) * math.cos(lat1), math.cos(d / R) - math.sin(lat1) * math.sin(lat2))
        
        investigation_waypoint = {"lat": math.degrees(lat2), "lon": math.degrees(lon2)}
        self.logger.log_event(f"Waypoint investigasi dibuat di: {investigation_waypoint}")
        print(f"[AsvHandler] Waypoint investigasi dibuat di: {investigation_waypoint}")
        
        self.current_state["waypoints"] = [investigation_waypoint]
        self.current_state["current_waypoint_index"] = 0
        self.pid_controller.reset()
        self._set_mission_phase("INVESTIGATING")

    def _resume_patrol(self):
        self.current_state["waypoints"] = list(self.current_state["original_patrol_waypoints"])
        self.current_state["current_waypoint_index"] = self.current_state["last_patrol_waypoint_index"]
        self.current_state["original_patrol_waypoints"] = []
        self.pid_controller.reset()
        self._set_mission_phase("PATROLLING")
        self.logger.log_event("Melanjutkan patroli dari waypoint terakhir.")
        print("[AsvHandler] Melanjutkan patroli dari waypoint terakhir.")

    def _handle_start_mission(self, payload):
        if not self.current_state["waypoints"]:
            print("[AsvHandler] Tidak bisa memulai misi: Tidak ada waypoint yang dimuat.")
            self.logger.log_event("Gagal memulai misi: tidak ada waypoint.")
            return
        self.logger.log_event("Misi Patroli Dimulai.")
        print("[AsvHandler] Misi Patroli Dimulai!")
        self.current_state["control_mode"] = "AUTO"
        self.pid_controller.reset()
        self._set_mission_phase("PATROLLING")

    def _handle_photography_complete(self, payload):
        # ... (fungsi tidak berubah)
        object_type = payload.get("object")
        print(f"[AsvHandler] Menerima konfirmasi fotografi untuk: {object_type}")
        if object_type == "green_box":
            self._set_mission_phase("SEARCHING_BLUE_BOX")
        elif object_type == "blue_box":
            self._set_mission_phase("RACING_TO_FINISH")

    def _handle_serial_configuration(self, payload):
        # ... (fungsi tidak berubah)
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO":
            self.serial_handler.find_and_connect_esp32(baud)
        else:
            self.serial_handler.connect(port, baud)

    def _handle_mode_change(self, payload):
        if (self.mission_phase not in ["IDLE", "MISSION_COMPLETE"] and not self.is_returning_home):
            self.logger.log_event(f"Gagal mengubah mode ke {payload} saat misi aktif.")
            print("[AsvHandler] Tidak bisa mengubah mode saat misi aktif.")
            return
        self.current_state["control_mode"] = payload
        self.logger.log_event(f"Mode kontrol diubah ke: {payload}")

    def _handle_manual_control(self, payload):
        # ... (fungsi tidak berubah)
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
        self.current_state["waypoints"] = payload
        self.current_state["current_waypoint_index"] = 0
        self.mission_phase = "IDLE"
        self.logger.log_event(f"Waypoints baru dimuat. Jumlah: {len(payload)}")

    def _handle_vision_target_update(self, payload):
        # ... (fungsi tidak berubah)
        self.vision_target["active"] = payload.get("active", False)
        self.vision_target["degree"] = payload.get("degree", 90)
        self.vision_target["is_inverted"] = payload.get("is_inverted", False)

    def _handle_initiate_rth(self, payload):
        if not self.current_state["is_connected_to_serial"]:
            self.logger.log_event("Gagal memulai RTH: Tidak terhubung ke hardware.")
            print("[AsvHandler] Tidak bisa memulai RTH: Tidak terhubung ke hardware.")
            return
        self.logger.log_event("Memulai sekuens Return to Home.")
        print("[AsvHandler] Memulai sekuens Return to Home...")
        home_point = None
        if self.current_state["waypoints"]:
            home_point = self.current_state["waypoints"][0]
        else:
            home_point = {"lat": self.current_state["latitude"], "lon": self.current_state["longitude"]}
        rth_waypoints = [home_point]
        self.current_state["waypoints"] = rth_waypoints
        self.current_state["current_waypoint_index"] = 0
        self.current_state["control_mode"] = "AUTO"
        self.current_state["status"] = "RETURNING TO HOME"
        self.is_returning_home = True
        self.mission_phase = "IDLE"
        self.pid_controller.reset()
        print(f"[AsvHandler] RTH diaktifkan. Target: {home_point}")

    def _stop_rth(self):
        self.logger.log_event("RTH selesai. Kembali ke mode MANUAL.")
        self.is_returning_home = False
        self.rth_paused_until = 0
        self.current_state["control_mode"] = "MANUAL"
        self.current_state["status"] = "CONNECTED"
        stop_pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
        default_servo = self.config.get("actuators", {}).get("servo_default_angle", 90)
        self.serial_handler.send_command(f"S{stop_pwm};D{default_servo}\n")

    def convert_degree_to_actuators(self, degree, is_inverted):
        # ... (fungsi tidak berubah)
        error = degree - 90
        if is_inverted:
            error = -error
        actuators = self.config.get("actuators", {})
        servo_def = actuators.get("servo_default_angle", 90)
        servo_min = actuators.get("servo_min_angle", 45)
        servo_range = servo_def - servo_min
        servo = servo_def - (error / 90.0) * servo_range
        servo = int(np.clip(servo, 0, 180))
        motor_base = actuators.get("motor_pwm_auto_base", 1650)
        reduction = actuators.get("motor_pwm_auto_reduction", 100)
        pwm = motor_base - (abs(error) / 90.0) * reduction
        return int(servo), int(pwm)


    def get_current_state(self):
        return self.current_state.copy()

    def stop(self):
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()  # Pastikan logger ditutup dengan benar
        print("[AsvHandler] Dihentikan.")