# backend/core/asv_handler.py
# --- VERSI FINAL THREAD-SAFE ---

import threading
import time
import math
import numpy as np
from PySide6.QtCore import QObject, Signal, Slot

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
        self.state_lock = threading.Lock()  # Kunci utama untuk state

        # State yang dilindungi oleh state_lock
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

        self.phase_transition_wp_index = {"NAVIGATING_GATES": 7}
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
            with self.state_lock:
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
            with self.state_lock:
                parts = line.strip("T:").split(";")
                for part in parts:
                    data = part.split(",")
                    if data[0] == "GPS":
                        (
                            self.current_state["latitude"],
                            self.current_state["longitude"],
                        ) = float(data[1]), float(data[2])
                    elif data[0] == "COMP":
                        self.current_state["heading"] = float(data[1])
                    elif data[0] == "SPD":
                        self.current_state["speed"] = float(data[1])
                    elif data[0] == "BAT":
                        self.current_state["battery_voltage"] = float(data[1])
                    elif data[0] == "IMU":
                        self.current_state["accel_x"], self.current_state["gyro_z"] = (
                            float(data[1]),
                            float(data[2]),
                        )
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
                    self.current_state["heading"] = (
                        np.degrees(fused_heading_rad) + 360
                    ) % 360
                    self.current_state["speed"] = fused_speed

            # --- PEMBARUAN STATE LAINNYA ---
            elapsed = time.time() - start_time
            mission_time_str = time.strftime("%H:%M:%S", time.gmtime(elapsed))
            new_connection_status = self.serial_handler.is_connected

            with self.state_lock:
                self.current_state["mission_time"] = mission_time_str
                if (
                    self.current_state["is_connected_to_serial"]
                    != new_connection_status
                ):
                    self.current_state["is_connected_to_serial"] = new_connection_status
                    status = "CONNECTED" if new_connection_status else "DISCONNECTED"
                    self.current_state["status"] = status
                    self.logger.log_event(f"Status Koneksi Serial: {status}")

            # --- AMBIL SALINAN STATE UNTUK LOGIKA ---
            with self.state_lock:
                state_for_logic = self.current_state.copy()
                mission_phase_for_logic = self.mission_phase
                is_rth_for_logic = self.is_returning_home

            if mission_phase_for_logic == "INVESTIGATING":
                if state_for_logic["current_waypoint_index"] >= len(
                    state_for_logic["waypoints"]
                ):
                    self.logger.log_event("Tiba di lokasi POI. Memulai dokumentasi.")
                    print(
                        "[AsvHandler] 📸 Tiba di lokasi POI. Memulai dokumentasi selama 5 detik..."
                    )
                    time.sleep(5)
                    self.logger.log_event("Dokumentasi POI selesai.")
                    print("[AsvHandler] Dokumentasi selesai. Kembali ke rute patroli.")
                    self._resume_patrol()

            self._check_mission_phase_transition()

            if state_for_logic.get("control_mode") == "AUTO":
                servo, pwm = 0, 0
                if (
                    is_rth_for_logic
                    and self.vision_target["active"]
                    and time.time() > self.rth_paused_until
                ):
                    servo = self.config.get("actuators", {}).get(
                        "servo_default_angle", 90
                    )
                    pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
                    self.rth_paused_until = time.time() + 5
                elif time.time() < self.rth_paused_until:
                    servo = self.config.get("actuators", {}).get(
                        "servo_default_angle", 90
                    )
                    pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
                elif self.vision_target["active"]:
                    servo, pwm = self.convert_degree_to_actuators(
                        self.vision_target["degree"],
                        self.vision_target.get("is_inverted", False),
                    )
                else:
                    servo, pwm = run_navigation_logic(
                        state_for_logic, self.config, self.pid_controller
                    )
                    if is_rth_for_logic and state_for_logic[
                        "current_waypoint_index"
                    ] >= len(state_for_logic["waypoints"]):
                        self._stop_rth()
                self.serial_handler.send_command(f"S{pwm};D{servo}\n")

            self.logger.log_telemetry(state_for_logic)
            time.sleep(0.2)
            self._update_and_emit_state()

    def _check_mission_phase_transition(self):
        with self.state_lock:
            current_wp_index = self.current_state["current_waypoint_index"]
            waypoints_len = len(self.current_state["waypoints"])
            current_phase = self.mission_phase
            is_rth = self.is_returning_home

        if current_phase == "NAVIGATING_GATES":
            transition_index = self.phase_transition_wp_index["NAVIGATING_GATES"]
            if current_wp_index >= transition_index:
                self._set_mission_phase("SEARCHING_GREEN_BOX")
        if (
            current_phase not in ["IDLE", "MISSION_COMPLETE"]
            and not is_rth
            and current_wp_index >= waypoints_len
        ):
            if current_phase != "INVESTIGATING":
                self._set_mission_phase("MISSION_COMPLETE")

    def _set_mission_phase(self, new_phase):
        with self.state_lock:
            if self.mission_phase == new_phase:
                return
            self.mission_phase = new_phase
            self.current_state["status"] = self.mission_phase
            if new_phase == "MISSION_COMPLETE":
                self.current_state["control_mode"] = "MANUAL"

        self.logger.log_event(f"Fase misi diubah ke: {new_phase}")
        print(f"[AsvHandler] Fase misi diubah ke: {new_phase}")

        target_map = {
            "PATROLLING": "ANOMALY",
            "NAVIGATING_GATES": "BUOYS",
            "SEARCHING_GREEN_BOX": "GREEN_BOX",
            "SEARCHING_BLUE_BOX": "BLUE_BOX",
        }
        self.mission_target_changed.emit(target_map.get(new_phase, "NONE"))

    def process_command(self, command, payload):
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
        self.logger.log_event(
            f"Menerima perintah investigasi untuk POI: {payload.get('class_name')}"
        )
        print("[AsvHandler] Menerima perintah investigasi. Menginterupsi patroli...")

        with self.state_lock:
            self.current_state["original_patrol_waypoints"] = list(
                self.current_state["waypoints"]
            )
            self.current_state["last_patrol_waypoint_index"] = self.current_state[
                "current_waypoint_index"
            ]
            current_lat = self.current_state["latitude"]
            current_lon = self.current_state["longitude"]

        # ... (kalkulasi waypoint investigasi tetap sama)
        investigation_distance_m, approach_distance_m = 10.0, 5.0
        target_bearing_rad = math.radians(payload.get("bearing_deg"))
        R = 6371000
        lat1, lon1 = math.radians(current_lat), math.radians(current_lon)
        d = investigation_distance_m - approach_distance_m
        lat2 = math.asin(
            math.sin(lat1) * math.cos(d / R)
            + math.cos(lat1) * math.sin(d / R) * math.cos(target_bearing_rad)
        )
        lon2 = lon1 + math.atan2(
            math.sin(target_bearing_rad) * math.sin(d / R) * math.cos(lat1),
            math.cos(d / R) - math.sin(lat1) * math.sin(lat2),
        )
        investigation_waypoint = {"lat": math.degrees(lat2), "lon": math.degrees(lon2)}

        with self.state_lock:
            self.current_state["waypoints"] = [investigation_waypoint]
            self.current_state["current_waypoint_index"] = 0

        self.pid_controller.reset()
        self._set_mission_phase("INVESTIGATING")

    def _resume_patrol(self):
        with self.state_lock:
            self.current_state["waypoints"] = list(
                self.current_state["original_patrol_waypoints"]
            )
            self.current_state["current_waypoint_index"] = self.current_state[
                "last_patrol_waypoint_index"
            ]
            self.current_state["original_patrol_waypoints"] = []
        self.pid_controller.reset()
        self._set_mission_phase("PATROLLING")
        self.logger.log_event("Melanjutkan patroli dari waypoint terakhir.")
        print("[AsvHandler] Melanjutkan patroli dari waypoint terakhir.")

    def _handle_start_mission(self, payload):
        with self.state_lock:
            if not self.current_state["waypoints"]:
                print(
                    "[AsvHandler] Tidak bisa memulai misi: Tidak ada waypoint yang dimuat."
                )
                self.logger.log_event("Gagal memulai misi: tidak ada waypoint.")
                return
            self.current_state["control_mode"] = "AUTO"
        self.logger.log_event("Misi Patroli Dimulai.")
        print("[AsvHandler] Misi Patroli Dimulai!")
        self.pid_controller.reset()
        self._set_mission_phase("PATROLLING")

    def _handle_photography_complete(self, payload):
        object_type = payload.get("object")
        print(f"[AsvHandler] Menerima konfirmasi fotografi untuk: {object_type}")
        if object_type == "green_box":
            self._set_mission_phase("SEARCHING_BLUE_BOX")
        elif object_type == "blue_box":
            self._set_mission_phase("RACING_TO_FINISH")

    def _handle_serial_configuration(self, payload):
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO":
            self.serial_handler.find_and_connect_esp32(baud)
        else:
            self.serial_handler.connect(port, baud)

    def _handle_mode_change(self, payload):
        with self.state_lock:
            # Pemeriksaan kondisi dilakukan di dalam lock untuk konsistensi
            is_mission_active = (
                self.mission_phase not in ["IDLE", "MISSION_COMPLETE"]
                and not self.is_returning_home
            )
            if is_mission_active:
                self.logger.log_event(
                    f"Gagal mengubah mode ke {payload} saat misi aktif."
                )
                print("[AsvHandler] Tidak bisa mengubah mode saat misi aktif.")
                return
            self.current_state["control_mode"] = payload
        self.logger.log_event(f"Mode kontrol diubah ke: {payload}")

    def _handle_manual_control(self, payload):
        with self.state_lock:
            if self.current_state.get("control_mode") != "MANUAL":
                return
        # ... (logika kontrol manual tidak perlu lock karena tidak mengubah state)
        keys = set(payload)
        actuator_config = self.config.get("actuators", {})
        pwm_stop, pwr = actuator_config.get(
            "motor_pwm_stop", 1500
        ), actuator_config.get("motor_pwm_manual_power", 150)
        servo_def, servo_min = actuator_config.get(
            "servo_default_angle", 90
        ), actuator_config.get("servo_min_angle", 45)
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

    def _handle_vision_target_update(self, payload):
        with self.state_lock:
            self.vision_target["active"] = payload.get("active", False)
            self.vision_target["degree"] = payload.get("degree", 90)
            self.vision_target["is_inverted"] = payload.get("is_inverted", False)

    def _handle_initiate_rth(self, payload):
        with self.state_lock:
            if not self.current_state["is_connected_to_serial"]:
                self.logger.log_event("Gagal memulai RTH: Tidak terhubung ke hardware.")
                print(
                    "[AsvHandler] Tidak bisa memulai RTH: Tidak terhubung ke hardware."
                )
                return

            print("[AsvHandler] Memulai sekuens Return to Home...")
            home_point = (
                self.current_state["waypoints"][0]
                if self.current_state["waypoints"]
                else {
                    "lat": self.current_state["latitude"],
                    "lon": self.current_state["longitude"],
                }
            )
            self.current_state["waypoints"] = [home_point]
            self.current_state["current_waypoint_index"] = 0
            self.current_state["control_mode"] = "AUTO"
            self.current_state["status"] = "RETURNING TO HOME"
            self.is_returning_home = True
            self.mission_phase = "IDLE"

        self.logger.log_event("Memulai sekuens Return to Home.")
        self.pid_controller.reset()
        print(f"[AsvHandler] RTH diaktifkan. Target: {home_point}")

    def _stop_rth(self):
        with self.state_lock:
            self.is_returning_home = False
            self.rth_paused_until = 0
            self.current_state["control_mode"] = "MANUAL"
            self.current_state["status"] = "CONNECTED"
        self.logger.log_event("RTH selesai. Kembali ke mode MANUAL.")
        stop_pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
        default_servo = self.config.get("actuators", {}).get("servo_default_angle", 90)
        self.serial_handler.send_command(f"S{stop_pwm};D{default_servo}\n")

    def convert_degree_to_actuators(self, degree, is_inverted):
        error = degree - 90
        if is_inverted:
            error = -error
        actuators = self.config.get("actuators", {})
        servo_def, servo_min = actuators.get("servo_default_angle", 90), actuators.get(
            "servo_min_angle", 45
        )
        servo_range = servo_def - servo_min
        servo = int(np.clip(servo_def - (error / 90.0) * servo_range, 0, 180))
        motor_base, reduction = actuators.get(
            "motor_pwm_auto_base", 1650
        ), actuators.get("motor_pwm_auto_reduction", 100)
        pwm = motor_base - (abs(error) / 90.0) * reduction
        return int(servo), int(pwm)

    def get_current_state(self):
        with self.state_lock:
            return self.current_state.copy()

    def stop(self):
        self.running = False
        self.serial_handler.disconnect()
        self.logger.log_event("AsvHandler dihentikan.")
        self.logger.stop()
        print("[AsvHandler] Dihentikan.")
