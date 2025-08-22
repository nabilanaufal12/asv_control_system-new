# backend/core/asv_handler.py
# --- VERSI MODIFIKASI: Dengan State Machine Patroli & Investigasi ---

import threading
import time
import math  # Diperlukan untuk kalkulasi waypoint investigasi
from backend.services.serial_service import SerialHandler
from backend.core.navigation import run_navigation_logic, PIDController
from PySide6.QtCore import QObject, Signal, Slot
import numpy as np


class AsvHandler(QObject):
    telemetry_updated = Signal(dict)
    mission_target_changed = Signal(str)

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.serial_handler = SerialHandler(config)
        self.running = True

        # --- PERUBAHAN 1: Penambahan state baru untuk investigasi ---
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
            # State baru untuk menyimpan data POI dan status patroli
            "last_patrol_waypoint_index": 0,
            "original_patrol_waypoints": [],
            "points_of_interest": [],
        }
        # -----------------------------------------------------------

        self.vision_target = {"active": False, "degree": 90}
        self.is_returning_home = False
        self.rth_paused_until = 0

        # Fase misi sekarang mencakup mode patroli dan investigasi
        self.mission_phase = "IDLE"  # State: IDLE, PATROLLING, INVESTIGATING, RESUMING
        self.phase_transition_wp_index = {
            "NAVIGATING_GATES": 7,
        }

        pid_config = self.config.get("navigation", {}).get("heading_pid", {})
        self.pid_controller = PIDController(
            Kp=pid_config.get("kp", 1.0),
            Ki=pid_config.get("ki", 0.0),
            Kd=pid_config.get("kd", 0.0),
        )
        print(
            "[AsvHandler] Handler diinisialisasi dengan Kontroler PID dan logika Investigasi."
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
        except (ValueError, IndexError):
            pass

    def _main_logic_loop(self):
        start_time = time.time()
        while self.running:
            elapsed = time.time() - start_time
            self.current_state["mission_time"] = time.strftime(
                "%H:%M:%S", time.gmtime(elapsed)
            )

            new_connection_status = self.serial_handler.is_connected
            if self.current_state["is_connected_to_serial"] != new_connection_status:
                self.current_state["is_connected_to_serial"] = new_connection_status
                self.current_state["status"] = (
                    "CONNECTED" if new_connection_status else "DISCONNECTED"
                )

            # --- PERUBAHAN 2: Logika utama sekarang menangani fase investigasi ---
            if self.mission_phase == "INVESTIGATING":
                # Jika waypoint investigasi tercapai (ASV berhenti di dekat POI)
                if self.current_state["current_waypoint_index"] >= len(
                    self.current_state["waypoints"]
                ):
                    print(
                        "[AsvHandler] 📸 Tiba di lokasi POI. Memulai dokumentasi selama 5 detik..."
                    )
                    # Di aplikasi nyata, ini akan memicu kamera untuk mengambil gambar
                    time.sleep(5)  # Simulasikan waktu untuk dokumentasi
                    print("[AsvHandler] Dokumentasi selesai. Kembali ke rute patroli.")
                    self._resume_patrol()  # Memulai proses kembali ke rute

            self._check_mission_phase_transition()
            # --------------------------------------------------------------------

            if self.current_state.get("control_mode") == "AUTO":
                servo, pwm = 0, 0

                if (
                    self.is_returning_home
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
                        self.current_state, self.config, self.pid_controller
                    )
                    if self.is_returning_home and self.current_state[
                        "current_waypoint_index"
                    ] >= len(self.current_state["waypoints"]):
                        self._stop_rth()

                self.serial_handler.send_command(f"S{pwm};D{servo}\n")

            time.sleep(0.2)
            self._update_and_emit_state()

    def _check_mission_phase_transition(self):
        current_wp_index = self.current_state["current_waypoint_index"]

        # Logika transisi lama tetap ada jika diperlukan
        if self.mission_phase == "NAVIGATING_GATES":
            transition_index = self.phase_transition_wp_index["NAVIGATING_GATES"]
            if current_wp_index >= transition_index:
                self._set_mission_phase("SEARCHING_GREEN_BOX")

        if (
            self.mission_phase not in ["IDLE", "MISSION_COMPLETE"]
            and not self.is_returning_home
            and current_wp_index >= len(self.current_state["waypoints"])
        ):
            # Jangan set ke MISSION_COMPLETE jika sedang investigasi
            if self.mission_phase != "INVESTIGATING":
                self._set_mission_phase("MISSION_COMPLETE")

    def _set_mission_phase(self, new_phase):
        if self.mission_phase != new_phase:
            self.mission_phase = new_phase
            print(f"[AsvHandler] Fase misi diubah ke: {self.mission_phase}")
            self.current_state["status"] = self.mission_phase

            if new_phase == "PATROLLING":
                self.mission_target_changed.emit(
                    "ANOMALY"
                )  # Beri tahu VisionService untuk mencari anomali
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
        # --- PERUBAHAN 3: Tambahkan handler untuk perintah investigasi baru ---
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control,
            "SET_WAYPOINTS": self._handle_set_waypoints,
            "VISION_TARGET_UPDATE": self._handle_vision_target_update,
            "INITIATE_RTH": self._handle_initiate_rth,
            "START_MISSION": self._handle_start_mission,
            "PHOTOGRAPHY_COMPLETE": self._handle_photography_complete,
            "INVESTIGATE_POI": self._handle_investigate_poi,  # Handler baru
        }
        # -------------------------------------------------------------------

        handler = command_handlers.get(command)
        if handler:
            handler(payload)
            self._update_and_emit_state()
        else:
            print(f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'")

    # --- PERUBAHAN 4: Buat handler dan fungsi pendukung untuk investigasi ---
    def _handle_investigate_poi(self, payload):
        """Menangani perintah dari VisionService untuk menginvestigasi POI."""
        print("[AsvHandler] Menerima perintah investigasi. Menginterupsi patroli...")

        # 1. Simpan konteks patroli saat ini
        self.current_state["original_patrol_waypoints"] = list(
            self.current_state["waypoints"]
        )
        self.current_state["last_patrol_waypoint_index"] = self.current_state[
            "current_waypoint_index"
        ]

        # 2. Hitung koordinat POI (dengan asumsi jarak)
        investigation_distance_m = 10.0  # Jarak asumsi ke target
        approach_distance_m = 5.0  # Jarak aman untuk berhenti

        target_bearing_rad = math.radians(payload.get("bearing_deg"))

        # Kalkulasi koordinat target sementara
        # Ini adalah penyederhanaan; pustaka geospasial akan lebih akurat
        R = 6371000  # Radius bumi
        lat1 = math.radians(self.current_state["latitude"])
        lon1 = math.radians(self.current_state["longitude"])

        # Hitung titik investigasi (5m sebelum target)
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
        print(f"[AsvHandler] Waypoint investigasi dibuat di: {investigation_waypoint}")

        # 3. Ganti rute ASV dengan rute investigasi
        self.current_state["waypoints"] = [investigation_waypoint]
        self.current_state["current_waypoint_index"] = 0
        self.pid_controller.reset()

        # 4. Ubah fase misi
        self._set_mission_phase("INVESTIGATING")

    def _resume_patrol(self):
        """Mengembalikan ASV ke rute patroli aslinya."""
        # 1. Kembalikan waypoint patroli
        self.current_state["waypoints"] = list(
            self.current_state["original_patrol_waypoints"]
        )
        self.current_state["current_waypoint_index"] = self.current_state[
            "last_patrol_waypoint_index"
        ]

        # 2. Hapus data sementara
        self.current_state["original_patrol_waypoints"] = []

        # 3. Reset PID dan ubah fase misi kembali ke patroli
        self.pid_controller.reset()
        self._set_mission_phase("PATROLLING")
        print("[AsvHandler] Melanjutkan patroli dari waypoint terakhir.")

    # --------------------------------------------------------------------

    def _handle_start_mission(self, payload):
        """Memulai misi dari fase pertama."""
        if not self.current_state["waypoints"]:
            print(
                "[AsvHandler] Tidak bisa memulai misi: Tidak ada waypoint yang dimuat."
            )
            return
        print("[AsvHandler] Misi Patroli Dimulai!")
        self.current_state["control_mode"] = "AUTO"
        self.pid_controller.reset()
        # Misi utama sekarang adalah PATROLLING
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
        if (
            self.mission_phase not in ["IDLE", "MISSION_COMPLETE"]
            and not self.is_returning_home
        ):
            print("[AsvHandler] Tidak bisa mengubah mode saat misi aktif.")
            return
        self.current_state["control_mode"] = payload

    def _handle_manual_control(self, payload):
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

    def _handle_vision_target_update(self, payload):
        self.vision_target["active"] = payload.get("active", False)
        self.vision_target["degree"] = payload.get("degree", 90)
        self.vision_target["is_inverted"] = payload.get("is_inverted", False)

    def _handle_initiate_rth(self, payload):
        if not self.current_state["is_connected_to_serial"]:
            print("[AsvHandler] Tidak bisa memulai RTH: Tidak terhubung ke hardware.")
            return
        print("[AsvHandler] Memulai sekuens Return to Home...")
        home_point = None
        if self.current_state["waypoints"]:
            home_point = self.current_state["waypoints"][0]
        else:
            home_point = {
                "lat": self.current_state["latitude"],
                "lon": self.current_state["longitude"],
            }
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
        self.is_returning_home = False
        self.rth_paused_until = 0
        self.current_state["control_mode"] = "MANUAL"
        self.current_state["status"] = "CONNECTED"
        stop_pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
        default_servo = self.config.get("actuators", {}).get("servo_default_angle", 90)
        self.serial_handler.send_command(f"S{stop_pwm};D{default_servo}\n")

    def convert_degree_to_actuators(self, degree, is_inverted):
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
        print("[AsvHandler] Dihentikan.")
