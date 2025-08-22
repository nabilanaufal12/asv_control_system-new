# backend/core/asv_handler.py
# --- VERSI FINAL: Dengan State Machine Misi Sekuensial ---

import threading
import time
from backend.services.serial_service import SerialHandler
from backend.core.navigation import run_pure_pursuit_logic
from PySide6.QtCore import QObject, Signal, Slot
import numpy as np


class AsvHandler(QObject):
    """
    Kelas 'otak' ASV dengan logika kontrol terpusat dan state machine misi,
    berjalan di dalam QThread.
    """

    telemetry_updated = Signal(dict)
    # Sinyal baru untuk memberi tahu VisionService tentang target yang relevan
    mission_target_changed = Signal(str) 

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.serial_handler = SerialHandler(config)
        self.running = True
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
        }
        self.vision_target = {"active": False, "degree": 90}
        self.is_returning_home = False
        self.rth_paused_until = 0

        # --- LANGKAH 1 (State Machine): Tambahkan State Misi ---
        self.mission_phase = "IDLE" # Status awal misi
        # Tentukan waypoint index di mana transisi fase terjadi
        self.phase_transition_wp_index = {
            "NAVIGATING_GATES": 7, # Setelah waypoint ke-7, mulai cari kotak
        }


        print("[AsvHandler] Handler diinisialisasi.")

    @Slot()
    def run(self):
        """Metode ini akan dijalankan oleh QThread dari MainWindow."""
        self._read_thread = threading.Thread(
            target=self._read_from_serial_loop, daemon=True
        )
        self._logic_thread = threading.Thread(target=self._main_logic_loop, daemon=True)
        self._read_thread.start()
        self._logic_thread.start()
        print("[AsvHandler] Thread-thread internal dimulai.")

    def _update_and_emit_state(self):
        """Fungsi helper untuk mengirim state terbaru melalui sinyal Qt."""
        if self.running:
            # Tambahkan fase misi ke data telemetri agar bisa ditampilkan di GUI jika perlu
            state_copy = self.current_state.copy()
            state_copy["mission_phase"] = self.mission_phase
            self.telemetry_updated.emit(state_copy)

    # ... (_read_from_serial_loop dan _parse_telemetry tidak berubah) ...
    def _read_from_serial_loop(self):
        """Loop untuk membaca data telemetri dari hardware secara terus-menerus."""
        while self.running:
            line = self.serial_handler.read_line()
            if line and line.startswith("T:"):
                self._parse_telemetry(line)
            else:
                time.sleep(0.1)

    def _parse_telemetry(self, line):
        """Mem-parsing string telemetri dan memperbarui state."""
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
            # Jangan panggil _update_and_emit_state() di sini agar tidak berlebihan
        except (ValueError, IndexError):
            pass

    def _main_logic_loop(self):
        """Loop logika utama yang menjadi satu-satunya pengambil keputusan."""
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

            # --- LANGKAH 4 (State Machine): Cek Transisi Fase Misi ---
            self._check_mission_phase_transition()

            if self.current_state.get("control_mode") == "AUTO":
                servo, pwm = 0, 0
                
                # Logika RTH tetap menjadi prioritas utama
                if self.is_returning_home and self.vision_target["active"] and time.time() > self.rth_paused_until:
                    print("[AsvHandler] Rintangan terdeteksi saat RTH! Berhenti sejenak...")
                    servo = self.config.get("actuators", {}).get("servo_default_angle", 90)
                    pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
                    self.rth_paused_until = time.time() + 5
                
                elif time.time() < self.rth_paused_until:
                    servo = self.config.get("actuators", {}).get("servo_default_angle", 90)
                    pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
                
                # Logika visi hanya aktif jika target dari visi relevan dengan fase misi saat ini
                elif self.vision_target["active"]:
                    servo, pwm = self.convert_degree_to_actuators(
                        self.vision_target["degree"],
                        self.vision_target.get("is_inverted", False),
                    )
                
                else: # Jika tidak ada intervensi visi, navigasi normal
                    servo, pwm = run_pure_pursuit_logic(self.current_state, self.config)
                    if self.is_returning_home and self.current_state["current_waypoint_index"] >= len(self.current_state["waypoints"]):
                        print("[AsvHandler] Titik Home tercapai. Fitur RTH selesai.")
                        self._stop_rth()

                self.serial_handler.send_command(f"S{pwm};D{servo}\n")

            time.sleep(0.2)
            self._update_and_emit_state()

    # --- LANGKAH 5 (State Machine): Buat fungsi untuk transisi fase ---
    def _check_mission_phase_transition(self):
        """Memeriksa apakah kondisi untuk berpindah ke fase misi berikutnya terpenuhi."""
        current_wp_index = self.current_state["current_waypoint_index"]

        if self.mission_phase == "NAVIGATING_GATES":
            transition_index = self.phase_transition_wp_index["NAVIGATING_GATES"]
            if current_wp_index >= transition_index:
                self._set_mission_phase("SEARCHING_GREEN_BOX")
        
        # Transisi setelah semua waypoint selesai
        if self.mission_phase != "IDLE" and not self.is_returning_home and current_wp_index >= len(self.current_state["waypoints"]):
             self._set_mission_phase("MISSION_COMPLETE")


    def _set_mission_phase(self, new_phase):
        """Mengubah fase misi dan mengirimkan sinyal ke VisionService."""
        if self.mission_phase != new_phase:
            self.mission_phase = new_phase
            print(f"[AsvHandler] Fase misi diubah ke: {self.mission_phase}")
            self.current_state["status"] = self.mission_phase # Update status utama
            
            # Beri tahu VisionService target apa yang harus dicari
            if new_phase == "NAVIGATING_GATES":
                self.mission_target_changed.emit("BUOYS")
            elif new_phase == "SEARCHING_GREEN_BOX":
                self.mission_target_changed.emit("GREEN_BOX")
            elif new_phase == "SEARCHING_BLUE_BOX":
                self.mission_target_changed.emit("BLUE_BOX")
            elif new_phase == "MISSION_COMPLETE":
                print("[AsvHandler] Misi Selesai!")
                self.current_state["control_mode"] = "MANUAL"
            else:
                 self.mission_target_changed.emit("NONE") # Jangan cari apa-apa


    def process_command(self, command, payload):
        command_handlers = {
            "CONFIGURE_SERIAL": self._handle_serial_configuration,
            "CHANGE_MODE": self._handle_mode_change,
            "MANUAL_CONTROL": self._handle_manual_control,
            "SET_WAYPOINTS": self._handle_set_waypoints,
            "VISION_TARGET_UPDATE": self._handle_vision_target_update,
            "INITIATE_RTH": self._handle_initiate_rth,
            # --- LANGKAH 2 (State Machine): Tambahkan handler baru ---
            "START_MISSION": self._handle_start_mission,
            "PHOTOGRAPHY_COMPLETE": self._handle_photography_complete
        }

        handler = command_handlers.get(command)
        if handler:
            handler(payload)
            self._update_and_emit_state()
        else:
            print(f"[AsvHandler] Peringatan: Perintah tidak dikenal '{command}'")

    # --- LANGKAH 3 (State Machine): Buat handler baru ---
    def _handle_start_mission(self, payload):
        """Memulai misi dari fase pertama."""
        if not self.current_state["waypoints"]:
            print("[AsvHandler] Tidak bisa memulai misi: Tidak ada waypoint yang dimuat.")
            return
        print("[AsvHandler] Misi Dimulai!")
        self.current_state["control_mode"] = "AUTO"
        self._set_mission_phase("NAVIGATING_GATES")

    def _handle_photography_complete(self, payload):
        """Dipanggil oleh VisionService setelah selesai memotret."""
        object_type = payload.get("object")
        print(f"[AsvHandler] Menerima konfirmasi fotografi untuk: {object_type}")
        if object_type == "green_box":
            self._set_mission_phase("SEARCHING_BLUE_BOX")
        elif object_type == "blue_box":
            self._set_mission_phase("RACING_TO_FINISH")
    
    # ... (Handler lain tidak berubah) ...
    def _handle_serial_configuration(self, payload):
        port, baud = payload.get("serial_port"), payload.get("baud_rate")
        if port == "AUTO":
            self.serial_handler.find_and_connect_esp32(baud)
        else:
            self.serial_handler.connect(port, baud)

    def _handle_mode_change(self, payload):
        # Mencegah mode diubah secara manual saat misi/RTH aktif
        if self.mission_phase not in ["IDLE", "MISSION_COMPLETE"] and not self.is_returning_home:
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
        self.mission_phase = "IDLE" # Reset fase misi setiap kali waypoint baru di-load

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
                "lon": self.current_state["longitude"]
            }
        rth_waypoints = [home_point]
        self.current_state["waypoints"] = rth_waypoints
        self.current_state["current_waypoint_index"] = 0
        self.current_state["control_mode"] = "AUTO"
        self.current_state["status"] = "RETURNING TO HOME"
        self.is_returning_home = True
        self.mission_phase = "IDLE" # Nonaktifkan state machine misi saat RTH
        print(f"[AsvHandler] RTH diaktifkan. Target: {home_point}")

    def _stop_rth(self):
        """Membersihkan state RTH dan kembali ke mode manual."""
        self.is_returning_home = False
        self.rth_paused_until = 0
        self.current_state["control_mode"] = "MANUAL"
        self.current_state["status"] = "CONNECTED"
        stop_pwm = self.config.get("actuators", {}).get("motor_pwm_stop", 1500)
        default_servo = self.config.get("actuators", {}).get("servo_default_angle", 90)
        self.serial_handler.send_command(f"S{stop_pwm};D{default_servo}\n")

    def convert_degree_to_actuators(self, degree, is_inverted):
        """Menghitung nilai servo dan PWM dari sudut target."""
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
        """Mengembalikan salinan state saat ini dengan aman."""
        return self.current_state.copy()

    def stop(self):
        """Menghentikan semua loop dan menutup koneksi serial."""
        self.running = False
        self.serial_handler.disconnect()
        print("[AsvHandler] Dihentikan.")