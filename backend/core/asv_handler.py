# backend/core/asv_handler.py
# --- VERSI MODIFIKASI: Logika pengambilan keputusan terpusat ---

import threading
import time
from backend.services.serial_service import SerialHandler
from backend.core.navigation import run_pure_pursuit_logic
from PySide6.QtCore import QObject, Signal, Slot
import numpy as np


class AsvHandler(QObject):
    """
    Kelas 'otak' ASV dengan logika kontrol terpusat,
    berjalan di dalam QThread.
    """

    telemetry_updated = Signal(dict)

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

        # ### PERUBAHAN 1: Ganti flag boolean dengan dictionary status ###
        # Ini akan menyimpan status terakhir yang dilaporkan oleh VisionService.
        self.vision_target = {"active": False, "degree": 90}

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
            self.telemetry_updated.emit(self.current_state.copy())

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
            self._update_and_emit_state()
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
                self._update_and_emit_state()

            # ### PERUBAHAN 2: Logika Pengambilan Keputusan Terpusat ###
            if self.current_state.get("control_mode") == "AUTO":
                if self.vision_target["active"]:
                    # Jika visi melaporkan target aktif, gunakan perhitungannya.
                    # 'is_inverted' akan diteruskan dari VisionService.
                    servo, pwm = self.convert_degree_to_actuators(
                        self.vision_target["degree"],
                        self.vision_target.get("is_inverted", False),
                    )
                else:
                    # Jika tidak, jalankan navigasi Pure Pursuit.
                    servo, pwm = run_pure_pursuit_logic(self.current_state, self.config)

                # Hanya ada satu titik pengiriman perintah dalam mode AUTO.
                self.serial_handler.send_command(f"S{pwm};D{servo}\n")

            time.sleep(0.2)
            self._update_and_emit_state()

    def process_command(self, command, payload):
        """Memproses perintah yang masuk dari GUI atau service lain."""
        if command == "CONFIGURE_SERIAL":
            port, baud = payload.get("serial_port"), payload.get("baud_rate")
            if port == "AUTO":
                self.serial_handler.find_and_connect_esp32(baud)
            else:
                self.serial_handler.connect(port, baud)
        elif command == "CHANGE_MODE":
            self.current_state["control_mode"] = payload
        elif (
            command == "MANUAL_CONTROL"
            and self.current_state.get("control_mode") == "MANUAL"
        ):
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
        elif command == "SET_WAYPOINTS":
            self.current_state["waypoints"] = payload
            self.current_state["current_waypoint_index"] = 0

        # ### PERUBAHAN 3: Ganti VISION_OVERRIDE dengan VISION_TARGET_UPDATE ###
        # Perintah ini tidak lagi mengirim perintah serial, hanya mengupdate status.
        elif command == "VISION_TARGET_UPDATE":
            self.vision_target["active"] = payload.get("active", False)
            self.vision_target["degree"] = payload.get("degree", 90)
            self.vision_target["is_inverted"] = payload.get("is_inverted", False)

        self._update_and_emit_state()

    # ### PERUBAHAN 4: Tambahkan fungsi konversi di sini ###
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
