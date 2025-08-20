# backend/core/asv_handler.py
# Kelas "otak" yang mengelola state, logika utama, dan koordinasi.

import threading
import time
from backend.services.serial_service import SerialHandler
from backend.core.navigation import run_pure_pursuit_logic

class AsvHandler:
    def __init__(self, config):
        self.config = config
        self.serial_handler = SerialHandler(config) # Menggunakan spesialis serial
        self.running = True
        self.current_state = {
            "control_mode": "MANUAL", "latitude": -6.9180, "longitude": 107.6185,
            "heading": 90.0, "cog": 0.0, "speed": 0.0, "battery_voltage": 12.5,
            "status": "DISCONNECTED", "mission_time": "00:00:00", "waypoints": [],
            "current_waypoint_index": 0, "is_connected_to_serial": False,
        }
        self.vision_override_active = False
        
        self._read_thread = threading.Thread(target=self._read_from_serial_loop, daemon=True)
        self._logic_thread = threading.Thread(target=self._main_logic_loop, daemon=True)
        self._read_thread.start()
        self._logic_thread.start()
        print("[AsvHandler] Handler dimulai.")

    def _read_from_serial_loop(self):
        """Loop untuk membaca data telemetri dari hardware secara terus-menerus."""
        while self.running:
            line = self.serial_handler.read_line()
            if line and line.startswith("T:"):
                self._parse_telemetry(line)
            else:
                # Jika tidak ada data atau koneksi terputus, tunggu sebentar
                time.sleep(0.5)

    def _parse_telemetry(self, line):
        """Mem-parsing string telemetri dan memperbarui state."""
        try:
            parts = line.strip('T:').split(';')
            for part in parts:
                data = part.split(',')
                if data[0] == "GPS":
                    self.current_state['latitude'] = float(data[1])
                    self.current_state['longitude'] = float(data[2])
                elif data[0] == "COMP": self.current_state['heading'] = float(data[1])
                elif data[0] == "SPD": self.current_state['speed'] = float(data[1])
                elif data[0] == "BAT": self.current_state['battery_voltage'] = float(data[1])
        except (ValueError, IndexError):
            pass # Abaikan telemetri yang formatnya rusak agar program tidak crash

    def _main_logic_loop(self):
        """Loop logika utama yang berjalan di background."""
        start_time = time.time()
        while self.running:
            elapsed = time.time() - start_time
            self.current_state["mission_time"] = time.strftime('%H:%M:%S', time.gmtime(elapsed))
            self.current_state["is_connected_to_serial"] = self.serial_handler.is_connected
            self.current_state["status"] = "CONNECTED" if self.serial_handler.is_connected else "DISCONNECTED"

            # Jalankan navigasi otomatis jika mode AUTO dan tidak di-override oleh visi
            if self.current_state.get("control_mode") == "AUTO" and not self.vision_override_active:
                servo, pwm = run_pure_pursuit_logic(self.current_state, self.config)
                self.serial_handler.send_command(f"S{pwm};D{servo}\n")
            
            time.sleep(0.2) # Frekuensi loop logika

    def process_command(self, command, payload):
        """Memproses perintah yang masuk dari API endpoint."""
        if command == "CONFIGURE_SERIAL":
            port, baud = payload.get("serial_port"), payload.get("baud_rate")
            if port == "AUTO": self.serial_handler.find_and_connect_esp32(baud)
            else: self.serial_handler.connect(port, baud)
        elif command == "CHANGE_MODE":
            self.current_state["control_mode"] = payload
        elif command == "MANUAL_CONTROL" and self.current_state.get("control_mode") == "MANUAL":
            keys = set(payload)
            actuator_config = self.config.get("actuators", {})
            pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
            pwr = actuator_config.get("motor_pwm_manual_power", 150)
            servo_def = actuator_config.get("servo_default_angle", 90)
            servo_min = actuator_config.get("servo_min_angle", 45)
            
            fwd = 1 if 'W' in keys else -1 if 'S' in keys else 0
            turn = 1 if 'D' in keys else -1 if 'A' in keys else 0
            
            pwm = pwm_stop + fwd * pwr
            servo = max(0, min(180, int(servo_def - turn * (servo_def - servo_min))))
            self.serial_handler.send_command(f"S{pwm};D{servo}\n")
        elif command == "SET_WAYPOINTS":
            self.current_state["waypoints"] = payload
            self.current_state["current_waypoint_index"] = 0
        elif command == "VISION_OVERRIDE":
            self.vision_override_active = (payload.get('status') == 'ACTIVE')
            if self.vision_override_active and payload.get('command'):
                self.serial_handler.send_command(payload.get('command'))

    def get_current_state(self):
        """Mengembalikan salinan state saat ini dengan aman."""
        return self.current_state.copy()