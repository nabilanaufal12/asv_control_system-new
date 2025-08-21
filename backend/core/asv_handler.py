# backend/core/asv_handler.py
# --- VERSI MODIFIKASI: Menggunakan Qt Signals, bukan WebSockets ---

import threading
import time
from backend.services.serial_service import SerialHandler
from backend.core.navigation import run_pure_pursuit_logic
from PySide6.QtCore import QObject, Signal

class AsvHandler(QObject):
    """
    Kelas 'otak' ASV yang sekarang berkomunikasi secara internal
    menggunakan Qt Signals dalam satu aplikasi GUI.
    """
    # Definisikan sinyal untuk mengirim pembaruan state ke GUI
    telemetry_updated = Signal(dict)

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.serial_handler = SerialHandler(config)
        self.running = True
        self.current_state = {
            "control_mode": "MANUAL", "latitude": -6.9180, "longitude": 107.6185,
            "heading": 90.0, "cog": 0.0, "speed": 0.0, "battery_voltage": 12.5,
            "status": "DISCONNECTED", "mission_time": "00:00:00", "waypoints": [],
            "current_waypoint_index": 0, "is_connected_to_serial": False,
        }
        self.vision_override_active = False
        print("[AsvHandler] Handler diinisialisasi.")

    def start_threads(self):
        """Memulai thread-thread yang berjalan di latar belakang."""
        self._read_thread = threading.Thread(target=self._read_from_serial_loop, daemon=True)
        self._logic_thread = threading.Thread(target=self._main_logic_loop, daemon=True)
        self._read_thread.start()
        self._logic_thread.start()
        print("[AsvHandler] Thread-thread dimulai.")

    def _update_and_emit_state(self):
        """Fungsi helper untuk mengirim state terbaru melalui sinyal Qt."""
        # Ganti socketio.emit dengan sinyal Qt
        self.telemetry_updated.emit(self.current_state.copy())

    def _read_from_serial_loop(self):
        """Loop untuk membaca data telemetri dari hardware secara terus-menerus."""
        while self.running:
            line = self.serial_handler.read_line()
            if line and line.startswith("T:"):
                self._parse_telemetry(line)
            else:
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
            self._update_and_emit_state()
        except (ValueError, IndexError):
            pass

    def _main_logic_loop(self):
        """Loop logika utama yang berjalan di background."""
        start_time = time.time()
        while self.running:
            elapsed = time.time() - start_time
            self.current_state["mission_time"] = time.strftime('%H:%M:%S', time.gmtime(elapsed))
            
            new_connection_status = self.serial_handler.is_connected
            if self.current_state["is_connected_to_serial"] != new_connection_status:
                self.current_state["is_connected_to_serial"] = new_connection_status
                self.current_state["status"] = "CONNECTED" if new_connection_status else "DISCONNECTED"
                self._update_and_emit_state()

            if self.current_state.get("control_mode") == "AUTO" and not self.vision_override_active:
                servo, pwm = run_pure_pursuit_logic(self.current_state, self.config)
                self.serial_handler.send_command(f"S{pwm};D{servo}\n")
            
            time.sleep(0.2)
            # Panggil emit juga di sini agar mission_time selalu update di GUI
            self._update_and_emit_state()

    def process_command(self, command, payload):
        """Memproses perintah yang masuk dari GUI."""
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
        
        # Selalu kirim state terbaru ke GUI setelah memproses perintah
        self._update_and_emit_state()

    def get_current_state(self):
        """Mengembalikan salinan state saat ini dengan aman."""
        return self.current_state.copy()
        
    def stop(self):
        """Menghentikan semua loop dan menutup koneksi serial."""
        self.running = False
        self.serial_handler.disconnect()
        print("[AsvHandler] Dihentikan.")