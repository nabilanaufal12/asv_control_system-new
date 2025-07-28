# gui/api_client.py
# --- MODIFIKASI: Membuat pembacaan serial lebih toleran terhadap data non-UTF8 ---

import time
import threading
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, Signal

class ApiClient(QObject):
    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)

    def __init__(self):
        super().__init__()
        
        self.serial_port = None
        self.is_connected = False
        self.running = True
        
        self.current_state = {
            "latitude": 0.0, "longitude": 0.0, "heading": 0.0,
            "speed": 0.0, "battery_voltage": 0.0, "status": "DISCONNECTED",
            "mission_time": "00:00:00", "control_mode": "MANUAL",
        }
        
        self._read_thread = threading.Thread(target=self._read_from_serial, daemon=True)
        self._read_thread.start()

    def _read_from_serial(self):
        while self.running:
            if not self.is_connected:
                if not self.running: break
                self.find_and_connect_esp32()
                time.sleep(2)
                continue

            try:
                if self.serial_port and self.serial_port.is_open:
                    # --- PERBAIKAN UTAMA: Tambahkan 'errors="ignore"' ---
                    # Ini akan mengabaikan karakter "sampah" dari log boot ESP32
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line and line.startswith("T:"):
                        self.parse_telemetry(line)
                        self.data_updated.emit(self.current_state)
                else:
                    if not self.running: break
                    self.is_connected = False
            except serial.SerialException:
                if not self.running: break
                print("Koneksi serial terputus. Mencoba menyambung kembali...")
                self.is_connected = False
                self.connection_status_changed.emit(False, "Koneksi ESP32 terputus")
                self.current_state["status"] = "DISCONNECTED"
                self.data_updated.emit(self.current_state)
            except Exception as e:
                if not self.running: break
                print(f"Error tak terduga di read thread: {e}")
                self.is_connected = False

    def find_and_connect_esp32(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Silicon Labs" in port.description or "CH340" in port.description or "CP210x" in port.description:
                try:
                    self.serial_port = serial.Serial(port.device, 115200, timeout=1)
                    self.is_connected = True
                    print(f"Berhasil terhubung ke ESP32 di {port.device}")
                    self.connection_status_changed.emit(True, f"Terhubung ke ESP32 di {port.device}")
                    self.current_state["status"] = "CONNECTED"
                    return
                except serial.SerialException as e:
                    print(f"Gagal membuka port {port.device}: {e}")
        self.connection_status_changed.emit(False, "ESP32 tidak ditemukan")

    def parse_telemetry(self, telemetry_string):
        try:
            parts = telemetry_string.strip('T:').split(';')
            for part in parts:
                data = part.split(',')
                if data[0] == "GPS": self.current_state['latitude'] = float(data[1]); self.current_state['longitude'] = float(data[2])
                elif data[0] == "BAT": self.current_state['battery_voltage'] = float(data[1])
                elif data[0] == "COMP": self.current_state['heading'] = int(data[1])
                elif data[0] == "SPD": self.current_state['speed'] = float(data[1])
        except (ValueError, IndexError) as e:
            print(f"Error parsing telemetri: {e} - Data: {telemetry_string}")

    def send_command(self, command, payload=None):
        if not self.is_connected: return
        serial_command = ""
        if command == "ACTUATOR_CONTROL":
            servo = payload.get('servo_angle')
            motor = payload.get('motor_pwm')
            serial_command = f"S{motor};D{servo}\n"
        elif command == "MANUAL_CONTROL":
            keys = set(payload)
            forward_input = 1 if 'W' in keys else -1 if 'S' in keys else 0
            turn_input = 1 if 'D' in keys else -1 if 'A' in keys else 0
            motor_pwm = 1500 + forward_input * 150
            servo_angle = 90 - turn_input * 45
            servo_angle = max(0, min(180, servo_angle))
            serial_command = f"S{motor_pwm};D{servo_angle}\n"
        if serial_command:
            try:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.write(serial_command.encode('utf-8'))
            except serial.SerialException:
                self.is_connected = False

    def shutdown(self):
        self.running = False
        if self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0) 
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
