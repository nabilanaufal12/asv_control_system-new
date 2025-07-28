# gui/api_client.py
# --- Versi Stabil: Kurir serial dengan koneksi otomatis ---

import time
import threading
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, Signal, Slot

class ApiClient(QObject):
    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_connected = False
        self.running = True
        self.current_state = { "status": "DISCONNECTED" }
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
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line and line.startswith("T:"):
                        # (Logika parsing telemetri bisa ditambahkan kembali di sini jika perlu)
                        pass 
            except serial.SerialException:
                if not self.running: break
                self.is_connected = False
                self.connection_status_changed.emit(False, "Koneksi ESP32 terputus")

    def find_and_connect_esp32(self):
        ports = [p for p in serial.tools.list_ports.comports() if "Silicon Labs" in p.description or "CH340" in p.description or "CP210x" in p.description]
        if ports:
            try:
                self.serial_port = serial.Serial(ports[0].device, 115200, timeout=1)
                self.is_connected = True
                self.connection_status_changed.emit(True, f"Terhubung ke {ports[0].device}")
            except serial.SerialException as e:
                self.connection_status_changed.emit(False, f"Gagal membuka {ports[0].device}")
        else:
            self.connection_status_changed.emit(False, "ESP32 tidak ditemukan")

    @Slot(str)
    def send_serial_command(self, command_string):
        """Mengirim string perintah mentah ke ESP32."""
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(command_string.encode('utf-8'))
                print(f"Mengirim ke ESP32: {command_string.strip()}")
            except serial.SerialException:
                self.is_connected = False
    
    def send_command(self, command, payload=None):
        """Menangani perintah yang tidak terkait langsung dengan aktuator (misal: mode manual)."""
        if command == "MANUAL_CONTROL":
            keys = set(payload)
            forward_input = 1 if 'W' in keys else -1 if 'S' in keys else 0
            turn_input = 1 if 'D' in keys else -1 if 'A' in keys else 0
            motor_pwm = 1500 + forward_input * 150
            servo_angle = 90 - turn_input * 45
            servo_angle = max(0, min(180, servo_angle))
            serial_command = f"S{motor_pwm};D{servo_angle}\n"
            self.send_serial_command(serial_command)

    def shutdown(self):
        self.running = False
        if self._read_thread.is_alive(): self._read_thread.join(timeout=1.0) 
        if self.serial_port and self.serial_port.is_open: self.serial_port.close()
