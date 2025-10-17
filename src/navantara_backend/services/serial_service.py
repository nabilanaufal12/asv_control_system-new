# backend/serial_handler.py
# Modul ini bertanggung jawab untuk semua interaksi serial dengan mikrokontroler.

import serial
import serial.tools.list_ports
import threading
import time


class SerialHandler:
    def __init__(self, config):
        self.config = config
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.is_connected = False

    def connect(self, port_name, baudrate):
        self.disconnect()
        time.sleep(0.1)
        try:
            print(f"[Serial] Mencoba terhubung ke {port_name} @ {baudrate}...")
            new_port = serial.Serial(port_name, int(baudrate), timeout=1)
            with self.serial_lock:
                self.serial_port = new_port
            self.is_connected = True
            print(f"[Serial] Berhasil terhubung ke {port_name}")
            return True
        except serial.SerialException as e:
            print(f"[Serial] Gagal membuka {port_name}: {e}")
            self.is_connected = False
            return False

    def find_and_connect_esp32(self, baudrate):
        ports = serial.tools.list_ports.comports()
        descriptors = self.config.get("serial_connection", {}).get(
            "auto_connect_descriptors", []
        )
        for port in ports:
            for desc in descriptors:
                if desc in port.description:
                    return self.connect(port.device, baudrate)
        # --- PERBAIKAN DI SINI ---
        print("[Serial] ESP32 tidak ditemukan")  # Menghapus f'' yang tidak perlu
        # -------------------------
        return False

    def disconnect(self):
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                print("[Serial] Port ditutup.")
        self.is_connected = False

    def send_command(self, command_string):
        with self.serial_lock:
            if self.is_connected and self.serial_port:
                try:
                    self.serial_port.write(command_string.encode("utf-8"))
                except Exception as e:
                    print(f"[Serial] Gagal mengirim data: {e}")
                    self.disconnect()

    def read_line(self):
        with self.serial_lock:
            if self.is_connected and self.serial_port:
                try:
                    return (
                        self.serial_port.readline()
                        .decode("utf-8", errors="ignore")
                        .strip()
                    )
                except Exception:
                    self.disconnect()
        return None
