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
        self.read_buffer = b""  # <-- [PERBAIKAN] Buffer internal untuk data

    def connect(self, port_name, baudrate):
        self.disconnect()
        time.sleep(0.1)
        try:
            print(f"[Serial] Mencoba terhubung ke {port_name} @ {baudrate}...")
            new_port = serial.Serial(port_name, int(baudrate), timeout=1)

            # --- [INI SUDAH BENAR] ---
            new_port.reset_input_buffer()
            print(
                f"[Serial] Koneksi {port_name} berhasil. Menunggu ESP32 boot (1.5 d)..."
            )
            time.sleep(1.5)
            new_port.reset_input_buffer()
            print("[Serial] ESP32 siap. Buffer bersih.")
            # --- [AKHIR] ---

            with self.serial_lock:
                self.serial_port = new_port
            self.is_connected = True
            self.read_buffer = b""  # <-- [PERBAIKAN] Pastikan buffer bersih saat konek
            print(f"[Serial] Berhasil terhubung dan sinkron dengan {port_name}")
            return True
        except serial.SerialException as e:
            print(f"[Serial] Gagal membuka {port_name}: {e}")
            self.is_connected = False
            return False

    def find_and_connect_esp32(self, baudrate):
        # --- [MODIFIKASI: PRIORITAS UDEV RULE] ---
        # 1. Cek device permanen /dev/ttyASV terlebih dahulu
        permanent_port = "/dev/ttyASV"
        try:
            import os

            # Cek apakah file device ada di sistem Linux
            if os.path.exists(permanent_port):
                print(f"[Serial] Mendeteksi device permanen: {permanent_port}")
                # Coba connect langsung
                if self.connect(permanent_port, baudrate):
                    return True
            else:
                print(f"[Serial] Device permanen {permanent_port} belum terpasang.")
        except Exception as e:
            print(f"[Serial] Gagal mengecek {permanent_port}: {e}")
        # -----------------------------------------
        # List semua port yang tersedia untuk diagnostik
        ports = serial.tools.list_ports.comports()
        print("[Serial] Port yang tersedia:")
        for port in ports:
            print(f"  - {port.device}: {port.description} ({port.hwid})")

        # If config provides a forced port, try it first
        forced = self.config.get("serial_connection", {}).get("force_serial_port")
        if forced:
            print(f"[Serial] Mencoba port paksa dari config: {forced}")
            if self.connect(forced, baudrate):
                return True

        ports = serial.tools.list_ports.comports()
        descriptors = self.config.get("serial_connection", {}).get(
            "auto_connect_descriptors", []
        )
        for port in ports:
            port_info = f"{port.description} {port.hwid}".lower()
            for desc in descriptors:
                if desc.lower() in port_info:
                    print(
                        f"[Serial] Menemukan ESP32 (cocok: '{desc}') di {port.device}"
                    )
                    if self.connect(port.device, baudrate):
                        return True

        # If no descriptor matched, try common fallbacks depending on OS
        # First try Linux default
        print(
            "[Serial] Tidak ada ESP32 yang cocok dari descriptor. Mencoba /dev/ttyUSB0 sebagai fallback..."
        )
        default_port_to_try = "/dev/ttyUSB0"
        port_exists = any(p.device == default_port_to_try for p in ports)

        if port_exists:
            if self.connect(default_port_to_try, baudrate):
                return True
        else:
            print(f"[Serial] Port default {default_port_to_try} tidak ditemukan.")

        # If running on Windows, try common COM ports (COM3..COM20)
        try:
            import sys

            if sys.platform.startswith("win"):
                print("[Serial] Mencoba fallback COM ports (Windows)...")
                for i in range(3, 21):
                    com_name = f"COM{i}"
                    try:
                        if self.connect(com_name, baudrate):
                            return True
                    except Exception:
                        # connect() already logs errors; continue trying
                        continue
        except Exception:
            pass

        print("[Serial] Gagal terhubung ke ESP32 secara otomatis.")
        return False

    def disconnect(self):
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                print("[Serial] Port ditutup.")
        self.is_connected = False
        self.read_buffer = b""  # <-- [PERBAIKAN] Bersihkan buffer saat disconnect

    def send_command(self, command_string):
        # Always log outgoing commands for visibility/debugging
        try:
            if command_string.strip() != "W":
                print(f"[Serial SEND] {command_string.strip()}")
        except Exception:
            pass

        if not self.is_connected:
            print("[Serial] Tidak terhubung, perintah tidak dikirim.")
            return

        with self.serial_lock:
            if self.serial_port:
                try:
                    self.serial_port.write(command_string.encode("utf-8"))
                except Exception as e:
                    print(f"[Serial] Gagal mengirim data: {e}")
                    self.disconnect()

    def read_line(self):

        # --- [MODIFIKASI BESAR] Logika Asli dengan Buffering ---
        if not self.is_connected:
            return None

        with self.serial_lock:
            if self.serial_port:
                try:
                    # 1. Cek apakah ada data di buffer OS
                    # Menggunakan in_waiting adalah kunci non-blocking
                    waiting = self.serial_port.in_waiting
                    if waiting > 0:
                        # Baca semua yang ada, jangan baca per-byte
                        data_in = self.serial_port.read(waiting)
                        self.read_buffer += data_in

                    # 2. Cari newline
                    newline_pos = self.read_buffer.find(b"\n")

                    if newline_pos != -1:
                        # Ambil baris lengkap
                        line = self.read_buffer[:newline_pos]
                        # Buang data yang sudah diambil dari buffer
                        self.read_buffer = self.read_buffer[newline_pos + 1 :]

                        # Decode dan return
                        return line.decode("utf-8", errors="ignore").strip()

                    # Jika buffer internal terlalu besar (misal sampah), bersihkan
                    if len(self.read_buffer) > 4096:
                        self.read_buffer = b""

                    return None

                except serial.SerialException as e:
                    print(f"[Serial] Error membaca data: {e}")
                    self.disconnect()
                except Exception as e:
                    print(f"[Serial] Error tak terduga: {e}")
                    self.disconnect()
        return None
