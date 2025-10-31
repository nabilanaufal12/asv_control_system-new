# backend/serial_handler.py
# Modul ini bertanggung jawab untuk semua interaksi serial dengan mikrokontroler.

import serial
import serial.tools.list_ports
import threading
import time
import random
import json

class SerialHandler:
    def __init__(self, config):
        self.config = config
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.is_connected = False
        self.read_buffer = b""  # <-- [PERBAIKAN] Buffer internal untuk data

        # If True, when no real serial is connected we'll still simulate
        # handling of outgoing commands for development/debugging.
        self.simulate_if_disconnected = self.config.get("serial_connection", {}).get(
            "simulate_if_disconnected", True
        )

        self.use_dummy_serial = self.config.get("serial_connection", {}).get("use_dummy_serial", False)
        if self.use_dummy_serial:
            self.is_connected = True
            self.dummy_heading = 90.0
            self.dummy_lat = -6.918000
            self.dummy_lon = 107.618500
            self.dummy_data_counter = 0
            self.dummy_servo = 90
            self.dummy_motor = 1500
            print("=================================================")
            print("  [PERINGATAN] MODE SERIAL DUMMY AKTIF  ")
            print("  Backend akan menghasilkan data serial JSON palsu.  ")
            print("=================================================")

    def connect(self, port_name, baudrate):
        self.disconnect()
        time.sleep(0.1)
        try:
            print(f"[Serial] Mencoba terhubung ke {port_name} @ {baudrate}...")
            new_port = serial.Serial(port_name, int(baudrate), timeout=1)
            
            # --- [INI SUDAH BENAR] ---
            new_port.reset_input_buffer()
            print(f"[Serial] Koneksi {port_name} berhasil. Menunggu ESP32 boot (1.5 d)...")
            time.sleep(1.5) 
            new_port.reset_input_buffer() 
            print(f"[Serial] ESP32 siap. Buffer bersih.")
            # --- [AKHIR] ---

            with self.serial_lock:
                self.serial_port = new_port
            self.is_connected = True
            self.read_buffer = b"" # <-- [PERBAIKAN] Pastikan buffer bersih saat konek
            print(f"[Serial] Berhasil terhubung dan sinkron dengan {port_name}")
            return True
        except serial.SerialException as e:
            print(f"[Serial] Gagal membuka {port_name}: {e}")
            self.is_connected = False
            return False

    def find_and_connect_esp32(self, baudrate):
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
                    print(f"[Serial] Menemukan ESP32 (cocok: '{desc}') di {port.device}")
                    if self.connect(port.device, baudrate):
                        return True

        # If no descriptor matched, try common fallbacks depending on OS
        # First try Linux default
        print("[Serial] Tidak ada ESP32 yang cocok dari descriptor. Mencoba /dev/ttyUSB0 sebagai fallback...")
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
        self.read_buffer = b"" # <-- [PERBAIKAN] Bersihkan buffer saat disconnect

    def send_command(self, command_string):
        # Always log outgoing commands for visibility/debugging
        try:
            print(f"[Serial SEND] {command_string.strip()}")
        except Exception:
            # If printing fails for any reason, ignore but continue
            pass

        # If we're using dummy serial, simulate that the command was applied
        if self.use_dummy_serial:
            # Try to parse common actuator command format 'A,servo,motor' to update dummy state
            try:
                parts = command_string.strip().split(',')
                if parts and parts[0] == 'A' and len(parts) >= 3:
                    servo_val = int(float(parts[1]))
                    motor_val = int(float(parts[2]))
                    with self.serial_lock:
                        self.dummy_servo = servo_val
                        self.dummy_motor = motor_val
            except Exception:
                pass
            return

        # If not connected, optionally simulate the command (useful for testing AI without hardware)
        if not self.is_connected:
            if self.simulate_if_disconnected:
                try:
                    print(f"[Serial SIMULATE] (no connection) {command_string.strip()}")
                except Exception:
                    pass
                # Try to apply actuator updates to in-memory dummy state so AI logic can reflect them
                try:
                    parts = command_string.strip().split(',')
                    if parts and parts[0] == 'A' and len(parts) >= 3:
                        servo_val = int(float(parts[1]))
                        motor_val = int(float(parts[2]))
                        # create dummy attrs if they don't exist
                        if not hasattr(self, 'dummy_servo'):
                            self.dummy_servo = servo_val
                        else:
                            self.dummy_servo = servo_val
                        if not hasattr(self, 'dummy_motor'):
                            self.dummy_motor = motor_val
                        else:
                            self.dummy_motor = motor_val
                except Exception:
                    pass
                return
            else:
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
        # --- [LOGIKA DUMMY (Tidak Berubah)] ---
        if self.use_dummy_serial:
            with self.serial_lock: 
                time.sleep(0.5)
                self.dummy_heading = (self.dummy_heading + 1.5) % 360
                self.dummy_lat += 0.00001 * random.uniform(0.8, 1.2)
                self.dummy_lon += 0.000005 * random.uniform(0.8, 1.2)
                self.dummy_data_counter += 1
                dummy_data = {
                    "heading": round(self.dummy_heading, 1),
                    "lat": round(self.dummy_lat, 6),
                    "lon": round(self.dummy_lon, 6),
                    "speed_kmh": round(1.2 + random.uniform(-0.1, 0.1), 1),
                    "sats": 10
                }
                if self.dummy_data_counter % 2 == 0:
                    self.dummy_servo = 110 if self.dummy_servo == 90 else 90
                    self.dummy_motor = 1550 if self.dummy_motor == 1500 else 1500
                    dummy_data.update({
                        "mode": "MANUAL", "status": "ACTIVE",
                        "servo_out": self.dummy_servo, "motor_out": self.dummy_motor
                    })
                else:
                    dist_to_wp = 15.0 - (self.dummy_data_counter % 10)
                    target_bearing = 120.0
                    heading_error = target_bearing - self.dummy_heading
                    if heading_error > 180: heading_error -= 360
                    if heading_error < -180: heading_error += 360
                    dummy_data.update({
                        "mode": "AUTO", "status": "WAYPOINT",
                        "servo_out": 100, "motor_out": 1600,
                        "wp_target_idx": 2, "wp_dist_m": round(dist_to_wp, 1),
                        "wp_target_brg": round(target_bearing, 1),
                        "wp_error_hdg": round(heading_error, 1)
                    })
                return json.dumps(dummy_data)
        # --- AKHIR LOGIKA DUMMY ---

        # --- [MODIFIKASI BESAR] Logika Asli dengan Buffering ---
        if not self.is_connected:
            return None

        with self.serial_lock:
            if self.serial_port:
                try:
                    # 1. Baca SEMUA data yang tersedia di buffer serial OS
                    if self.serial_port.in_waiting > 0:
                        data_in = self.serial_port.read(self.serial_port.in_waiting)
                        self.read_buffer += data_in
                    
                    # 2. Cari pemisah baris (newline) di buffer internal kita
                    newline_pos = self.read_buffer.find(b'\n')
                    
                    if newline_pos != -1:
                        # 3. Jika newline ditemukan, kita punya satu baris lengkap
                        # Ambil baris lengkap (sebelum newline)
                        line = self.read_buffer[:newline_pos]
                        
                        # Simpan sisa data (setelah newline) untuk panggilan berikutnya
                        self.read_buffer = self.read_buffer[newline_pos + 1:]
                        
                        # 4. Kembalikan baris yang sudah bersih (decode dan strip)
                        return line.decode("utf-8", errors="ignore").strip()
                    else:
                        # 5. Jika tidak ada newline, kita masih menunggu data
                        # Kembalikan None agar tidak memblokir
                        return None 

                except serial.SerialException as e:
                    print(f"[Serial] Error membaca data: {e}. Memutuskan koneksi.")
                    self.disconnect()
                except Exception as e:
                    print(f"[Serial] Error tidak terduga saat membaca: {e}")
                    self.disconnect()
        return None