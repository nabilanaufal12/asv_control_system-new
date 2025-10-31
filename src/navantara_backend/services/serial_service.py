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
        
        print("[Serial] Tidak ada ESP32 yang cocok. Mencoba /dev/ttyUSB0...")
        default_port_to_try = "/dev/ttyUSB0"
        port_exists = any(p.device == default_port_to_try for p in ports)
        
        if port_exists:
            if self.connect(default_port_to_try, baudrate):
                return True
        else:
            print(f"[Serial] Port default {default_port_to_try} tidak ditemukan.")

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
        if not self.is_connected or self.use_dummy_serial:
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