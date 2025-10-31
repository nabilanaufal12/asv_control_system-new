# backend/serial_handler.py
# Modul ini bertanggung jawab untuk semua interaksi serial dengan mikrokontroler.

import serial
import serial.tools.list_ports
import threading
import time
import random

class SerialHandler:
    def __init__(self, config):
        self.config = config
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.is_connected = False

        self.use_dummy_serial = self.config.get("serial_connection", {}).get("use_dummy_serial", False)
        if self.use_dummy_serial:
            self.is_connected = True  # Langsung set 'connected'
            self.dummy_heading = 90.0
            self.dummy_lat = -6.918000
            self.dummy_lon = 107.618500
            self.dummy_data_counter = 0
            self.dummy_servo = 90
            self.dummy_motor = 1500
            print("=================================================")
            print("  [PERINGATAN] MODE SERIAL DUMMY AKTIF  ")
            print("  Backend akan menghasilkan data serial palsu.  ")
            print("=================================================")

    def connect(self, port_name, baudrate):
        self.disconnect()
        time.sleep(0.1) # Beri jeda singkat sebelum mencoba koneksi baru
        try:
            print(f"[Serial] Mencoba terhubung ke {port_name} @ {baudrate}...")
            new_port = serial.Serial(port_name, int(baudrate), timeout=1)

            # --- [SOLUSI] TAMBAHKAN BLOK INI ---
            # 1. Membersihkan semua data sampah sisa bootloader dari buffer input.
            new_port.reset_input_buffer()
            # 2. Beri waktu 1.5 detik agar ESP32 selesai booting sepenuhnya
            #    sebelum kita mulai membaca datanya.
            print(f"[Serial] Koneksi {port_name} berhasil. Menunggu ESP32 boot (1.5 d)...")
            time.sleep(1.5) 
            # 3. Bersihkan buffer sekali lagi untuk jaga-jaga jika ada sisa data boot.
            new_port.reset_input_buffer() 
            print(f"[Serial] ESP32 siap. Buffer bersih.")
            # --- [AKHIR SOLUSI] ---

            with self.serial_lock:
                self.serial_port = new_port
            self.is_connected = True
            
            # Pindahkan pesan sukses ke setelah jeda
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
        
        # Coba port yang mengandung deskriptor terlebih dahulu
        for port in ports:
            # Periksa port.description dan port.hwid untuk kecocokan yang lebih baik
            port_info = f"{port.description} {port.hwid}".lower()
            for desc in descriptors:
                if desc.lower() in port_info:
                    print(f"[Serial] Menemukan ESP32 (cocok: '{desc}') di {port.device}")
                    if self.connect(port.device, baudrate):
                        return True
        
        # Jika tidak ada yang cocok, coba port USB pertama yang ditemukan
        print("[Serial] Tidak ada ESP32 yang cocok dengan deskriptor. Mencoba /dev/ttyUSB0...")
        default_port_to_try = "/dev/ttyUSB0"
        
        # Cek apakah /dev/ttyUSB0 ada di daftar port
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

    def send_command(self, command_string):
        if not self.is_connected or self.use_dummy_serial:
            return # Jangan kirim apapun jika dummy atau tidak terhubung
            
        with self.serial_lock:
            if self.serial_port:
                try:
                    self.serial_port.write(command_string.encode("utf-8"))
                except Exception as e:
                    print(f"[Serial] Gagal mengirim data: {e}")
                    self.disconnect() # Putuskan koneksi jika terjadi error

    def read_line(self):
        # --- LOGIKA DUMMY (Tidak Berubah) ---
        if self.use_dummy_serial:
            with self.serial_lock: # Tambahkan lock di sekitar dummy logic
                time.sleep(0.5)  # Simulasikan jeda pengiriman data dari ESP32

                # Update nilai-nilai dummy agar terlihat bergerak
                self.dummy_heading = (self.dummy_heading + 1.5) % 360
                self.dummy_lat += 0.00001 * random.uniform(0.8, 1.2)
                self.dummy_lon += 0.000005 * random.uniform(0.8, 1.2)
                self.dummy_data_counter += 1

                if self.dummy_data_counter % 2 == 0:
                    self.dummy_servo = 110 if self.dummy_servo == 90 else 90
                    self.dummy_motor = 1550 if self.dummy_motor == 1500 else 1500
                    dummy_string = f"DATA:MANUAL,{self.dummy_heading:.1f},1.2,10,{self.dummy_servo},{self.dummy_motor},{self.dummy_lat:.6f},{self.dummy_lon:.6f}"
                    return dummy_string
                else:
                    dist_to_wp = 15.0 - (self.dummy_data_counter % 10)
                    target_bearing = 120.0
                    heading_error = target_bearing - self.dummy_heading
                    if heading_error > 180: heading_error -= 360
                    if heading_error < -180: heading_error += 360

                    dummy_string = f"DATA:AUTO,WAYPOINT,2,{dist_to_wp:.1f},{target_bearing:.1f},{self.dummy_heading:.1f},{heading_error:.1f},100,1600,1.1,11,{self.dummy_lat:.6f},{self.dummy_lon:.6f}"
                    return dummy_string
        # --- AKHIR LOGIKA DUMMY ---

        # --- Logika Asli (jika mode dummy tidak aktif) ---
        if not self.is_connected:
            return None # Jangan coba membaca jika tidak terhubung

        with self.serial_lock:
            if self.serial_port:
                try:
                    # Cek dulu apakah ada data yang menunggu
                    if self.serial_port.in_waiting > 0:
                        line = self.serial_port.readline()
                        return (
                            line.decode("utf-8", errors="ignore")
                            .strip()
                        )
                    else:
                        # Tidak ada data, kembalikan None agar loop bisa tidur
                        return None 
                except serial.SerialException as e:
                    print(f"[Serial] Error membaca data: {e}. Memutuskan koneksi.")
                    self.disconnect()
                except Exception as e:
                    print(f"[Serial] Error tidak terduga saat membaca: {e}")
                    self.disconnect()
        return None
