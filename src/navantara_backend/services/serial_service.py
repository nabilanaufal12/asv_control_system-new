# backend/serial_handler.py
# Modul ini bertanggung jawab untuk semua interaksi serial dengan mikrokontroler.

import serial
import serial.tools.list_ports
import threading
import time
import random  # <-- TAMBAHKAN IMPORT INI

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
            # --- LOGIKA DUMMY ---
            if self.use_dummy_serial:
                time.sleep(0.5)  # Simulasikan jeda pengiriman data dari ESP32

                # Update nilai-nilai dummy agar terlihat bergerak
                self.dummy_heading = (self.dummy_heading + 1.5) % 360
                self.dummy_lat += 0.00001 * random.uniform(0.8, 1.2)
                self.dummy_lon += 0.000005 * random.uniform(0.8, 1.2)
                self.dummy_data_counter += 1

                # Kirim data format MANUAL dan AUTO/WAYPOINT secara bergantian
                # Ini untuk menguji kedua parser di asv_handler.py

                if self.dummy_data_counter % 2 == 0:
                    # Kirim data format MANUAL (Mode MANUAL RC)
                    # Format: "DATA:MANUAL,[heading],[speed],[sats],[servoPos],[motorMicros],[lat],[lon]"
                    self.dummy_servo = 110 if self.dummy_servo == 90 else 90
                    self.dummy_motor = 1550 if self.dummy_motor == 1500 else 1500
                    dummy_string = f"DATA:MANUAL,{self.dummy_heading:.1f},1.2,10,{self.dummy_servo},{self.dummy_motor},{self.dummy_lat:.6f},{self.dummy_lon:.6f}"
                    return dummy_string
                else:
                    # Kirim data format AUTO/WAYPOINT (Mode AUTO Jetson)
                    # Format: "DATA:AUTO,WAYPOINT,[counter+1],[dist],[targetBearing],[heading],[errorHeading],[servoPos],[motorSpeed],[speed],[sats],[lat],[lon]"
                    dist_to_wp = 15.0 - (self.dummy_data_counter % 10)
                    target_bearing = 120.0
                    heading_error = target_bearing - self.dummy_heading
                    if heading_error > 180: heading_error -= 360
                    if heading_error < -180: heading_error += 360

                    dummy_string = f"DATA:AUTO,WAYPOINT,2,{dist_to_wp:.1f},{target_bearing:.1f},{self.dummy_heading:.1f},{heading_error:.1f},100,1600,1.1,11,{self.dummy_lat:.6f},{self.dummy_lon:.6f}"
                    return dummy_string
            # --- AKHIR LOGIKA DUMMY ---

            # --- Logika Asli (jika mode dummy tidak aktif) ---
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
