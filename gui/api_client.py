# gui/api_client.py
# --- PERBAIKAN FINAL: Prosedur koneksi/diskoneksi serial yang lebih kuat ---

import time
import threading
import serial
import serial.tools.list_ports
import math
from PySide6.QtCore import QObject, Signal, Slot

class ApiClient(QObject):
    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)
    mode_changed_for_video = Signal(str)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = True
        
        # --- Gunakan Lock untuk sinkronisasi thread ---
        self.serial_lock = threading.Lock()

        self.current_state = { "control_mode": "MANUAL", "latitude": 0.0, "longitude": 0.0, "heading": 0.0, "waypoints": [], "current_waypoint_index": 0, }
        self.vision_override_active = False
        
        self._read_thread = threading.Thread(target=self._read_from_serial, daemon=True)
        self._logic_thread = threading.Thread(target=self._main_logic_loop, daemon=True)
        self._read_thread.start()
        self._logic_thread.start()
    
    # ... (Fungsi helper navigasi tidak berubah) ...
    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        lat1_rad, lon1_rad, lat2_rad, lon2_rad = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon, dlat = lon2_rad - lon1_rad, lat2_rad - lat1_rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def _get_bearing_to_point(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dLon = lon2 - lon1
        y, x = math.sin(dLon) * math.cos(lat2), math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        return (math.degrees(math.atan2(y, x)) + 360) % 360

    def _parse_telemetry(self, line):
        try:
            parts = line.strip('T:').split(';')
            for part in parts:
                data = part.split(',')
                if data[0] == "GPS": self.current_state['latitude'], self.current_state['longitude'] = float(data[1]), float(data[2])
                elif data[0] == "COMP": self.current_state['heading'] = float(data[1])
            self.data_updated.emit(self.current_state)
        except (ValueError, IndexError) as e:
            print(f"Error parsing telemetri: {e} - Data: {line}")


    def _read_from_serial(self):
        while self.running:
            port = None
            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    port = self.serial_port

            if port:
                try:
                    line = port.readline().decode('utf-8', errors='ignore').strip()
                    if line and line.startswith("T:"):
                        self._parse_telemetry(line)
                except (serial.SerialException, TypeError, OSError):
                    # Jika terjadi error saat membaca, lepaskan koneksi
                    self.disconnect_serial()
            else:
                # Jika tidak ada port, tunggu sejenak
                time.sleep(0.5)

    def _main_logic_loop(self):
        # ... (fungsi ini tidak berubah) ...
        while self.running:
            time.sleep(0.2)
            if self.current_state.get("control_mode") != "AUTO" or self.vision_override_active:
                continue
            
            waypoints, wp_index = self.current_state.get("waypoints", []), self.current_state.get("current_waypoint_index", 0)
            if not waypoints or wp_index >= len(waypoints):
                self.send_serial_command("S1500;D90\n"); continue

            current_lat, current_lon, current_heading = self.current_state.get("latitude", 0.0), self.current_state.get("longitude", 0.0), self.current_state.get("heading", 0.0)
            target_wp = waypoints[wp_index]
            distance_to_wp = self._haversine_distance(current_lat, current_lon, target_wp['lat'], target_wp['lon'])

            if distance_to_wp < 7.0:
                print(f"Waypoint {wp_index + 1} tercapai. Lanjut ke waypoint berikutnya.")
                self.current_state["current_waypoint_index"] += 1; continue

            target_bearing = self._get_bearing_to_point(current_lat, current_lon, target_wp['lat'], target_wp['lon'])
            turn_error = (target_bearing - current_heading + 180) % 360 - 180
            servo_angle = max(45, min(135, int(90 - (turn_error / 90.0) * 45)))
            motor_pwm = int(1650 - (abs(turn_error) / 90.0) * 100)
            self.send_serial_command(f"S{motor_pwm};D{servo_angle}\n")


    # --- PERBAIKAN UTAMA PADA KONEKSI/DISKONEKSI ---
    def disconnect_serial(self):
        """Fungsi terpusat untuk menutup koneksi serial dengan aman."""
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                    print(f"Port {self.serial_port.port} berhasil ditutup.")
                except Exception as e:
                    print(f"Error saat menutup port: {e}")
                self.serial_port = None
                self.connection_status_changed.emit(False, "Koneksi terputus")

    @Slot(dict)
    def connect_manual(self, connection_details):
        """Prosedur koneksi yang lebih kuat."""
        # 1. Putuskan koneksi lama jika ada
        self.disconnect_serial()
        time.sleep(0.1) # Beri waktu OS untuk melepaskan port

        # 2. Coba buat koneksi baru
        port_name = connection_details.get("serial_port")
        baudrate = connection_details.get("baud_rate")
        try:
            print(f"Mencoba terhubung ke {port_name}...")
            new_port = serial.Serial(port_name, baudrate, timeout=1)
            
            with self.serial_lock:
                self.serial_port = new_port
            
            self.connection_status_changed.emit(True, f"Terhubung ke {port_name}")
            print(f"Berhasil terhubung ke {port_name}")
        except serial.SerialException as e:
            self.connection_status_changed.emit(False, f"Gagal membuka {port_name}: {e}")
            with self.serial_lock:
                self.serial_port = None

    def send_serial_command(self, command_string):
        """Mengirim perintah dengan aman menggunakan lock."""
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.write(command_string.encode('utf-8'))
                    # print(f"Mengirim ke ESP32: {command_string.strip()}") # Nonaktifkan untuk mengurangi spam
                except (serial.SerialException, TypeError, OSError) as e:
                    print(f"Gagal mengirim data: {e}")
                    self.disconnect_serial()
    
    # ... (Fungsi handle_* dan set_waypoints tidak berubah) ...
    @Slot(str)
    def handle_mode_change(self, mode):
        print(f"--- [ApiClient] Mode diubah menjadi: {mode} ---")
        self.current_state["control_mode"] = mode
        self.mode_changed_for_video.emit(mode)

    @Slot(list)
    def handle_manual_keys(self, keys):
        if self.current_state.get("control_mode") != "MANUAL":
            return
        forward_input = 1 if 'W' in keys else -1 if 'S' in keys else 0
        turn_input = 1 if 'D' in keys else -1 if 'A' in keys else 0
        motor_pwm = 1500 + forward_input * 150
        servo_angle = max(0, min(180, int(90 - turn_input * 45)))
        self.send_serial_command(f"S{motor_pwm};D{servo_angle}\n")

    @Slot(list)
    def set_waypoints(self, waypoints):
        print(f"Menerima {len(waypoints)} waypoints baru dari GUI.")
        self.current_state["waypoints"] = waypoints
        self.current_state["current_waypoint_index"] = 0

    @Slot(dict)
    def handle_vision_status(self, vision_data):
        if self.current_state.get("control_mode") != "AUTO":
            return
        status, command = vision_data.get('status'), vision_data.get('command')
        self.vision_override_active = (status == 'ACTIVE')
        if self.vision_override_active and command:
            self.send_serial_command(command)

    def shutdown(self):
        print("Memulai prosedur shutdown ApiClient...")
        self.running = False
        self.disconnect_serial()
        # Tunggu thread selesai setelah flag running diubah
        if self._read_thread.is_alive(): self._read_thread.join(timeout=1.0)
        if self._logic_thread.is_alive(): self._logic_thread.join(timeout=1.0)
        print("ApiClient shutdown selesai.")