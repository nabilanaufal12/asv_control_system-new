# gui/api_client.py
# --- FINAL GABUNGAN: Koneksi Andal, Deteksi Port Otomatis, dan Navigasi Pure Pursuit ---

import time
import threading
import serial
import serial.tools.list_ports
import math
from PySide6.QtCore import QObject, Signal, Slot

class ApiClient(QObject):
    """
    Kelas ini bertindak sebagai "otak" di sisi GUI, menjembatani antarmuka
    dengan logika kontrol dan komunikasi perangkat keras.
    """
    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)
    mode_changed_for_video = Signal(str)

    def __init__(self, use_simulation=False):
        super().__init__()
        self.serial_port = None
        self.running = True
        self.use_simulation = use_simulation
        
        self.serial_lock = threading.Lock()

        # State terpusat untuk menyimpan semua data penting ASV
        self.current_state = {
            "control_mode": "MANUAL",
            "latitude": -6.9180, "longitude": 107.6185, "heading": 90.0,
            "cog": 0.0, "speed": 0.0, "battery_voltage": 12.5,
            "status": "DISCONNECTED", "mission_time": "00:00:00",
            "waypoints": [], "current_waypoint_index": 0,
            "manual_keys": set(), # Untuk mode simulasi
        }
        self.vision_override_active = False
        
        # Jalankan thread yang sesuai (simulasi atau hardware)
        if self.use_simulation:
            self._logic_thread = threading.Thread(target=self._simulation_loop, daemon=True)
            print("ApiClient berjalan dalam MODE SIMULASI.")
        else:
            self._read_thread = threading.Thread(target=self._read_from_serial, daemon=True)
            self._logic_thread = threading.Thread(target=self._main_logic_loop, daemon=True)
            self._read_thread.start()
            print("ApiClient berjalan dalam MODE HARDWARE.")
        
        self._logic_thread.start()

    # --- Bagian Komunikasi Serial & Hardware ---

    def _read_from_serial(self):
        """Loop di background untuk terus membaca data dari port serial."""
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
                    self.disconnect_serial()
            else:
                time.sleep(0.5)

    def _parse_telemetry(self, line):
        """Mem-parsing string telemetri dari ESP32."""
        try:
            parts = line.strip('T:').split(';')
            for part in parts:
                data = part.split(',')
                if data[0] == "GPS":
                    self.current_state['latitude'] = float(data[1])
                    self.current_state['longitude'] = float(data[2])
                elif data[0] == "COMP":
                    self.current_state['heading'] = float(data[1])
                elif data[0] == "SPD":
                    self.current_state['speed'] = float(data[1])
                elif data[0] == "BAT":
                    self.current_state['battery_voltage'] = float(data[1])
            self.data_updated.emit(self.current_state.copy())
        except (ValueError, IndexError) as e:
            print(f"Error parsing telemetri: {e} - Data: {line}")

    @Slot(dict)
    def connect_to_port(self, connection_details):
        """Slot untuk memulai koneksi serial, bisa otomatis atau manual."""
        self.disconnect_serial()
        time.sleep(0.1)

        port_name = connection_details.get("serial_port")
        baudrate = connection_details.get("baud_rate")

        if port_name == "AUTO":
            self.find_and_connect_esp32(baudrate)
        else:
            self.connect_manual(port_name, baudrate)

    def connect_manual(self, port_name, baudrate):
        """Menghubungkan ke port serial yang ditentukan secara manual."""
        try:
            print(f"Mencoba terhubung ke {port_name}...")
            new_port = serial.Serial(port_name, baudrate, timeout=1)
            with self.serial_lock:
                self.serial_port = new_port
            self.connection_status_changed.emit(True, f"Terhubung ke {port_name}")
            self.current_state["status"] = "CONNECTED"
            print(f"Berhasil terhubung ke {port_name}")
        except serial.SerialException as e:
            self.connection_status_changed.emit(False, f"Gagal membuka {port_name}: {e}")
            with self.serial_lock:
                self.serial_port = None

    def find_and_connect_esp32(self, baudrate):
        """Mencari dan terhubung ke port ESP32 secara otomatis."""
        ports = serial.tools.list_ports.comports()
        esp32_port = None
        for port in ports:
            if "Silicon Labs" in port.description or "CH340" in port.description or "CP210x" in port.description:
                esp32_port = port.device
                break
        
        if esp32_port:
            self.connect_manual(esp32_port, baudrate)
        else:
            self.connection_status_changed.emit(False, "ESP32 tidak ditemukan")
            print("ESP32 tidak ditemukan pada port manapun.")

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
                self.current_state["status"] = "DISCONNECTED"
                self.connection_status_changed.emit(False, "Koneksi terputus")

    def send_serial_command(self, command_string):
        """Mengirim perintah ke ESP32 dengan aman menggunakan lock."""
        if self.use_simulation:
            print(f"[SIMULASI] Mengirim Perintah: {command_string.strip()}")
            return

        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.write(command_string.encode('utf-8'))
                except (serial.SerialException, TypeError, OSError) as e:
                    print(f"Gagal mengirim data: {e}")
                    self.disconnect_serial()

    # --- Bagian Logika Navigasi & Simulasi ---

    def _main_logic_loop(self):
        """Loop di background untuk menjalankan logika navigasi otomatis dengan data asli."""
        while self.running:
            time.sleep(0.2)
            if self.current_state.get("control_mode") != "AUTO" or self.vision_override_active:
                self.current_state['cog'] = self.current_state.get('heading', 0.0)
                continue
            
            self.run_pure_pursuit_logic()
    
    def _simulation_loop(self):
        """Loop di background untuk menyimulasikan pergerakan kapal."""
        start_time = time.time()
        while self.running:
            turn_rate = 0.0
            speed_ms = 0.0

            if self.current_state["control_mode"] == "MANUAL":
                keys = self.current_state.get("manual_keys", set())
                speed_ms = 1.0 if 'W' in keys else -1.0 if 'S' in keys else 0
                turn_input = 1 if 'D' in keys else -1 if 'A' in keys else 0
                turn_rate = turn_input * 3.0
                self.current_state["status"] = "MANUAL CONTROL" if keys else "IDLE"

            elif self.current_state["control_mode"] == "AUTO":
                turn_rate = self.run_pure_pursuit_logic(is_simulation=True)
                speed_ms = 1.2 # Kecepatan konstan untuk simulasi auto

            # Update state berdasarkan hasil simulasi
            self.current_state["heading"] = (self.current_state["heading"] + turn_rate) % 360
            
            rad_heading = math.radians(self.current_state["heading"])
            speed_factor = speed_ms * 0.000009 # Aproksimasi: 1 m/s = 0.000009 derajat lat/lon
            
            self.current_state["latitude"] += math.cos(rad_heading) * speed_factor
            self.current_state["longitude"] += math.sin(rad_heading) * speed_factor
            self.current_state["speed"] = abs(speed_ms)

            elapsed = time.time() - start_time
            self.current_state["mission_time"] = time.strftime('%H:%M:%S', time.gmtime(elapsed))
            self.data_updated.emit(self.current_state.copy())
            time.sleep(0.1)

    def run_pure_pursuit_logic(self, is_simulation=False):
        """Menjalankan logika Pure Pursuit dan mengirim perintah atau mengembalikan turn_rate."""
        waypoints = self.current_state.get("waypoints", [])
        wp_index = self.current_state.get("current_waypoint_index", 0)

        if not waypoints or wp_index >= len(waypoints):
            if not is_simulation: self.send_serial_command("S1500;D90\n")
            self.current_state['cog'] = self.current_state.get('heading', 0.0)
            return 0.0

        current_lat = self.current_state.get("latitude", 0.0)
        current_lon = self.current_state.get("longitude", 0.0)
        current_heading = self.current_state.get("heading", 0.0)
        target_wp = waypoints[wp_index]
        
        target_bearing = self._get_bearing_to_point(current_lat, current_lon, target_wp['lat'], target_wp['lon'])
        self.current_state['cog'] = target_bearing
        
        distance_to_wp = self._haversine_distance(current_lat, current_lon, target_wp['lat'], target_wp['lon'])

        if distance_to_wp < 7.0:
            print(f"Waypoint {wp_index + 1} tercapai. Lanjut ke waypoint berikutnya.")
            self.current_state["current_waypoint_index"] += 1
            return 0.0

        turn_error = (target_bearing - current_heading + 180) % 360 - 180
        
        if is_simulation:
            speed = 1.2
            lookahead_distance = 15.0
            turn_rate_rad = (2 * speed * math.sin(math.radians(turn_error))) / lookahead_distance
            turn_rate_deg = math.degrees(turn_rate_rad) * 0.1
            return max(-4.0, min(4.0, turn_rate_deg))
        else:
            servo_angle = max(45, min(135, int(90 - (turn_error / 90.0) * 45)))
            motor_pwm = int(1650 - (abs(turn_error) / 90.0) * 100)
            self.send_serial_command(f"S{motor_pwm};D{servo_angle}\n")
            return 0.0

    # --- Bagian Fungsi Helper & Kalkulasi ---
    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        # (Fungsi ini tidak berubah)
        R = 6371000
        lat1_rad, lon1_rad, lat2_rad, lon2_rad = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon, dlat = lon2_rad - lon1_rad, lat2_rad - lat1_rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def _get_bearing_to_point(self, lat1, lon1, lat2, lon2):
        # (Fungsi ini tidak berubah)
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dLon = lon2 - lon1
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        return (math.degrees(math.atan2(y, x)) + 360) % 360

    # --- Bagian Slot untuk Interaksi GUI ---
    @Slot(str)
    def handle_mode_change(self, mode):
        print(f"--- [ApiClient] Mode diubah menjadi: {mode} ---")
        self.current_state["control_mode"] = mode
        self.mode_changed_for_video.emit(mode)

    @Slot(list)
    def handle_manual_keys(self, keys):
        if self.current_state.get("control_mode") != "MANUAL":
            return
            
        if self.use_simulation:
            self.current_state["manual_keys"] = set(keys)
        else:
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
        if not self.use_simulation:
            self.disconnect_serial()
            if self._read_thread.is_alive(): self._read_thread.join(timeout=1.0)
        if self._logic_thread.is_alive(): self._logic_thread.join(timeout=1.0)
        print("ApiClient shutdown selesai.")