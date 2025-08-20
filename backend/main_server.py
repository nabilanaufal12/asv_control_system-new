# backend/main_server.py
# --- VERSI BARU: Dengan perbaikan path untuk memuat config.json ---

import time
import threading
import serial
import serial.tools.list_ports
import math
import json
import os # <-- Diperlukan untuk path yang cerdas
from flask import Flask, jsonify, request

# --- PERBAIKAN DIMULAI DI SINI ---
# Bagian ini membuat skrip bisa menemukan config.json di mana pun ia dijalankan
try:
    # Dapatkan path absolut dari direktori tempat skrip ini berada (yaitu, 'backend/')
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Buat path lengkap ke config.json (satu direktori di atas 'backend/')
    config_path = os.path.join(script_dir, '..', 'config.json')

    with open(config_path, 'r') as f:
        config = json.load(f)
    print("[Server] File 'config.json' berhasil dimuat.")
except Exception as e:
    print(f"[Server] KRITIS: Gagal memuat config.json. Pastikan file ada di root proyek. Error: {e}")
    exit()
# --- AKHIR PERBAIKAN ---


class AsvHandler:
    """
    Kelas ini bertindak sebagai "otak" dari ASV, menjembatani logika kontrol
    dan komunikasi perangkat keras. Ini adalah logika inti Anda.
    """
    def __init__(self, config_data):
        self.config = config_data # Gunakan config yang sudah dimuat
        self.serial_port = None
        self.running = True
        self.serial_lock = threading.Lock()

        # State terpusat untuk menyimpan semua data penting ASV
        self.current_state = {
            "control_mode": "MANUAL", "latitude": -6.9180, "longitude": 107.6185,
            "heading": 90.0, "cog": 0.0, "speed": 0.0, "battery_voltage": 12.5,
            "status": "DISCONNECTED", "mission_time": "00:00:00", "waypoints": [],
            "current_waypoint_index": 0, "is_connected_to_serial": False,
        }
        self.vision_override_active = False
        
        self.use_simulation = self.config.get("general", {}).get("use_simulation", False)

        if self.use_simulation:
            self._logic_thread = threading.Thread(target=self._simulation_loop, daemon=True)
            print("[AsvHandler] berjalan dalam MODE SIMULASI.")
        else:
            self._read_thread = threading.Thread(target=self._read_from_serial_loop, daemon=True)
            self._logic_thread = threading.Thread(target=self._main_logic_loop, daemon=True)
            self._read_thread.start()
            print("[AsvHandler] berjalan dalam MODE HARDWARE.")
        
        self._logic_thread.start()

    # ... Sisa dari kelas AsvHandler tetap sama persis seperti sebelumnya ...
    def _read_from_serial_loop(self):
        while self.running:
            port = self.serial_port if self.serial_port and self.serial_port.is_open else None
            if port:
                try:
                    line = port.readline().decode('utf-8', errors='ignore').strip()
                    if line and line.startswith("T:"): self._parse_telemetry(line)
                except Exception: self.disconnect_serial()
            else: time.sleep(0.5)

    def _parse_telemetry(self, line):
        try:
            parts = line.strip('T:').split(';')
            for part in parts:
                data = part.split(',')
                if data[0] == "GPS":
                    self.current_state['latitude'] = float(data[1])
                    self.current_state['longitude'] = float(data[2])
                elif data[0] == "COMP": self.current_state['heading'] = float(data[1])
                elif data[0] == "SPD": self.current_state['speed'] = float(data[1])
                elif data[0] == "BAT": self.current_state['battery_voltage'] = float(data[1])
        except (ValueError, IndexError) as e: print(f"Error parsing telemetri: {e} - Data: {line}")

    def connect_serial(self, port_name, baudrate):
        self.disconnect_serial()
        time.sleep(0.1)
        try:
            print(f"[Serial] Mencoba terhubung ke {port_name}...")
            new_port = serial.Serial(port_name, int(baudrate), timeout=1)
            with self.serial_lock: self.serial_port = new_port
            self.current_state["is_connected_to_serial"] = True
            self.current_state["status"] = "CONNECTED"
            print(f"Berhasil terhubung ke {port_name}")
        except serial.SerialException as e:
            print(f"Gagal membuka {port_name}: {e}")
            self.current_state["is_connected_to_serial"] = False
            self.current_state["status"] = "DISCONNECTED"

    def find_and_connect_esp32(self, baudrate):
        ports = serial.tools.list_ports.comports()
        descriptors = self.config.get("serial_connection", {}).get("auto_connect_descriptors", [])
        for port in ports:
            for desc in descriptors:
                if desc in port.description:
                    self.connect_serial(port.device, baudrate)
                    return
        print("ESP32 tidak ditemukan")

    def disconnect_serial(self):
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open: self.serial_port.close()
        self.current_state["is_connected_to_serial"] = False
        self.current_state["status"] = "DISCONNECTED"

    def send_serial_command(self, command_string):
        if self.use_simulation:
            print(f"[SIMULASI] Mengirim Perintah: {command_string.strip()}")
            return
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try: self.serial_port.write(command_string.encode('utf-8'))
                except Exception as e:
                    print(f"Gagal mengirim data: {e}")
                    self.disconnect_serial()

    def _main_logic_loop(self):
        start_time = time.time()
        while self.running:
            elapsed = time.time() - start_time
            self.current_state["mission_time"] = time.strftime('%H:%M:%S', time.gmtime(elapsed))
            if self.current_state.get("control_mode") == "AUTO" and not self.vision_override_active:
                self.run_pure_pursuit_logic()
            else:
                self.current_state['cog'] = self.current_state.get('heading', 0.0)
            time.sleep(0.2)
    
    def _simulation_loop(self):
        start_time = time.time()
        manual_keys = set()
        while self.running:
            turn_rate, speed_ms = 0.0, 0.0
            if self.current_state["control_mode"] == "MANUAL":
                speed_ms = 1.0 if 'W' in manual_keys else -1.0 if 'S' in manual_keys else 0
                turn_input = 1 if 'D' in manual_keys else -1 if 'A' in manual_keys else 0
                turn_rate = turn_input * 3.0
                self.current_state["status"] = "MANUAL CONTROL" if manual_keys else "IDLE"
            elif self.current_state["control_mode"] == "AUTO":
                turn_rate = self.run_pure_pursuit_logic(is_simulation=True)
                speed_ms = 1.2
            
            self.current_state["heading"] = (self.current_state["heading"] + turn_rate) % 360
            rad_heading = math.radians(self.current_state["heading"])
            speed_factor = speed_ms * 0.000009
            self.current_state["latitude"] += math.cos(rad_heading) * speed_factor
            self.current_state["longitude"] += math.sin(rad_heading) * speed_factor
            self.current_state["speed"] = abs(speed_ms)
            elapsed = time.time() - start_time
            self.current_state["mission_time"] = time.strftime('%H:%M:%S', time.gmtime(elapsed))
            time.sleep(0.1)

    def run_pure_pursuit_logic(self, is_simulation=False):
        waypoints=self.current_state.get("waypoints",[]);wp_index=self.current_state.get("current_waypoint_index",0);nav_config=self.config.get("navigation",{});actuator_config=self.config.get("actuators",{});
        if not waypoints or wp_index>=len(waypoints):
            if not is_simulation:self.send_serial_command(f"S{actuator_config.get('motor_pwm_stop',1500)};D{actuator_config.get('servo_default_angle',90)}\n")
            self.current_state['cog']=self.current_state.get('heading',0.0);return 0.0
        current_lat,current_lon,current_heading=self.current_state.get("latitude",0.0),self.current_state.get("longitude",0.0),self.current_state.get("heading",0.0);target_wp=waypoints[wp_index];target_bearing=self._get_bearing_to_point(current_lat,current_lon,target_wp['lat'],target_wp['lon']);self.current_state['cog']=target_bearing;distance_to_wp=self._haversine_distance(current_lat,current_lon,target_wp['lat'],target_wp['lon']);
        if distance_to_wp<nav_config.get("waypoint_reach_distance_m",7.0):print(f"Waypoint {wp_index+1} tercapai.");self.current_state["current_waypoint_index"]+=1;return 0.0
        turn_error=(target_bearing-current_heading+180)%360-180
        if is_simulation:speed=1.2;lookahead=nav_config.get("pure_pursuit_lookahead_distance_m",15.0);turn_rate_rad=(2*speed*math.sin(math.radians(turn_error)))/lookahead;turn_rate_deg=math.degrees(turn_rate_rad)*0.1;return max(-4.0,min(4.0,turn_rate_deg))
        else:servo_min=actuator_config.get("servo_min_angle",45);servo_max=actuator_config.get("servo_max_angle",135);servo_default=actuator_config.get("servo_default_angle",90);motor_base=actuator_config.get("motor_pwm_auto_base",1650);motor_reduction=actuator_config.get("motor_pwm_auto_reduction",100);servo_angle=max(servo_min,min(servo_max,int(servo_default-(turn_error/90.0)*(servo_default-servo_min))));motor_pwm=int(motor_base-(abs(turn_error)/90.0)*motor_reduction);self.send_serial_command(f"S{motor_pwm};D{servo_angle}\n");return 0.0

    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        R=6371000;lat1,lon1,lat2,lon2=map(math.radians,[lat1,lon1,lat2,lon2]);dlon,dlat=lon2-lon1,lat2-lat1;a=math.sin(dlat/2)**2+math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2;c=2*math.atan2(math.sqrt(a),math.sqrt(1-a));return R*c
    def _get_bearing_to_point(self, lat1, lon1, lat2, lon2):
        lat1,lon1,lat2,lon2=map(math.radians,[lat1,lon1,lat2,lon2]);dLon=lon2-lon1;y=math.sin(dLon)*math.cos(lat2);x=math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(dLon);return(math.degrees(math.atan2(y,x))+360)%360

    def process_command(self, command, payload):
        if command == "CONFIGURE_SERIAL":
            if payload.get("serial_port") == "AUTO": self.find_and_connect_esp32(payload.get("baud_rate"))
            else: self.connect_serial(payload.get("serial_port"), payload.get("baud_rate"))
        elif command == "CHANGE_MODE": self.current_state["control_mode"] = payload
        elif command == "MANUAL_CONTROL" and self.current_state.get("control_mode") == "MANUAL":
            keys=set(payload);actuator_config=self.config.get("actuators",{});pwm_stop,pwm_power=actuator_config.get("motor_pwm_stop",1500),actuator_config.get("motor_pwm_manual_power",150);servo_default,servo_min=actuator_config.get("servo_default_angle",90),actuator_config.get("servo_min_angle",45);forward,turn=(1 if'W'in keys else-1 if'S'in keys else 0),(1 if'D'in keys else-1 if'A'in keys else 0);motor_pwm=pwm_stop+forward*pwm_power;servo_angle=max(0,min(180,int(servo_default-turn*(servo_default-servo_min))));self.send_serial_command(f"S{motor_pwm};D{servo_angle}\n")
        elif command == "SET_WAYPOINTS": self.current_state["waypoints"],self.current_state["current_waypoint_index"]=payload,0
        elif command == "VISION_OVERRIDE":
            self.vision_override_active=(payload.get('status')=='ACTIVE')
            if self.vision_override_active and payload.get('command'):self.send_serial_command(payload.get('command'))

app = Flask(__name__)
asv_handler = AsvHandler(config)

@app.route('/status', methods=['GET'])
def get_status(): return jsonify(asv_handler.current_state.copy())

@app.route('/command', methods=['POST'])
def handle_command():
    data = request.json
    asv_handler.process_command(data.get("command"), data.get("payload"))
    return jsonify({"status": "command_received"})

if __name__ == "__main__":
    print("🚀 Memulai Backend Server ASV...")
    app.run(host='0.0.0.0', port=5000, debug=False)