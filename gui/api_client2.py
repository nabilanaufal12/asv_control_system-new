# gui/api_client.py
# --- MODIFIKASI: Implementasi Pure Pursuit untuk navigasi yang lebih akurat & perbaikan Pause/Resume ---

import time
import threading
import math
from PySide6.QtCore import QObject, Signal

class ApiClient(QObject):
    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)

    def __init__(self):
        super().__init__()
        
        self.is_connected = True 
        self.current_state = {
            "latitude": -6.9180, "longitude": 107.6185, "heading": 90.0,
            "speed": 0.0, "battery_voltage": 12.5, "status": "IDLE",
            "mission_time": "00:00:00", "control_mode": "MANUAL",
            
            "manual_keys": set(),
            "auto_sub_mode": "IDLE",
            "waypoints": [],
            "original_waypoints": [],
            "current_waypoint_index": 0,
            "home_position": {"lat": -6.9180, "lon": 107.6185},
            "vision_data": {"detected": False, "degree": None},
            "pass_through_timer": 0
        }
        
        self._simulation_thread = threading.Thread(target=self._simulate_data_changes, daemon=True)
        self._simulation_thread.start()

    def start_polling(self, interval=1000):
        self.connection_status_changed.emit(True, "Terhubung ke Backend (Dummy)")
        
    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        dlon, dlat = lon2_rad - lon1_rad, lat2_rad - lat1_rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def _get_bearing_to_point(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dLon = lon2 - lon1
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        return bearing

    def _simulate_data_changes(self):
        start_time = time.time()
        while self.is_connected:
            lat_change, lon_change = 0.0, 0.0
            turn_rate = 0.0
            
            if self.current_state["control_mode"] == "MANUAL":
                keys = self.current_state.get("manual_keys", set())
                speed_factor = 0.00001
                if 'W' in keys: lat_change += speed_factor
                if 'S' in keys: lat_change -= speed_factor
                if 'A' in keys: lon_change -= speed_factor
                if 'D' in keys: lon_change += speed_factor
                turn_input = 1 if 'D' in keys else -1 if 'A' in keys else 0
                turn_rate = turn_input * 3.0
                self.current_state["status"] = "MANUAL CONTROL" if keys else "IDLE"

            elif self.current_state["control_mode"] == "AUTO":
                sub_mode = self.current_state["auto_sub_mode"]
                vision_detected = self.current_state["vision_data"]["detected"]
                if vision_detected and sub_mode not in ["OBJECT_FOLLOW", "PASS_THROUGH"]:
                    sub_mode = "OBJECT_FOLLOW"
                    self.current_state["auto_sub_mode"] = sub_mode
                
                if sub_mode == "WAYPOINT":
                    waypoints = self.current_state["waypoints"]
                    wp_index = self.current_state["current_waypoint_index"]
                    self.current_state["status"] = f"WAYPOINT {wp_index + 1}"
                    if not waypoints or wp_index >= len(waypoints):
                        self.current_state["status"] = "MISSION COMPLETE"
                        self.current_state["auto_sub_mode"] = "IDLE"
                    else:
                        current_lat, current_lon = self.current_state["latitude"], self.current_state["longitude"]
                        target_wp = waypoints[wp_index]
                        distance_to_wp = self._haversine_distance(current_lat, current_lon, target_wp['lat'], target_wp['lon'])
                        
                        if distance_to_wp < 7.0:
                            self.current_state["current_waypoint_index"] += 1
                        else:
                            # --- PERBAIKAN 1: Logika Navigasi Pure Pursuit ---
                            speed = 1.2 # Sedikit percepat
                            
                            # Tentukan 'lookahead distance' - seberapa jauh ke depan kapal akan menargetkan
                            lookahead_distance = 15.0 # meter
                            
                            # Hitung arah ke waypoint
                            target_bearing = self._get_bearing_to_point(current_lat, current_lon, target_wp['lat'], target_wp['lon'])
                            
                            # Hitung error antara arah kapal saat ini dan arah ke waypoint
                            turn_error = (target_bearing - self.current_state["heading"] + 180) % 360 - 180
                            
                            # Hitung laju belok menggunakan rumus Pure Pursuit sederhana
                            # Ini akan membuat belokan lebih mulus
                            turn_rate = (2 * speed * math.sin(math.radians(turn_error))) / lookahead_distance
                            # Konversi dari radian/detik (kira-kira) ke derajat/frame
                            turn_rate = math.degrees(turn_rate) * 0.1 
                            
                            # Batasi kecepatan belok maksimum
                            max_turn_rate = 4.0
                            turn_rate = max(-max_turn_rate, min(max_turn_rate, turn_rate))

                            rad = math.radians(self.current_state["heading"])
                            speed_factor = speed * 0.000001
                            lat_change = math.cos(rad) * speed_factor
                            lon_change = math.sin(rad) * speed_factor
                
                elif sub_mode == "OBJECT_FOLLOW" or sub_mode == "PASS_THROUGH":
                    speed = 0.5 if sub_mode == "OBJECT_FOLLOW" else 0.8
                    if sub_mode == "OBJECT_FOLLOW":
                        self.current_state["status"] = "OBJECT FOLLOWING"
                        degree = self.current_state["vision_data"]["degree"]
                        if degree is not None:
                            error = degree - 90
                            turn_rate = -0.2 * error
                            if abs(error) < 5:
                                self.current_state["auto_sub_mode"] = "PASS_THROUGH"
                                self.current_state["pass_through_timer"] = 20
                        else: self.current_state["auto_sub_mode"] = "WAYPOINT"
                    else:
                        self.current_state["status"] = "PASSING OBJECT"
                        self.current_state["pass_through_timer"] -= 1
                        if self.current_state["pass_through_timer"] <= 0:
                            self.current_state["auto_sub_mode"] = "WAYPOINT"
                    rad = math.radians(self.current_state["heading"])
                    speed_factor = speed * 0.000001
                    lat_change = math.cos(rad) * speed_factor
                    lon_change = math.sin(rad) * speed_factor

            self.current_state["heading"] = (self.current_state["heading"] + turn_rate) % 360
            self.current_state["latitude"] += lat_change
            self.current_state["longitude"] += lon_change
            elapsed = time.time() - start_time
            self.current_state["mission_time"] = time.strftime('%H:%M:%S', time.gmtime(elapsed))
            self.data_updated.emit(self.current_state)
            time.sleep(0.1)

    def shutdown(self):
        self.is_connected = False
        
    def send_command(self, command, payload=None):
        if command == "EMERGENCY_STOP":
            self.current_state["control_mode"] = "MANUAL"
            self.current_state["manual_keys"] = set()
            self.current_state["auto_sub_mode"] = "IDLE"
            self.current_state["status"] = "EMERGENCY"
        
        elif command == "NAVIGATION":
            if self.current_state["control_mode"] == "AUTO":
                # --- PERBAIKAN 2: Logika Start/Resume yang benar ---
                if payload == "START":
                    # Jika misi dijeda (IDLE), lanjutkan saja
                    if self.current_state["auto_sub_mode"] == "IDLE":
                        # Jika misi sudah selesai, mulai ulang dari awal
                        if self.current_state["current_waypoint_index"] >= len(self.current_state["waypoints"]):
                            self.current_state["current_waypoint_index"] = 0
                        
                        # Jika sebelumnya "Return Home", kembalikan misi asli
                        if self.current_state["original_waypoints"]:
                            self.current_state["waypoints"] = self.current_state["original_waypoints"]
                            self.current_state["original_waypoints"] = []
                            self.current_state["current_waypoint_index"] = 0

                        self.current_state["auto_sub_mode"] = "WAYPOINT"
                
                elif payload == "PAUSE":
                    self.current_state["auto_sub_mode"] = "IDLE"
                elif payload == "RETURN":
                    if not self.current_state["original_waypoints"]:
                         self.current_state["original_waypoints"] = self.current_state["waypoints"]
                    self.current_state["waypoints"] = [self.current_state["home_position"]]
                    self.current_state["current_waypoint_index"] = 0
                    self.current_state["auto_sub_mode"] = "WAYPOINT"

        elif command == "MANUAL_CONTROL": self.current_state["manual_keys"] = set(payload)
        elif command == "CHANGE_MODE":
            self.current_state["control_mode"] = payload
            if payload == "AUTO": self.current_state["auto_sub_mode"] = "IDLE"
        elif command == "SET_WAYPOINTS":
            self.current_state["waypoints"] = payload
            self.current_state["original_waypoints"] = payload
            self.current_state["current_waypoint_index"] = 0
            if self.current_state["control_mode"] == "AUTO":
                self.current_state["auto_sub_mode"] = "IDLE"
        elif command == "UPDATE_VISION_DATA": self.current_state["vision_data"] = payload
