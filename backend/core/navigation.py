# backend/navigation.py
# Modul ini berisi semua fungsi dan logika matematika untuk navigasi.

import math

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius bumi dalam meter
    lat1_rad, lon1_rad, lat2_rad, lon2_rad = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def get_bearing_to_point(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def run_pure_pursuit_logic(current_state, config):
    """
    Menghitung sudut servo dan PWM motor berdasarkan logika Pure Pursuit.
    Mengembalikan tuple (servo_angle, motor_pwm).
    """
    waypoints = current_state.get("waypoints", [])
    wp_index = current_state.get("current_waypoint_index", 0)
    nav_config = config.get("navigation", {})
    actuator_config = config.get("actuators", {})

    if not waypoints or wp_index >= len(waypoints):
        pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
        servo_default = actuator_config.get("servo_default_angle", 90)
        return servo_default, pwm_stop

    current_lat = current_state.get("latitude")
    current_lon = current_state.get("longitude")
    current_heading = current_state.get("heading")
    target_wp = waypoints[wp_index]

    target_bearing = get_bearing_to_point(current_lat, current_lon, target_wp['lat'], target_wp['lon'])
    turn_error = (target_bearing - current_heading + 180) % 360 - 180
    
    current_state['cog'] = target_bearing

    distance_to_wp = haversine_distance(current_lat, current_lon, target_wp['lat'], target_wp['lon'])
    if distance_to_wp < nav_config.get("waypoint_reach_distance_m", 7.0):
        print(f"Waypoint {wp_index + 1} tercapai.")
        current_state["current_waypoint_index"] += 1
        pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
        servo_default = actuator_config.get("servo_default_angle", 90)
        return servo_default, pwm_stop

    servo_min = actuator_config.get("servo_min_angle", 45)
    servo_max = actuator_config.get("servo_max_angle", 135)
    servo_default = actuator_config.get("servo_default_angle", 90)
    motor_base = actuator_config.get("motor_pwm_auto_base", 1650)
    motor_reduction = actuator_config.get("motor_pwm_auto_reduction", 100)
    
    servo_angle = max(servo_min, min(servo_max, int(servo_default - (turn_error / 90.0) * (servo_default - servo_min))))
    motor_pwm = int(motor_base - (abs(turn_error) / 90.0) * motor_reduction)
    
    return servo_angle, motor_pwm