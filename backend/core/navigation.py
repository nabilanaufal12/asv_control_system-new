# backend/core/navigation.py
# --- VERSI MODIFIKASI: Integrasi Kontroler PID untuk Heading ---

import math
import time


def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius bumi dalam meter
    lat1_rad, lon1_rad, lat2_rad, lon2_rad = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def get_bearing_to_point(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(
        dLon
    )
    return (math.degrees(math.atan2(y, x)) + 360) % 360


class PIDController:
    """Kontroler PID sederhana untuk menstabilkan heading."""

    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_time = time.time()
        self.last_error = 0
        self.integral = 0

    def update(self, current_value):
        """Menghitung output PID berdasarkan nilai saat ini."""
        current_time = time.time()
        dt = current_time - self.last_time

        # Menghitung error sudut yang benar (mempertimbangkan putaran 360 derajat)
        error = self.setpoint - current_value
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Proportional term
        p_out = self.Kp * error

        # Integral term
        self.integral += error * dt
        i_out = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        d_out = self.Kd * derivative

        # Simpan state untuk iterasi berikutnya
        self.last_error = error
        self.last_time = current_time

        # Total output
        output = p_out + i_out + d_out
        return output

    def reset(self):
        """Mereset state kontroler."""
        self.last_time = time.time()
        self.last_error = 0
        self.integral = 0


def run_navigation_logic(current_state, config, pid_controller):
    """
    Menggabungkan Pure Pursuit untuk COG dan PID untuk koreksi heading.
    Mengembalikan tuple (servo_angle, motor_pwm).
    """
    waypoints = current_state.get("waypoints", [])
    wp_index = current_state.get("current_waypoint_index", 0)
    nav_config = config.get("navigation", {})
    actuator_config = config.get("actuators", {})

    if not waypoints or wp_index >= len(waypoints):
        pid_controller.reset()
        pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
        servo_default = actuator_config.get("servo_default_angle", 90)
        return servo_default, pwm_stop

    current_lat = current_state.get("latitude")
    current_lon = current_state.get("longitude")
    current_heading = current_state.get("heading")
    target_wp = waypoints[wp_index]

    # 1. Pure Pursuit menentukan Bearing Target (COG)
    target_bearing = get_bearing_to_point(
        current_lat, current_lon, target_wp["lat"], target_wp["lon"]
    )
    current_state["cog"] = target_bearing

    # Cek apakah waypoint sudah tercapai
    distance_to_wp = haversine_distance(
        current_lat, current_lon, target_wp["lat"], target_wp["lon"]
    )
    if distance_to_wp < nav_config.get("waypoint_reach_distance_m", 7.0):
        print(f"Waypoint {wp_index + 1} tercapai.")
        current_state["current_waypoint_index"] += 1
        pid_controller.reset()  # Reset PID saat ganti waypoint
        pwm_stop = actuator_config.get("motor_pwm_stop", 1500)
        servo_default = actuator_config.get("servo_default_angle", 90)
        return servo_default, pwm_stop

    # 2. PID menghitung koreksi kemudi
    pid_controller.setpoint = target_bearing
    steering_correction = pid_controller.update(current_heading)

    # 3. Gabungkan hasil untuk menentukan sudut servo akhir
    servo_min = actuator_config.get("servo_min_angle", 45)
    servo_max = actuator_config.get("servo_max_angle", 135)
    servo_default = actuator_config.get("servo_default_angle", 90)

    # Output PID adalah koreksi, bukan error absolut.
    # Tanda negatif karena jika error positif (perlu belok kanan), servo butuh sudut lebih kecil
    servo_angle = servo_default - steering_correction
    servo_angle = int(max(servo_min, min(servo_max, servo_angle)))

    # PWM motor tetap bisa diatur berdasarkan error untuk melambat di tikungan
    motor_base = actuator_config.get("motor_pwm_auto_base", 1650)
    motor_reduction = actuator_config.get("motor_pwm_auto_reduction", 100)
    turn_error = abs(pid_controller.last_error)  # Ambil error dari PID
    motor_pwm = int(motor_base - (turn_error / 90.0) * motor_reduction)

    return servo_angle, motor_pwm
