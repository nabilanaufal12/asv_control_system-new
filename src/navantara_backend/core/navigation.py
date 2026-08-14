# backend/core/navigation.py
# --- VERSI MODIFIKASI: Dengan Anti-Windup dan Pesan Debug ---

import time


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
        # --- PERUBAHAN 1: Tambahkan batas untuk integral ---
        self.integral_max = 100  # Batas atas untuk akumulasi integral
        self.integral_min = -100  # Batas bawah untuk akumulasi integral
        # --- AKHIR PERUBAHAN ---

    def update(self, current_value):
        """Menghitung output PID berdasarkan nilai saat ini."""
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            return (
                self.Kp * self.last_error + self.Ki * self.integral
            )  # Hindari pembagian dengan nol

        error = self.setpoint - current_value
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # --- PERUBAHAN 2: Terapkan mekanisme Anti-Windup ---
        self.integral += error * dt
        # Batasi (clamp) nilai integral agar tidak meledak
        self.integral = max(self.integral_min, min(self.integral_max, self.integral))
        # --- AKHIR PERUBAHAN ---

        derivative = (error - self.last_error) / dt

        p_out = self.Kp * error
        i_out = self.Ki * self.integral
        d_out = self.Kd * derivative

        output = p_out + i_out + d_out

        self.last_error = error
        self.last_time = current_time

        print(f"    PID -> P: {p_out:.2f}, I: {i_out:.2f}, D: {d_out:.2f}")

        return output

    def reset(self):
        """Mereset state kontroler."""
        self.last_time = time.time()
        self.last_error = 0
        self.integral = 0
