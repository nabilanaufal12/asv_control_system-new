# src/navantara_backend/core/kalman_filter.py
import numpy as np

class SimpleEKF:
    """
    Kelas EKF sederhana untuk fusi sensor (GPS, Kompas, IMU).
    State vector [x, y, heading, v, omega] -> [pos_x, pos_y, heading, kecepatan, kecepatan_sudut]
    """
    def __init__(self, initial_state, initial_covariance):
        self.state = initial_state  # Vektor state awal
        self.P = initial_covariance # Matriks kovariansi error awal
        self.Q = np.diag([0.1, 0.1, np.radians(1.0), 0.1, np.radians(1.0)]) # Noise proses
        self.R_gps = np.diag([0.5, 0.5])  # Noise pengukuran GPS
        self.R_comp = np.diag([np.radians(2.0)]) # Noise pengukuran Kompas
        self.R_imu = np.diag([0.1, np.radians(2.0)]) # Noise pengukuran IMU (kecepatan & giro)

    def predict(self, dt):
        # Model prediksi gerak (Constant Velocity, Constant Turn Rate)
        x, y, theta, v, omega = self.state
        
        # Prediksi state berikutnya
        self.state[0] = x + v * np.cos(theta) * dt
        self.state[1] = y + v * np.sin(theta) * dt
        self.state[2] = self.normalize_angle(theta + omega * dt)
        # v dan omega diasumsikan konstan untuk prediksi singkat
        
        # Matriks Jacobian dari model gerak (F)
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt, 0],
            [0, 1, v * np.cos(theta) * dt, np.sin(theta) * dt, 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])
        
        # Prediksi kovariansi error
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, z_gps):
        # Matriks Jacobian pengukuran GPS (H)
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])
        # Inovasi atau residual
        y = z_gps - H @ self.state
        # Gain Kalman
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        # Update state dan kovariansi
        self.state = self.state + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_compass(self, z_comp):
        H = np.array([[0, 0, 1, 0, 0]])
        y = z_comp - self.state[2]
        y = self.normalize_angle(y)
        S = H @ self.P @ H.T + self.R_comp
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_imu(self, z_imu):
        # Mengupdate kecepatan dan kecepatan sudut dari IMU
        H = np.array([
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])
        y = z_imu - H @ self.state
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def normalize_angle(self, angle):
        """Normalisasi sudut ke rentang [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi