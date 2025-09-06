# navantara_backend/vision/path_planner.py
"""
Modul ini berisi implementasi dari algoritma Dynamic Window Approach (DWA)
untuk perencanaan jalur lokal (local path planning).
"""

import numpy as np


def dwa_path_planning(current_state, obstacles, goal_point, config):
    """
    Menghitung kecepatan linear (v) dan angular (ω) optimal menggunakan DWA.

    Args:
        current_state (dict): Status ASV saat ini. Harus mengandung 'speed' (m/s)
                              dan 'gyro_z' (rad/s, kecepatan sudut).
        obstacles (list of list): Daftar posisi rintangan, misal [[x1, y1], [x2, y2], ...].
                                  Koordinat dalam meter, relatif terhadap ASV.
        goal_point (list): Posisi titik tujuan [x, y] dalam meter, relatif terhadap ASV.
        config (dict): Objek konfigurasi aplikasi.

    Returns:
        tuple: Kecepatan linear optimal (v_opt, m/s) dan kecepatan angular optimal (omega_opt, rad/s).
    """
    # Ambil parameter DWA dari file konfigurasi
    dwa_cfg = config.get("dwa_params", {})
    max_speed = dwa_cfg.get("max_speed_mps", 1.5)  # m/s
    min_speed = dwa_cfg.get("min_speed_mps", 0.0)  # m/s
    max_yaw_rate = np.radians(dwa_cfg.get("max_yaw_rate_dps", 40.0))  # rad/s
    max_accel = dwa_cfg.get("max_accel_mps2", 0.5)  # m/s^2
    max_dyaw_rate = np.radians(dwa_cfg.get("max_dyaw_rate_dps2", 40.0))  # rad/s^2

    v_resolution = dwa_cfg.get("v_resolution", 0.1)  # m/s
    yaw_rate_resolution = np.radians(
        dwa_cfg.get("yaw_rate_resolution_dps", 1.0)
    )  # rad/s

    dt = dwa_cfg.get("dt_seconds", 0.2)  # detik
    predict_time = dwa_cfg.get("predict_time_seconds", 2.0)  # detik
    robot_radius = dwa_cfg.get("robot_radius_meters", 0.5)  # meter

    # Bobot untuk fungsi skor
    to_goal_cost_gain = dwa_cfg.get("goal_cost_gain", 1.0)
    speed_cost_gain = dwa_cfg.get("speed_cost_gain", 0.5)
    obstacle_cost_gain = dwa_cfg.get("obstacle_cost_gain", 1.5)

    # 1. Hitung Jendela Dinamis (Dynamic Window)
    v_window = [
        max(min_speed, current_state.get("speed", 0) - max_accel * dt),
        min(max_speed, current_state.get("speed", 0) + max_accel * dt),
    ]
    omega_window = [
        max(-max_yaw_rate, current_state.get("gyro_z", 0) - max_dyaw_rate * dt),
        min(max_yaw_rate, current_state.get("gyro_z", 0) + max_dyaw_rate * dt),
    ]

    best_trajectory_info = {
        "score": -float("inf"),
        "v": 0.0,
        "omega": 0.0,
        "trajectory": np.array([]),
    }

    # 2. Evaluasi semua kemungkinan trajektori
    for v in np.arange(v_window[0], v_window[1], v_resolution):
        for omega in np.arange(omega_window[0], omega_window[1], yaw_rate_resolution):

            trajectory = _simulate_trajectory(v, omega, predict_time, dt)

            # Hitung skor untuk setiap kriteria
            heading_score = to_goal_cost_gain * _calculate_heading_score(
                trajectory, goal_point
            )
            clearance_score = obstacle_cost_gain * _calculate_clearance_score(
                trajectory, obstacles, robot_radius
            )
            speed_score = speed_cost_gain * v

            # Jika trajektori menabrak, abaikan
            if clearance_score <= 0:
                continue

            total_score = heading_score + clearance_score + speed_score

            # Simpan trajektori dengan skor terbaik
            if total_score > best_trajectory_info["score"]:
                best_trajectory_info["score"] = total_score
                best_trajectory_info["v"] = v
                best_trajectory_info["omega"] = omega
                best_trajectory_info["trajectory"] = trajectory

    return best_trajectory_info["v"], best_trajectory_info["omega"]


def _simulate_trajectory(v, omega, predict_time, dt):
    """
    Mensimulasikan pergerakan ASV untuk durasi tertentu berdasarkan model unicycle.
    ASV dianggap selalu mulai dari (0, 0) dengan heading 0.
    """
    x, y, theta = 0.0, 0.0, 0.0
    trajectory = np.array([[x, y]])

    time = 0.0
    while time <= predict_time:
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += omega * dt
        trajectory = np.vstack([trajectory, [x, y]])
        time += dt

    return trajectory


def _calculate_heading_score(trajectory, goal_point):
    """
    Menghitung skor berdasarkan seberapa dekat heading akhir trajektori dengan arah ke tujuan.
    Skor dinormalisasi antara 0 dan 1.
    """
    final_x, final_y = trajectory[-1]

    # Sudut dari posisi akhir ke titik tujuan
    angle_to_goal = np.arctan2(goal_point[1] - final_y, goal_point[0] - final_x)

    # Heading akhir dari trajektori (diestimasi dari 2 titik terakhir)
    dx = final_x - trajectory[-2, 0]
    dy = final_y - trajectory[-2, 1]
    final_heading = np.arctan2(dy, dx)

    # Perbedaan sudut (error)
    angle_error = abs(angle_to_goal - final_heading)
    # Normalisasi error agar skor tertinggi (1.0) saat error = 0
    score = (np.pi - angle_error) / np.pi

    return score


def _calculate_clearance_score(trajectory, obstacles, robot_radius):
    """
    Menghitung skor berdasarkan jarak terdekat ke rintangan.
    Jika terjadi tabrakan, skor adalah 0.
    """
    if not obstacles:
        return 1.0  # Jika tidak ada rintangan, skor maksimal

    min_dist = float("inf")

    for point in trajectory:
        for obs in obstacles:
            dist = np.sqrt((point[0] - obs[0]) ** 2 + (point[1] - obs[1]) ** 2)

            if dist <= robot_radius:
                return 0.0  # Terjadi tabrakan, skor 0

            min_dist = min(min_dist, dist)

    # Skor berbanding terbalik dengan jarak, tapi kita batasi agar tidak terlalu besar
    # Jarak aman misalnya 2x radius. Jika lebih jauh, skor tetap 1.0.
    safe_distance = robot_radius * 2
    if min_dist > safe_distance:
        return 1.0
    else:
        # Normalisasi skor antara 0 dan 1 berdasarkan jarak dari radius robot ke jarak aman
        return (min_dist - robot_radius) / (safe_distance - robot_radius)
