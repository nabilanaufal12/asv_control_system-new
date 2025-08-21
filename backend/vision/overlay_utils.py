# backend/vision/overlay_utils.py
# --- VERSI FINAL: Menggunakan OpenCV untuk membuat overlay, menghapus imgkit ---

import cv2
import time
from datetime import datetime

def create_overlay_from_html(telemetry_data, mission_type="Surface Imaging"):
    """
    Fungsi ini sekarang menjadi pembungkus untuk fungsi OpenCV yang baru.
    Ini mempertahankan nama fungsi yang sama agar tidak perlu mengubah vision_service.py
    secara signifikan.

    Args:
        telemetry_data: Dictionary berisi data telemetri.
        mission_type: String nama misi (misal: "Surface Imaging").

    Returns:
        Gambar overlay kosong berukuran 640x480 jika diperlukan, atau None.
        Logika utama dipindahkan ke vision_service untuk akses langsung ke frame.
    """
    # Fungsi ini tidak lagi membuat gambar sendiri karena membutuhkan frame asli.
    # Logika overlay akan dipanggil langsung dari vision_service.
    # Namun, kita biarkan kerangkanya di sini untuk menjaga konsistensi impor.
    return None

def apply_overlay(background, telemetry_data, mission_type=""):
    """
    Menggambar overlay data telemetri langsung ke frame gambar menggunakan OpenCV.
    Ini menggantikan fungsi lama dan logika imgkit.

    Args:
        background: Frame gambar (NumPy array) yang akan digambar.
        telemetry_data: Dictionary berisi data telemetri.
        mission_type: String nama misi (misal: "Surface Imaging").

    Returns:
        Frame gambar dengan overlay yang sudah digambar.
    """
    # Konfigurasi untuk tampilan teks
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.55
    font_color = (255, 255, 255) # Putih
    thickness = 1
    line_type = cv2.LINE_AA

    # Buat latar belakang semi-transparan untuk teks agar mudah dibaca
    overlay = background.copy()
    h, w, _ = background.shape
    cv2.rectangle(overlay, (0, h - 110), (w, h), (0, 0, 0), -1) # Kotak hitam di bawah
    alpha = 0.6 # Tingkat transparansi
    frame_with_overlay = cv2.addWeighted(overlay, alpha, background, 1 - alpha, 0)

    # Ambil dan format data telemetri
    lat = telemetry_data.get('latitude', 0.0)
    lon = telemetry_data.get('longitude', 0.0)
    hdg = telemetry_data.get('heading', 0.0)
    cog = telemetry_data.get('cog', 0.0)
    sog_ms = telemetry_data.get('speed', 0.0)
    sog_kts = sog_ms * 1.94384
    
    timestamp = datetime.now().strftime("%d-%m-%Y %H:%M:%S")

    # Siapkan teks yang akan ditampilkan
    mission_text = f"Misi: {mission_type.upper()}"
    lat_lon_text = f"Lat: {lat:.6f} | Lon: {lon:.6f}"
    hdg_cog_text = f"HDG: {hdg:.2f} | COG: {cog:.2f}"
    sog_text = f"SOG: {sog_kts:.2f} kts"
    
    # Tulis setiap baris data ke frame
    cv2.putText(frame_with_overlay, mission_text, (10, h - 85), font, font_scale, font_color, thickness, line_type)
    cv2.putText(frame_with_overlay, lat_lon_text, (10, h - 60), font, font_scale, font_color, thickness, line_type)
    cv2.putText(frame_with_overlay, hdg_cog_text, (10, h - 35), font, font_scale, font_color, thickness, line_type)
    cv2.putText(frame_with_overlay, sog_text, (10, h - 10), font, font_scale, font_color, thickness, line_type)
    
    # Tambahkan timestamp di pojok kanan bawah
    (w_time, h_time), _ = cv2.getTextSize(timestamp, font, font_scale, thickness)
    cv2.putText(frame_with_overlay, timestamp, (w - w_time - 10, h - 10), font, font_scale, font_color, thickness, line_type)

    return frame_with_overlay