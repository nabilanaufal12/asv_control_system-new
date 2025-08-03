# gui/components/camera/overlay_utils.py
# --- FILE BARU: Utilitas untuk menggambar overlay pada gambar ---

import cv2
import time
import random
from datetime import datetime

def draw_geotag_overlay(telemetry_data, image, mission_type="Surface Imaging"):
    """Menggambar overlay yang rapi dan informatif, menggabungkan elemen terbaik."""
    h, w, _ = image.shape
    overlay = image.copy()
    
    # --- Pengaturan Umum ---
    font = cv2.FONT_HERSHEY_SIMPLEX
    white = (255, 255, 255)
    yellow = (0, 215, 255)
    
    # --- Latar Belakang Utama ---
    start_y_bg = h - 210
    cv2.rectangle(overlay, (10, start_y_bg), (w - 10, h - 10), (0,0,0), -1)
    alpha = 0.6
    image = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

    # --- Blok Kiri Bawah (Informasi Utama) ---
    start_x, start_y = 25, h - 180
    
    # Judul Misi
    cv2.putText(image, mission_type.upper(), (start_x, start_y), font, 0.9, white, 2, cv2.LINE_AA)

    # Kotak Waktu
    now = datetime.now()
    time_str = now.strftime("%H:%M")
    cv2.rectangle(image, (start_x, start_y + 15), (start_x + 110, start_y + 48), yellow, -1)
    cv2.putText(image, "VISIT", (start_x + 8, start_y + 39), font, 0.7, (0,0,0), 2, cv2.LINE_AA)
    cv2.putText(image, time_str, (start_x + 60, start_y + 39), font, 0.7, (0,0,0), 2, cv2.LINE_AA)

    # Detail Teks (Tanggal, Lokasi, Koordinat, SOG, COG)
    day_map = ["Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"]
    day = day_map[now.weekday()]
    date_str = now.strftime("%d/%m/%Y")
    full_date_str = f"{day}, {date_str}"
    
    lat = telemetry_data.get('latitude', 1.177697)
    lon = telemetry_data.get('longitude', 104.319206)
    sog_ms = telemetry_data.get('speed', 0.0)
    cog = telemetry_data.get('heading', 0.0)
    sog_knot = sog_ms * 1.94384
    sog_kmh = sog_ms * 3.6

    coord_str = f"{abs(lat):.6f}°{'N' if lat >= 0 else 'S'}, {abs(lon):.6f}°{'E' if lon >= 0 else 'W'}"
    sog_str = f"SOG: {sog_knot:.2f} knot / {sog_kmh:.2f} km/h"
    cog_str = f"COG: {cog:.2f} deg"
    
    text_y = start_y + 80
    font_scale_small = 0.5
    y_step = 22
    cv2.putText(image, full_date_str, (start_x, text_y), font, font_scale_small, white, 1, cv2.LINE_AA)
    cv2.putText(image, "Lokasi: ASV NAVANTARA - UMRAH", (start_x, text_y + y_step), font, font_scale_small, white, 1, cv2.LINE_AA)
    cv2.putText(image, coord_str, (start_x, text_y + y_step * 2), font, font_scale_small, white, 1, cv2.LINE_AA)
    cv2.putText(image, sog_str, (start_x, text_y + y_step * 3), font, font_scale_small, white, 1, cv2.LINE_AA)
    cv2.putText(image, cog_str, (start_x, text_y + y_step * 4), font, font_scale_small, white, 1, cv2.LINE_AA)

    # Garis pemisah vertikal
    cv2.line(image, (start_x - 8, start_y + 65), (start_x - 8, text_y + y_step * 4 + 5), yellow, 2)

    # Footer (Kode Foto Unik)
    photo_code = f"Kode Foto: KKI25-{time.strftime('%H%M%S')}-{random.randint(100,999)}"
    cv2.line(image, (15, h - 48), (w - 15, h - 48), (255,255,255), 1)
    # Ikon centang
    cv2.line(image, (start_x, h - 28), (start_x + 5, h - 23), white, 2)
    cv2.line(image, (start_x + 5, h - 23), (start_x + 12, h - 33), white, 2)
    cv2.putText(image, photo_code, (start_x + 25, h - 25), font, 0.5, white, 1, cv2.LINE_AA)

    # --- Kanan Atas (Branding) ---
    cv2.putText(image, "NAVANTARA - UMRAH", (w - 200, 30), font, 0.6, white, 2, cv2.LINE_AA)
    cv2.putText(image, "Foto 100% akurat", (w - 200, 50), font, 0.4, (200,200,200), 1, cv2.LINE_AA)

    return image