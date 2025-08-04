# gui/components/camera/overlay_utils.py
# --- PERBAIKAN: Kembali ke metode overlay tunggal yang andal ---

import cv2
import time
import random
import imgkit
import numpy as np
from datetime import datetime
from pathlib import Path

# Konfigurasi wkhtmltoimage (tidak berubah)
path_wkhtmltoimage = r'C:\Program Files\wkhtmltopdf\bin\wkhtmltoimage.exe'
config = imgkit.config(wkhtmltoimage=path_wkhtmltoimage)


def create_overlay_from_html(telemetry_data, mission_type="Surface Imaging"):
    """
    Membuat satu gambar overlay utuh (header + info) dari template HTML.
    """
    # --- 1. Ambil dan Format Data ---
    now = datetime.now()
    lat = telemetry_data.get('latitude', 1.177697)
    lon = telemetry_data.get('longitude', 104.319206)
    sog_ms = telemetry_data.get('speed', 0.0)
    cog = telemetry_data.get('heading', 0.0)
    
    day_map = ["Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"]
    
    # Dapatkan path absolut ke folder assets
    current_dir = Path(__file__).parent
    assets_path = (current_dir.parents[1] / 'assets').as_uri()

    replacements = {
        "MISSION_TYPE": mission_type.upper(),
        "TIME": now.strftime("%H:%M"),
        "DATE_DAY": f"{day_map[now.weekday()]}, {now.strftime('%d/%m/%Y')}",
        "LOCATION": "ASV NAVANTARA - UMRAH",
        "LATITUDE": f"{abs(lat):.6f}°{'N' if lat >= 0 else 'S'}",
        "LONGITUDE": f"{abs(lon):.6f}°{'E' if lon >= 0 else 'W'}",
        "SOG_KNOT": f"{sog_ms * 1.94384:.2f}",
        "SOG_KMH": f"{sog_ms * 3.6:.2f}",
        "COG_DEG": f"{cog:.2f}",
        "PHOTO_CODE": f"KKI25-{time.strftime('%H%M%S')}-{random.randint(100,999)}",
        "ASSETS_PATH": assets_path
    }

    # --- 2. Baca Template HTML ---
    try:
        template_path = current_dir / "template.html"
        with open(template_path, 'r', encoding='utf-8') as f:
            html_content = f.read()
    except FileNotFoundError:
        print(f"ERROR: File 'template.html' tidak ditemukan di {current_dir}")
        return None

    # --- 3. Ganti Placeholder ---
    for key, value in replacements.items():
        html_content = html_content.replace(f"{{{key}}}", str(value))

    # --- 4. Konversi HTML ke Gambar ---
    try:
        options = {
            'format': 'png',
            'crop-w': '640', 'crop-h': '480',
            'encoding': "UTF-8",
            '--enable-local-file-access': None,
            '--transparent': '', 
            '--quiet': ''
        }
        img_bytes = imgkit.from_string(html_content, False, options=options, config=config)
        nparr = np.frombuffer(img_bytes, np.uint8)
        return cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
    except Exception as e:
        print(f"Error saat mengonversi HTML ke gambar: {e}")
        return None

def apply_overlay(background, overlay_image):
    """Menempelkan gambar overlay ke gambar background."""
    if overlay_image is None: return background
    h, w, c = overlay_image.shape
    if c < 4: return background
    
    alpha = overlay_image[:, :, 3] / 255.0
    
    for c_channel in range(0, 3):
        background[0:h, 0:w, c_channel] = (alpha * overlay_image[:, :, c_channel] +
                                          (1 - alpha) * background[0:h, 0:w, c_channel])
    return background