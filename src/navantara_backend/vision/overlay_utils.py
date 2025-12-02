# backend/vision/overlay_utils.py
# --- VERSI DENGAN PATH ASET YANG DIPERBAIKI ---

import cv2
import time
import random
import numpy as np
from datetime import datetime
from pathlib import Path

# Cache untuk menyimpan logo yang sudah dimuat
loaded_logos = {}


def add_transparent_logo(background, logo, top_left_corner):
    """Menempelkan logo dengan transparansi ke background."""
    x, y = top_left_corner
    if x < 0 or y < 0:
        return background

    # Memastikan logo tidak keluar dari batas gambar
    h, w, channels = logo.shape
    if channels < 4:
        return background  # Hanya proses jika ada alpha channel

    bg_h, bg_w, _ = background.shape
    if x + w > bg_w or y + h > bg_h:
        return background

    # Ekstrak alpha channel dan buat overlay
    alpha = logo[:, :, 3] / 255.0
    for c in range(0, 3):
        background[y : y + h, x : x + w, c] = (
            alpha * logo[:, :, c] + (1 - alpha) * background[y : y + h, x : x + w, c]
        )
    return background


def get_logo(name, size):
    """Memuat, mengubah ukuran, dan menyimpan logo di cache."""
    cache_key = f"{name}_{size[0]}x{size[1]}"
    if cache_key in loaded_logos:
        return loaded_logos[cache_key]
    try:
        # --- PERBAIKAN UTAMA DI SINI ---
        # Path sekarang dibangun dari file ini, naik ke 'src', lalu turun ke 'navantara_gui/assets'
        assets_dir = Path(__file__).resolve().parents[2] / "navantara_gui" / "assets"
        logo_path = assets_dir / name
        # --- AKHIR PERBAIKAN ---

        logo = cv2.imread(str(logo_path), cv2.IMREAD_UNCHANGED)
        if logo is None:
            raise FileNotFoundError(f"File logo tidak ditemukan di: {logo_path}")

        resized_logo = cv2.resize(logo, size, interpolation=cv2.INTER_AREA)
        loaded_logos[cache_key] = resized_logo
        return resized_logo
    except Exception as e:
        print(f"Peringatan: Gagal memuat logo '{name}'. Error: {e}")
        # Kembalikan placeholder transparan jika gagal
        placeholder = np.zeros((size[1], size[0], 4), dtype=np.uint8)
        loaded_logos[cache_key] = placeholder
        return placeholder


def putText_with_shadow(
    frame, text, org, fontFace, fontScale, color, thickness=1, shadow_color=(0, 0, 0)
):
    """Menggambar teks dengan bayangan tipis untuk keterbacaan maksimal."""
    x, y = org
    # Gambar teks bayangan (sedikit di kanan bawah)
    cv2.putText(
        frame,
        text,
        (x + 1, y + 1),
        fontFace,
        fontScale,
        shadow_color,
        thickness,
        cv2.LINE_AA,
    )
    # Gambar teks utama
    cv2.putText(frame, text, (x, y), fontFace, fontScale, color, thickness, cv2.LINE_AA)


def create_overlay_from_html(telemetry_data, mission_type="Surface Imaging"):
    # Fungsi dummy untuk kompatibilitas, data langsung dilewatkan
    return telemetry_data, mission_type


def apply_overlay(background_frame, overlay_data):
    """Menggambar overlay dengan teks berbayang profesional."""
    telemetry_data, mission_type = overlay_data
    frame = background_frame.copy()
    h, w, _ = frame.shape

    # === 1. DEFINISI FONT DAN WARNA ===
    font_header = cv2.FONT_HERSHEY_DUPLEX
    font_main = cv2.FONT_HERSHEY_SIMPLEX
    font_sub = cv2.FONT_HERSHEY_SIMPLEX
    color_white = (255, 255, 255)
    color_grey = (200, 200, 200)

    # === 2. GAMBAR HEADER ===
    # Logo Sponsor
    logo_sponsor1 = get_logo("logo_sponsor1.png", size=(90, 28))
    logo_sponsor2 = get_logo("logo_sponsor2.png", size=(28, 28))
    logo_sponsor3 = get_logo("logo_sponsor3.png", size=(28, 28))
    frame = add_transparent_logo(frame, logo_sponsor1, top_left_corner=(10, 12))
    frame = add_transparent_logo(frame, logo_sponsor2, top_left_corner=(110, 12))
    frame = add_transparent_logo(frame, logo_sponsor3, top_left_corner=(145, 12))

    # Teks Header Kanan
    putText_with_shadow(
        frame, "NAVANTARA - UMRAH", (w - 200, 28), font_header, 0.5, color_white
    )
    putText_with_shadow(
        frame, "Foto 100% akurat", (w - 125, 45), font_sub, 0.35, color_grey
    )

    # === 3. GAMBAR INFO BOX (HANYA TEKS DAN LOGO) ===
    info_x = 15
    info_y_start = h - 160  # Posisi mulai blok info dari bawah

    # Logo di dalam info box
    logo_umrah = get_logo("logo_umrah.png", size=(35, 35))
    logo_src = get_logo("logo_src.png", size=(35, 35))
    frame = add_transparent_logo(
        frame, logo_umrah, top_left_corner=(info_x, info_y_start - 60)
    )
    frame = add_transparent_logo(
        frame, logo_src, top_left_corner=(info_x + 45, info_y_start - 60)
    )

    # Judul Misi
    putText_with_shadow(
        frame,
        mission_type.upper(),
        (info_x, info_y_start),
        font_main,
        0.9,
        color_white,
        2,
    )
    # Garis putih horizontal halus (Posisi di +8)
    cv2.line(
        frame,
        (info_x, info_y_start + 8),
        (info_x + 280, info_y_start + 8),
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )

    # --- PERSIAPAN DATA ---
    now = datetime.now()
    day_map = ["Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu", "Minggu"]

    lat = telemetry_data.get("latitude", 0)
    lon = telemetry_data.get("longitude", 0)
    sog_ms = telemetry_data.get("speed", 0)
    cog = telemetry_data.get("cog", 0)
    hdg = telemetry_data.get("heading", 0)

    # 1. Format GPS (N/S dan E/W)
    lat_dir = "N" if lat >= 0 else "S"
    lon_dir = "E" if lon >= 0 else "W"
    gps_str = f"{lat_dir} {abs(lat):.6f}, {lon_dir} {abs(lon):.6f}"

    # 2. Format SOG (km/jam dan knots)
    speed_kmh = sog_ms * 3.6
    speed_knots = sog_ms * 1.94384
    sog_str = f"{speed_kmh:.2f} km/jam ({speed_knots:.0f} kn)"

    # Daftar baris data
    lines = [
        f"DAY : {day_map[now.weekday()]}",
        f"DATE: {now.strftime('%d/%m/%Y')}",
        f"TIME: {now.strftime('%H:%M:%S')}",
        f"GPS : {gps_str}",
        f"SOG : {sog_str}",
        f"COG : {cog:.2f} deg",
        f"HDG : {hdg:.2f} deg",
    ]

    # --- KONFIGURASI LAYOUT ---
    # [MODIFIKASI] Ubah angka 20 menjadi 45 agar teks turun ke bawah garis
    y_pos = info_y_start + 28

    line_height = 20  # Jarak antar baris
    font_size = 0.5  # Ukuran font
    font_face = cv2.FONT_HERSHEY_DUPLEX

    # Loop menggambar setiap baris teks
    for line_text in lines:
        putText_with_shadow(
            frame, line_text, (info_x, y_pos), font_face, font_size, color_white
        )
        y_pos += line_height

    return frame
