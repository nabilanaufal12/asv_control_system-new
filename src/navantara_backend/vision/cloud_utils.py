# src/navantara_backend/vision/cloud_utils.py
import requests
import time


def send_telemetry_to_firebase(telemetry_data, config):
    """Mengirim data telemetri ke Firebase Realtime Database."""
    try:
        FIREBASE_URL = config.get("api_urls", {}).get("firebase_url")
        if not FIREBASE_URL:
            # print("[Firebase] URL tidak diset di config, pengiriman dibatalkan.") # Opsional: uncomment untuk debug
            return

        # --- MODIFIKASI DIMULAI ---
        # Ambil data arena dan indeks waypoint dari state
        active_arena = telemetry_data.get("active_arena")
        # Kita kirim indeks waypoint yang sedang dituju (dimulai dari 0)
        # Tambah 1 agar sesuai ekspektasi monitor (dimulai dari 1)
        # Jika indeks >= jumlah waypoints, berarti selesai, kirim 0
        current_wp_index_zero_based = telemetry_data.get("current_waypoint_index", -1)
        total_waypoints = len(telemetry_data.get("waypoints", []))
        point_to_send = 0  # Default jika tidak ada waypoint atau sudah selesai
        if total_waypoints > 0 and current_wp_index_zero_based < total_waypoints:
            point_to_send = current_wp_index_zero_based + 1

        data_to_send = {
            "gps": {
                "lat": telemetry_data.get("latitude", 0.0),
                "lng": telemetry_data.get("longitude", 0.0),
            },
            "hdg": telemetry_data.get("heading", 0.0),
            "cog": telemetry_data.get("cog", 0.0),
            "sog": telemetry_data.get("speed", 0.0),  # speed dalam m/s
            "jam": time.strftime("%H:%M:%S"),
            "arena": active_arena if active_arena else "N/A",  # Kirim arena
            "point": point_to_send,  # Kirim indeks waypoint target (mulai dari 1)
        }
        # --- MODIFIKASI SELESAI ---

        # Kirim data menggunakan PUT (akan menimpa data lama di path tersebut)
        response = requests.put(
            FIREBASE_URL, json=data_to_send, timeout=5
        )  # Timeout 5 detik
        response.raise_for_status()  # Akan error jika status code bukan 2xx

        # print(f"[Firebase] Data terkirim: {data_to_send}") # Opsional: uncomment untuk debug

    except requests.exceptions.RequestException as e:
        # Tangani error koneksi atau timeout secara diam-diam
        # print(f"[Firebase] Gagal mengirim data: {e}") # Opsional: uncomment untuk debug
        pass
    except Exception as e:
        # Tangani error lainnya
        print(f"[Firebase] Error tidak terduga saat mengirim data: {e}")
        pass


def upload_image_to_supabase(image_buffer, filename, config):
    """Mengunggah buffer gambar ke Supabase Storage."""
    try:
        # --- PERUBAHAN DI SINI ---
        print(f"? [CAPTURE] Memulai proses unggah untuk '{filename}'...")
        api_config = config.get("api_urls", {})
        ENDPOINT_TEMPLATE, TOKEN = api_config.get("supabase_endpoint"), api_config.get(
            "supabase_token"
        )
        if not ENDPOINT_TEMPLATE or not TOKEN:
            print(f"? [CAPTURE] Gagal: Konfigurasi Supabase tidak ditemukan.")
            return

        ENDPOINT = ENDPOINT_TEMPLATE.replace("{filename}", filename)
        headers = {
            "Authorization": f"Bearer {TOKEN}",
            "Content-Type": "image/jpeg",
            "x-upsert": "true",  # Membuat file baru atau menimpa jika sudah ada
        }
        response = requests.post(
            ENDPOINT,
            headers=headers,
            data=image_buffer.tobytes(),
            timeout=10,  # Timeout 10 detik
        )
        if response.ok:
            print(f"? [CAPTURE] Berhasil! Gambar '{filename}' telah diunggah.")
        else:
            print(
                f"? [CAPTURE] Gagal mengunggah '{filename}': {response.status_code} {response.text}"
            )
        # --- AKHIR PERUBAHAN ---
    except Exception as e:
        print(f"? [CAPTURE] Error koneksi saat mengunggah: {e}")
