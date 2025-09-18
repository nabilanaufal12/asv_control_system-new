# src/navantara_backend/vision/cloud_utils.py
import requests
import time


def send_telemetry_to_firebase(telemetry_data, config):
    """Mengirim data telemetri ke Firebase Realtime Database."""
    try:
        FIREBASE_URL = config.get("api_urls", {}).get("firebase_url")
        if not FIREBASE_URL:
            return
        data_to_send = {
            "gps": {
                "lat": telemetry_data.get("latitude", 0.0),
                "lng": telemetry_data.get("longitude", 0.0),
            },
            "hdg": telemetry_data.get("heading", 0.0),
            "cog": telemetry_data.get("cog", 0.0),
            "sog": telemetry_data.get("speed", 0.0),
            "jam": time.strftime("%H:%M:%S"),
        }
        requests.put(FIREBASE_URL, json=data_to_send, timeout=5)
    except Exception:
        pass  # Gagal secara diam-diam agar tidak menghentikan loop utama


def upload_image_to_supabase(image_buffer, filename, config):
    """Mengunggah buffer gambar ke Supabase Storage."""
    try:
        # --- PERUBAHAN DI SINI ---
        print(f"📸 [CAPTURE] Memulai proses unggah untuk '{filename}'...")
        api_config = config.get("api_urls", {})
        ENDPOINT_TEMPLATE, TOKEN = api_config.get("supabase_endpoint"), api_config.get(
            "supabase_token"
        )
        if not ENDPOINT_TEMPLATE or not TOKEN:
            print(f"🔥 [CAPTURE] Gagal: Konfigurasi Supabase tidak ditemukan.")
            return

        ENDPOINT = ENDPOINT_TEMPLATE.replace("{filename}", filename)
        headers = {
            "Authorization": f"Bearer {TOKEN}",
            "Content-Type": "image/jpeg",
            "x-upsert": "true",
        }
        response = requests.post(
            ENDPOINT, headers=headers, data=image_buffer.tobytes(), timeout=10
        )
        if response.ok:
            print(f"✅ [CAPTURE] Berhasil! Gambar '{filename}' telah diunggah.")
        else:
            print(
                f"🔥 [CAPTURE] Gagal mengunggah '{filename}': {response.status_code} {response.text}"
            )
        # --- AKHIR PERUBAHAN ---
    except Exception as e:
        print(f"🔥 [CAPTURE] Error koneksi saat mengunggah: {e}")
