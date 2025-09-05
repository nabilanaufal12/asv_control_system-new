# backend/main.py
# --- VERSI FINAL: Memastikan VisionService berjalan secara independen untuk operasi otonom ---

import json
import os

from flask import Flask, g, current_app

# 1. Impor instance socketio dari extensions.py
from navantara_backend.extensions import socketio

# 2. Impor semua komponen utama
from navantara_backend.core.asv_handler import AsvHandler
from navantara_backend.services.vision_service import VisionService
from navantara_backend.api.endpoints import api_blueprint


def create_app():
    """
    Membuat dan mengkonfigurasi instance aplikasi Flask (Application Factory).
    """
    app = Flask(__name__)

    # 🔹 Muat konfigurasi dari file config.json
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, "..", "..", "config", "config.json")
        with open(config_path, "r") as f:
            app.config["ASV_CONFIG"] = json.load(f)
        print("[Server] File 'config.json' berhasil dimuat.")
    except Exception as e:
        print(f"[Server] KRITIS: Gagal memuat config.json. Error: {e}")
        exit(1)

    # 🔹 Inisialisasi semua service
    asv_handler = AsvHandler(app.config["ASV_CONFIG"])
    vision_service = VisionService(app.config["ASV_CONFIG"], asv_handler)

    # 🔹 Simpan instance service ke dalam konteks aplikasi
    app.asv_handler = asv_handler
    app.vision_service = vision_service

    # 🔹 Sediakan service dalam konteks request
    @app.before_request
    def before_request():
        g.asv_handler = current_app.asv_handler
        g.vision_service = current_app.vision_service

    # 🔹 Daftarkan semua endpoint API
    app.register_blueprint(api_blueprint)

    # 🔹 Inisialisasi SocketIO dengan aplikasi Flask
    socketio.init_app(app, cors_allowed_origins="*")

    # --- PERUBAHAN UTAMA DI SINI ---
    # 🔹 Jalankan Vision Service secara langsung saat aplikasi dibuat.
    # Ini memastikan deteksi selalu aktif, bahkan tanpa GUI.
    print("🚀 Memulai layanan visi secara otomatis untuk mode otonom...")
    vision_service.start()  # Memanggil metode start dari VisionService
    # --------------------------------

    print("[Server] Konfigurasi aplikasi selesai.")
    return app


# Panggil create_app() di level global agar 'app' bisa diimpor oleh server
app = create_app()

if __name__ == "__main__":
    # Jalankan server normal
    print("🚀 Backend Server ASV (Mode Otonom) siap menerima koneksi...")
    socketio.run(
        app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True
    )
