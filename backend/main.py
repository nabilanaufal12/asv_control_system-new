# backend/main.py
# --- FINAL: Integrasi Flask + SocketIO + VisionService (stream video ke video.html) ---

import json
import os
from flask import Flask, g, current_app
from backend.extensions import socketio   # gunakan socketio dari extensions.py

from backend.core.asv_handler import AsvHandler
from backend.services.vision_service import VisionService
from backend.api.endpoints import api_blueprint


def create_app():
    """
    Membuat dan mengkonfigurasi instance aplikasi Flask.
    """
    app = Flask(__name__)

    # 🔹 Muat konfigurasi dari config.json
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config.json')
        with open(config_path, 'r') as f:
            app.config['ASV_CONFIG'] = json.load(f)
        print("[Server] File 'config.json' berhasil dimuat.")
    except Exception as e:
        print(f"[Server] KRITIS: Gagal memuat config.json. Error: {e}")
        exit(1)

    # 🔹 Buat handler utama + vision service, passing socketio
    asv_handler = AsvHandler(app.config['ASV_CONFIG'], socketio)
    vision_service = VisionService(app.config['ASV_CONFIG'], asv_handler, socketio=socketio)

    # 🔹 Simpan ke dalam app untuk akses global
    app.asv_handler = asv_handler
    app.vision_service = vision_service

    # 🔹 Inject ke request context (biar bisa diakses lewat g)
    @app.before_request
    def before_request():
        g.asv_handler = current_app.asv_handler
        g.vision_service = current_app.vision_service

    # 🔹 Daftarkan semua endpoint API
    app.register_blueprint(api_blueprint)

    # 🔹 Integrasi Flask ke dalam SocketIO
    socketio.init_app(app, cors_allowed_origins="*")

    # 🔹 Jalankan Vision Service (kamera & deteksi)
    print("🚀 Memulai layanan visi...")
    vision_service.start()

    return app


if __name__ == "__main__":
    # 🔹 Buat app & jalankan server dengan socketio
    app = create_app()
    print("🚀 Backend Server ASV aktif dengan dukungan WebSockets...")

    socketio.run(app, host="0.0.0.0", port=5000, debug=False)
