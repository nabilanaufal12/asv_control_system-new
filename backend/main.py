# backend/main.py
# --- MODIFIKASI: Mengimpor socketio dari extensions.py untuk mengatasi Circular Import ---

import json
import os
from flask import Flask, g, current_app
# 1. Impor instance socketio dari file extensions.py yang baru
from backend.extensions import socketio

from backend.core.asv_handler import AsvHandler
from backend.services.vision_service import VisionService
from backend.api.endpoints import api_blueprint

# (Baris 'socketio = SocketIO(...)' telah dihapus dari sini)

def create_app():
    """
    Membuat dan mengkonfigurasi instance aplikasi Flask.
    """
    app = Flask(__name__)

    # Memuat Konfigurasi dari file config.json
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config.json')
        with open(config_path, 'r') as f:
            app.config['ASV_CONFIG'] = json.load(f)
        print("[Server] File 'config.json' berhasil dimuat.")
    except Exception as e:
        print(f"[Server] KRITIS: Gagal memuat config.json. Error: {e}")
        exit()

    # Teruskan instance socketio yang sudah diimpor ke dalam AsvHandler
    asv_handler = AsvHandler(app.config['ASV_CONFIG'], socketio)
    vision_service = VisionService(app.config['ASV_CONFIG'], asv_handler)

    # Menyimpan instance service di 'app' untuk stabilitas
    app.asv_handler = asv_handler
    app.vision_service = vision_service

    @app.before_request
    def before_request():
        g.asv_handler = current_app.asv_handler
        g.vision_service = current_app.vision_service

    # Daftarkan semua rute API dari file endpoints.py
    app.register_blueprint(api_blueprint)
    
    # Inisialisasi aplikasi Flask ke dalam SocketIO
    socketio.init_app(app)
    
    # Mulai service background (kamera & deteksi)
    print("🚀 Memulai layanan visi...")
    vision_service.start()
    
    return app

if __name__ == "__main__":
    # Buat aplikasi menggunakan "pabrik" di atas
    app = create_app()
    print("🚀 Memulai Backend Server ASV dengan dukungan WebSockets...")
    
    # Jalankan server menggunakan socketio.run, bukan app.run
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)